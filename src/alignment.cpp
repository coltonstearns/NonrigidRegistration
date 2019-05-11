// class header file
#include "alignment.h"


// ============================= Public Methods =================================

/**
 * Constructor. source and target are self explanatory.
 * num_samples represents how many points to be randomly sampled in computing the alignment (-1 means use all points)
 */
NonrigidAlign::NonrigidAlign(pcl::PointCloud<pcl::PointXYZ>::Ptr source,
 pcl::PointCloud<pcl::PointXYZ>::Ptr target, int npoints, int num_samples){
    // set start by setting input values for source and target
    this->pcl_data.source = source;
    this->pcl_data.target = target;
    this->npoints = npoints;

    // create eigen source and target
    Eigen::MatrixXf s (npoints, dims);
    Eigen::MatrixXf t (npoints, dims);
    for (int i = 0; i < npoints; i++){
        s(i,0) = pcl_data.source->at(i).x;
        s(i,1) = pcl_data.source->at(i).y;
        s(i,2) = pcl_data.source->at(i).z;
        t(i,0) = pcl_data.target->at(i).x;
        t(i,1) = pcl_data.target->at(i).y;
        t(i,2) = pcl_data.target->at(i).z;
    }
    this->eigen_data.source = s;
    this->eigen_data.target = t;

    // compute and update pcl and eigen correspondences 
    update_correspondences(8.9);  // 8.9

    // update pcl and eigen putative sets
    update_putative_sets();
};


/**
 * solve one iteration of the non-rigid alignment
 */
void NonrigidAlign::alignOneiter(float lambda1, float lambda2) {
    // start by getting correspondences and sets
    // std::printf("-1\n");
    update_correspondences(8.9);
    update_putative_sets();

    // get laplacian and Tau
    // std::printf("0\n");
    update_laplacian();
    cout << "/////////////////////////" << std::endl;
    cout << laplacian.nonZeros() << std::endl;
    update_Tau();

    // create our ALS object for this iteration
    // std::printf("1\n");
    AlternatingLeastSquares als (Tau, laplacian, eigen_data.putative_source,
     eigen_data.putative_target, lambda1, lambda2, volume, npoints,
     ncorrs);

    // perform ALS for 5 iterations to get optimal C
    // std::printf("2\n");
    als.optimize(1);

    // update our values to T(X)
    // std::printf("3\n");
    transform_source(als.C);

}


/**
 * Displays 3D view of correspondencess
 */
void NonrigidAlign::displayCorrespondences(){
    visualize_correspondences(pcl_data.source, pcl_data.target, pcl_data.correspondences);
}



// ============================= Private Methods ================================

/**
 * Rather inefficient for now, but I doubt it will matter much
 */
void NonrigidAlign::update_putative_sets() {
    Eigen::MatrixXf eig_putative_source(ncorrs, dims);
    Eigen::MatrixXf eig_putative_target(ncorrs, dims);
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_putative_source (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_putative_target (new pcl::PointCloud<pcl::PointXYZ>);
    
    for (int i = 0; i < ncorrs; i++){
        pcl::Correspondence corr = pcl_data.correspondences.at(i);
        // update eigen matrices
        eig_putative_source(i,0) = pcl_data.source->at(corr.index_query).x;
        eig_putative_source(i,1) = pcl_data.source->at(corr.index_query).y;
        eig_putative_source(i,2) = pcl_data.source->at(corr.index_query).z;
        eig_putative_target(i,0) = pcl_data.target->at(corr.index_match).x;
        eig_putative_target(i,1) = pcl_data.target->at(corr.index_match).y;
        eig_putative_target(i,2) = pcl_data.target->at(corr.index_match).z;

        // update pcl version
        pcl_putative_source->push_back(pcl_data.source->at(corr.index_query));
        pcl_putative_target->push_back(pcl_data.source->at(corr.index_match));
    }

    eigen_data.putative_source = eig_putative_source;
    eigen_data.putative_target = eig_putative_target;
    pcl_data.putative_source = pcl_putative_source;
    pcl_data.putative_target = pcl_putative_target;
}


void NonrigidAlign::update_laplacian(){
    // compute the laplacian
    this->laplacian = getLaplacian(pcl_data.source);

    // break laplacian down into eigenvecs and eigenvals and build up approximation
    // TODO
}

void NonrigidAlign::update_Tau(){
    Eigen::MatrixXf t = computeGramKernel(eigen_data.source, 0.1); //set beta = .1
    std::cout << "------------" << std::endl;
    std::cout << t(16,0) << std::endl;
    std::cout << t(17,0) << std::endl;
    std::cout << t(18,0) << std::endl;
    std::cout << t(19,0) << std::endl;
    std::cout << t(20,0) << std::endl;
    std::cout << t(21,0) << std::endl;
    Tau = t;
    // TODO Tau is mostly sparse --> should change this
}

// TODO: read over this and make it cleaner
void NonrigidAlign::transform_source(Eigen::MatrixXf C) {
    this->transformed_X = Eigen::MatrixXf::Zero(npoints, 3);
    for (int i = 0; i < npoints; i++){
        Eigen::Block<Eigen::MatrixXf, 1, -1, false> precomputed_kernel_vals = Tau.row(i);
        for (int j = 0; j < npoints; j++) {
            // Kernel(x_i, x_{all}) * C_{all} to get the transformed point T(x_i)
            if (precomputed_kernel_vals(j) > 0.000001){
                this->transformed_X.row(i) += C.row(i) * (precomputed_kernel_vals(j) * Eigen::MatrixXf::Identity(3,3)); // results in 1x3
            }
        }
    }    
}

void NonrigidAlign::update_correspondences(float fpfh_distance){
    // compute correspondences and update PCL form
    pcl_data.correspondences = calculateCorrespondences(pcl_data.source, pcl_data.target, fpfh_distance);
    
    // update Eigen form
    ncorrs = pcl_data.correspondences.size();
    eigen_data.correspondences = Eigen::MatrixXf(ncorrs, 2);
    for (int i = 0; i < ncorrs; i++){
        pcl::Correspondence corr = pcl_data.correspondences.at(i);
        eigen_data.correspondences(i,0) = corr.index_query;
        eigen_data.correspondences(i,1) = corr.index_match;    
    }
}