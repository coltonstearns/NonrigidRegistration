// class header file
#include "alignment.h"


// ============================= Public Methods =================================

/**
 * Constructor. source and target are self explanatory.
 * nsamples represents how many points to be randomly sampled in computing the alignment (-1 means use all points)
 */
NonrigidAlign::NonrigidAlign(pcl::PointCloud<pcl::PointXYZ>::Ptr source,
 pcl::PointCloud<pcl::PointXYZ>::Ptr target, int npoints, int nsamples,
  float beta /*= .15*/, float epsilon /*= .15*/, float lambda1 /*= .4*/, float lambda2 /*= .4*/,
  float sparsity_threshold /*=.0000001*/) : npoints(npoints), nsamples(nsamples), beta(beta), epsilon(epsilon),
   lambda1(lambda1), lambda2(lambda2), sparsity_threshold(sparsity_threshold), transformed_X(Eigen::MatrixXf::Zero(npoints, dims)) {
    // set start by setting input values for source and target
    this->pcl_data.source = source;
    this->pcl_data.target = target;

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
    update_correspondences(tosca_fpfh_distance);

    // update pcl and eigen putative sets
    update_putative_sets();
};


/**
 * solve one iteration of the non-rigid alignment
 */
void NonrigidAlign::alignOneiter(int iteration) {
    // start by getting correspondences and sets
    // std::printf("-1\n");
    update_correspondences(tosca_fpfh_distance);
    update_putative_sets();

    // get laplacian and Tau
    // std::printf("0\n");
    this->laplacian = getLaplacian(pcl_data.source, epsilon, sparsity_threshold);
    cout << "/////////////////////////" << std::endl;
    cout << laplacian.nonZeros() << std::endl;
    // visualize_laplacian(pcl_data.source, laplacian, 5, 0);

    // create our ALS object for this iteration
    // std::printf("1\n");
    AlternatingLeastSquares als (eigen_data, laplacian, lambda1, lambda2, beta, volume, npoints,
     ncorrs, nsamples, iteration);

    // perform ALS for 5 iterations to get optimal C
    // std::printf("2\n");
    als.optimize(10);

    // update our values to T(X)
    // std::printf("3\n");
    // transform_source mutates this->transformed_X
    transform_source(als.C, als.X_downsampled, this->transformed_X, this->eigen_data.source, beta);

    // update source to be new source
    this->eigen_data.source = transformed_X;

    // update pcl source   
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    for (int i = 0; i < npoints; i++){
        pcl::PointXYZ point = pcl::PointXYZ(transformed_X(i,0), transformed_X(i,1), transformed_X(i,2));
        cloud->push_back(point);
    }
    this->pcl_data.source = cloud;

    // reset transformed_X to zeros
    transformed_X = Eigen::MatrixXf::Zero(npoints, dims);

    // update round number
    this->round += 1;

}


/**
 * Displays 3D view of correspondencess
 */
void NonrigidAlign::displayCorrespondences(){
    // pcl::Correspondences corrs;
    // for (int i = 0; i < ncorrs; i++){
    //     pcl::Correspondence corr; corr.index_match = i; corr.index_query = i;
    //     corrs.push_back(corr);
    // }
    visualize_correspondences(pcl_data.source, pcl_data.target, pcl_data.correspondences, beta, round);
}



// ============================= Private Methods ================================

/**
 * Rather inefficient for now, but I doubt it will matter much
 */
void NonrigidAlign::update_putative_sets() {
    // full data sets
    // Eigen::MatrixXf shuffled_source (npoints, dims);
    // Eigen::MatrixXf shuffled_target (npoints, dims);
    // pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_source (new pcl::PointCloud<pcl::PointXYZ>);
    // pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_target (new pcl::PointCloud<pcl::PointXYZ>);

    // putative sets
    eigen_data.putative_source.resize(ncorrs, dims);
    eigen_data.putative_target.resize(ncorrs, dims);
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_putative_source (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_putative_target (new pcl::PointCloud<pcl::PointXYZ>);

    // bool *source_corrs = new bool[npoints];
    // bool *target_corrs = new bool[npoints];

    // for (int i = 0; i < npoints; i++){
    //     source_corrs[i] = false;
    //     target_corrs[i] = false;
    // }

    for (int i = 0; i < ncorrs; i++){
        pcl::Correspondence corr = pcl_data.correspondences.at(i);
        // update eigen matrices
        eigen_data.putative_source(i,0) = pcl_data.source->at(corr.index_query).x;
        eigen_data.putative_source(i,1) = pcl_data.source->at(corr.index_query).y;
        eigen_data.putative_source(i,2) = pcl_data.source->at(corr.index_query).z;
        eigen_data.putative_target(i,0) = pcl_data.target->at(corr.index_match).x;
        eigen_data.putative_target(i,1) = pcl_data.target->at(corr.index_match).y;
        eigen_data.putative_target(i,2) = pcl_data.target->at(corr.index_match).z;

        // update pcl version
        pcl_putative_source->push_back(pcl_data.source->at(corr.index_query));
        pcl_putative_target->push_back(pcl_data.source->at(corr.index_match));

        // eigen_data.putative_source(i,0) = pcl_data.source->at(i*2).x;
        // eigen_data.putative_source(i,1) = pcl_data.source->at(i*2).y;
        // eigen_data.putative_source(i,2) = pcl_data.source->at(i*2).z;
        // eigen_data.putative_target(i,0) = pcl_data.target->at(i*2).x;
        // eigen_data.putative_target(i,1) = pcl_data.target->at(i*2).y;
        // eigen_data.putative_target(i,2) = pcl_data.target->at(i*2).z;

        // // update pcl version
        // pcl_putative_source->push_back(pcl_data.source->at(i*2));
        // pcl_putative_target->push_back(pcl_data.source->at(i*2));

    }

    // update putative pcl clouds
    pcl_data.putative_source = pcl_putative_source;
    pcl_data.putative_target = pcl_putative_target;



}


/**
 * Computes the FPFH correspondences and updates the inner rep to reflect
 * the newest correspondences.
 */
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