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

    // create our ALS object for this iteration
    // std::printf("1\n");
    AlternatingLeastSquares als (eigen_data, laplacian, lambda1, lambda2, volume, npoints,
     ncorrs, num_samples);

    // perform ALS for 5 iterations to get optimal C
    // std::printf("2\n");
    als.optimize(2);

    // update our values to T(X)
    // std::printf("3\n");
    transform_source(als.C, als.Tau);

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


// TODO: read over this and make it cleaner
void NonrigidAlign::transform_source(Eigen::MatrixXf C, Eigen::MatrixXf subsampled_tau) {
    this->transformed_X = Eigen::MatrixXf::Zero(npoints, 3);
    for (int i = 0; i < npoints; i++){
        Eigen::Block<Eigen::MatrixXf, 1, -1, false> precomputed_kernel_vals = subsampled_tau.row(i);
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



// ========================= OTHER METHODS ===============================

Eigen::SparseMatrix<float> getLaplacian(pcl::PointCloud<pcl::PointXYZ>::Ptr source){
    // define our ratio epsilon
    float epsilon = .15;  //.1 is optimal to get about 25 neighbors per point!
    float max_threshold = - log(.000001) * epsilon;

    // get time
    time_t start_time;
	start_time = time(NULL);
    
    // get the matrix of the source points
    int num_source_points = int (source->width);
    std::cout << "Number of data points: " << num_source_points << std::endl;

    // compute symmetric x differences matrix
    // SPEED UP: search through KD tree and get k nearest neighbors; compute graph laplacian based on that
    Eigen::MatrixXf edge_weights(num_source_points,num_source_points);
    float curr_weight;
    for (int i = 0; i < num_source_points; i++) {
        for (int j = 0; j <= i; j++){
            float dist = pow((source->at(i).x - source->at(j).x), 2) + pow((source->at(i).y - source->at(j).y), 2) + pow((source->at(i).z - source->at(j).z), 2);
            if (dist < max_threshold){
                if (i != j){
                    curr_weight = exp(- dist / epsilon);
                    edge_weights(i,j) = curr_weight;
                    edge_weights(j,i) = curr_weight;
                }
            }
        }
    }
    Eigen::SparseMatrix<float> edge_weights_sparse = edge_weights.sparseView();
    std::cout << "Weight Matrix Num Nonzero Entries: " << edge_weights_sparse.nonZeros() << std::endl;

    // compute diagonal vertex weight matrix
    Eigen::SparseMatrix<float> vertex_vals(num_source_points,num_source_points);
    for (int i = 0; i < num_source_points; i++) {
        float vertex_weight = edge_weights_sparse.col(i).sum();
        vertex_vals.insert(i,i) = vertex_weight;
    }

    // calculate Laplacian
    Eigen::SparseMatrix<float> laplacian = vertex_vals - edge_weights_sparse;

    time_t end_time;
	end_time = time(NULL);
    std::cout << "Time Elapsed: " << end_time - start_time << std::endl;
    std::cout << "Num Nonzero entries in Laplacian " << laplacian.nonZeros() << std::endl;

    return laplacian;
}
