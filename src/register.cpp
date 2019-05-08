#include <iostream>
#include <cerrno>
#include <cstring>
#include <cfenv>
#include "register.h"


void getLaplacian(pcl::PointCloud<pcl::PointXYZ>::Ptr source, Eigen::SparseMatrix<float> laplacian){
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
    laplacian = vertex_vals - edge_weights_sparse;

    time_t end_time;
	end_time = time(NULL);
    std::cout << "Time Elapsed: " << end_time - start_time << std::endl;
    std::cout << "Num Nonzero entries in Laplacian " << laplacian.nonZeros() << std::endl;
}


/*
HELLA SLOW!
Takes two matrices of (points by dims) and calculates the
squared Euclidean distance between them. 
This takes O(d*n*m), where d is dimensions, n is data points, and m is centers.
*/
Eigen::MatrixXf point_set_squared_distance(Eigen::MatrixXf X, Eigen::MatrixXf C) {

    const int ndata = X.rows();
    const int dimx = X.cols();
    const int ncenters = C.rows();
    const int dimc = C.cols();

    if (dimx != dimc) {
        throw std::runtime_error("Data dimension does not match the dimension of the centers");
    }

    Eigen::MatrixXf tempDataOnes = Eigen::MatrixXf::Ones(ndata, 1);
    Eigen::MatrixXf tempCentersOnes = Eigen::MatrixXf::Ones(ncenters, 1);
    Eigen::MatrixXf sumDataTemp = X.array().pow(2).transpose().colwise().sum();
    Eigen::MatrixXf sumCentersTemp = C.array().pow(2).transpose().colwise().sum();
    Eigen::MatrixXf firstTerm = tempCentersOnes * sumDataTemp;
    Eigen::MatrixXf secondTerm = tempDataOnes * sumCentersTemp;
    Eigen::MatrixXf thirdTerm = 2 * X * C.transpose();
    return firstTerm.transpose() + secondTerm - thirdTerm;

}















// ===============================================================================================
// ===============================================================================================
// ================================= NOT IMPLEMENTED YET =========================================
// ===============================================================================================
// ===============================================================================================


/*
Computes the non-rigid registration of two point clouds given point correspondences.
*/
void transform(pcl::PointCloud<pcl::PointXYZ>::Ptr source, pcl::PointCloud<pcl::PointXYZ>::Ptr target,
 pcl::Correspondences correspondences, pcl::PointCloud<pcl::PointXYZ>::Ptr transformed){
    // define our ratio epsilon
    float epsilon = .15;  //.1 is optimal to get about 25 neighbors per point!
    float max_threshold = - log(.000001) * epsilon;

    // get time
    time_t start_time;
	start_time = time(NULL);
    std::cout << "Start Time: " << start_time << std::endl;
    
    // get the matrix of the source points
    // Eigen::Map< Eigen::MatrixXf, Eigen::Aligned, Eigen::OuterStride<>> source_points =  source->getMatrixXfMap();
    int num_source_points = int (source->width);
    std::cout << num_source_points << std::endl;

    // compute symmetric x differences matrix
    // TODO: Allocate Blocks in creation because random insertion is slow!
    // Eigen::SparseMatrix<float> edge_weights_sparse(num_source_points,num_source_points); 
    int k = 0;
    Eigen::MatrixXf edge_weights(num_source_points,num_source_points);
    float curr_weight;
    for (int i = 0; i < num_source_points; i++) {
        if (i % 1000 == 0){
            std::cout << "At " << i << " out of " << num_source_points << std::endl;
        }
        for (int j = 0; j <= i; j++){
            float dist = pow((source->at(i).x - source->at(j).x), 2) + pow((source->at(i).y - source->at(j).y), 2) + pow((source->at(i).z - source->at(j).z), 2);
            if (dist < max_threshold){
                if (i != j){
                    curr_weight = exp(- dist / epsilon);
                    edge_weights(i,j) = curr_weight;
                    edge_weights(j,i) = curr_weight;
                    k++;
                }
            }
            //     else {
            //         edge_weights_sparse.insert(i,j) = weight;
            //         edge_weights_sparse.insert(j,i) = weight;
            //     }
            // }
        }
    }
    Eigen::SparseMatrix<float> edge_weights_sparse = edge_weights.sparseView();
    std::cout << "Weight Matrix Num Nonzero Entries: " << edge_weights_sparse.nonZeros() << std::endl;

    // compute diagonal vertex weight matrix
    // Eigen::DiagonalMatrix<float, Eigen::Dynamic, Eigen::Dynamic> vertex_vals (num_source_points);
    Eigen::SparseMatrix<float> vertex_vals(num_source_points,num_source_points);
    for (int i = 0; i < num_source_points; i++) {
        float vertex_weight = edge_weights_sparse.col(i).sum();
        vertex_vals.insert(i,i) = vertex_weight;
    }
    // Eigen::DiagonalMatrix<float, Eigen::Dynamic> vertex_vals (num_source_points);  // how to define a diagonal matrix
    // vertex_vals.diagonal() = vertex_vals_vec;


    // calculate Laplacian
    Eigen::SparseMatrix<float> A (num_source_points, num_source_points);
    // Eigen::MatrixXf A(num_source_points,num_source_points);
    A = vertex_vals - edge_weights_sparse;

    time_t end_time;
	end_time = time(NULL);
    std::cout << "Time Elapsed: " << end_time - start_time << std::endl;
    std::cout << "Num Nonzero entries in Laplacian " << A.nonZeros() << std::endl;

    // Eigen::Matrix<float, Eigen::Dynamic, 1> source_xs = 
    // Map<MatrixXf,0,OuterStride<> > M2(M1.data(), M1.rows(), (M1.cols()+2)/3, OuterStride<>(M1.outerStride()*3));
    
}
