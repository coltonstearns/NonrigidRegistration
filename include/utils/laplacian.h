#include <iostream>
#include <vector>
#include <eigen3/Eigen/Dense>
#include <math.h>
#include <stdio.h>

// PCL dependencies
#include <pcl/io/pcd_io.h>  // for working with point cloud data structures
#include <pcl/impl/point_types.hpp>  // for defining point objects
#include <pcl/point_types.h>
#include <pcl/registration/correspondence_estimation.h> // for calculating distance correspondences
#include <pcl/visualization/cloud_viewer.h>  // visualization tool


// Eigen Dependencies
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Sparse>


// Spectra Dependency
#include <Spectra/GenEigsSolver.h>
#include <Spectra/MatOp/SparseSymMatProd.h>


struct LaplaceEigenInfo {
    int neigs;
    Eigen::MatrixXf eigenvectors;
    Eigen::VectorXf eigenvalues;
};

struct Color {
    double r,g,b;
};

Eigen::SparseMatrix<float> getLaplacian(pcl::PointCloud<pcl::PointXYZ>::Ptr source, float epsilon, float sparsity_threshold);

LaplaceEigenInfo approximate_laplacian(Eigen::SparseMatrix<float> &laplacian, int neigs);

void visualize_laplacian(pcl::PointCloud<pcl::PointXYZ>::Ptr lap_points, Eigen::VectorXf eigenvector);

Color getColor(float value, float min_value, float max_value);
