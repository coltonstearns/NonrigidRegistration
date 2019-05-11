#include <iostream>
#include <vector>
#include <math.h>
#include <time.h>

// PCL dependencies
#include <pcl/io/pcd_io.h>  // for working with point cloud data structures
#include <pcl/impl/point_types.hpp>  // for defining point objects
#include <pcl/point_types.h>

// Eigen Dependencies
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Sparse>

#include "fpfh.h"  // my implementation of fpfh


void transform(pcl::PointCloud<pcl::PointXYZ>::Ptr source, pcl::PointCloud<pcl::PointXYZ>::Ptr target,
 pcl::Correspondences correspondences, pcl::PointCloud<pcl::PointXYZ>::Ptr transformed);

Eigen::SparseMatrix<float> getLaplacian(pcl::PointCloud<pcl::PointXYZ>::Ptr source);
Eigen::MatrixXf point_set_squared_distance(Eigen::MatrixXf X, Eigen::MatrixXf C);
