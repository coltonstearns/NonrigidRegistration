// 6.838 Final Project.cpp : This file contains the 'main' function. Program execution begins and ends there.


// #include <vector>
// #include <array>
// #include <eigen3/Eigen/Dense>

// #include <pcl/visualization/cloud_viewer.h>  // visualization tool
// // #include <pcl/io/io.h>   // idk what this is for
// #include <pcl/io/pcd_io.h>
// #include <pcl/impl/point_types.hpp>
// #include <pcl/features/normal_3d.h>

#include <iostream>
#include <stdio.h>
#include <time.h>

#include "alignment.h"

int main(){
    time_t start;
    time_t end;
	start = time(NULL);

    // load data and initialize
    NonrigidAlign test; // initializes as test() in python
    end = time(NULL);
    std::printf("Loading Data Took %lld seconds.\n", (long long) end-start);
    start = end;

    // get correspondences
    test.getCorrespondences();
    end = time(NULL);
    std::printf("Computing Correspondences Took %lld seconds.\n", (long long) end-start);
    start = end;

    // copy all point cloud data to eigen
    test.generateEigenMatrices();
    end = time(NULL);
    std::printf("Creating Putative Sets Took %lld seconds.\n", (long long) end-start);
    start = end;

    // compute graph laplacian
    test.computeLaplacian();
    end = time(NULL);
    std::printf("Computing Graph Laplacian of entire Source Took %lld seconds.\n", (long long) end-start);
    start = end;  

    // display our correspondences
    // test.displayCorrespondences();

    // compute our kernel matrix!
    test.alignOneiter(.1, .1);
}















// =============================================================================
// =============================================================================
// =========================== Old Version of Main =============================
// =============================================================================
// =============================================================================


// int main()
// {
    
    // =============== Test point_set_squared_distance ==================
    //Eigen::Matrix<double, 4, 2> x;
    //x << 1, 2, 3, 4, 5, 6, 7, 8;
    //std::cout << x << std::endl;

    //Eigen::Matrix<double, 3, 2> c;
    //c << 1, 1, 2, 2, 3, 3;

    //Eigen::MatrixXd A = point_set_squared_distance(x, c);
    //std::cout << A << std::endl;

    // ===================== Test parseToscaData =========================
    //wchar_t test_dir[] = L"C:\\Users\\cstea\\Documents\\6.838 Final Project\\6.838 Final Project\\Datasets\\*"; //because is array, is automatically a pointer
    //std::vector<std::array<double, 3>> data = parseToscaData(test_dir);

    // ===================== Test generateCatPointCloud ==================
    // catData rawdata = generateCatPointCloud();
    // int ypoints = rawdata.baseline->size() / 3;
    // int xpoints = rawdata.transformed->size() / 3;
    // Eigen::MatrixXd ydata = Eigen::Map<Eigen::MatrixXd>(rawdata.baseline->data(), 3, ypoints).transpose();
    // Eigen::MatrixXd xdata = Eigen::Map<Eigen::MatrixXd>(rawdata.transformed->data(), 3, xpoints).transpose();

    // ===================== Test Creating Own Cloud =====================
    // std::vector<pcl::PointXYZ> points;
    // for (int i=0; i < ypoints; i++){
    //     pcl::PointXYZ curr_point = pcl::PointXYZ(rawdata.baseline->at(3*i), rawdata.baseline->at(3*i+1), rawdata.baseline->at(3*i+2));
    //     points.push_back(curr_point);
    // }

    // // Example of Visualization
    // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    // cloud->width = ypoints; cloud->height = 1;
    // cloud->is_dense = true;
    // cloud->points.resize (cloud->width * cloud->height);
    // for (size_t i = 0; i < cloud->points.size (); ++i) {
    //     cloud->points[i] = points[i];
    // }
    
    // ===================== Test Computing Correspondences =====================
    // pcl::Correspondences correspondences = calculateCorrespondences(rawdata.source, rawdata.target, 8.9); // 8.9 is optimal value
    // std::cout << correspondences.size() << endl;
    // std::cout << correspondences.at(0) << endl;  // is a correspondence struct from PCL library

    // ============================== Visualize =================================
    // visualize_correspondences(rawdata.baseline, rawdata.transformed, correspondences);

    // ====================== Test Laplacian Matrix Generation ===================
    // pcl::PointCloud<pcl::PointXYZ>::Ptr transformed (new pcl::PointCloud<pcl::PointXYZ>);
    // transform(rawdata.source, rawdata.target, correspondences, transformed);
// }