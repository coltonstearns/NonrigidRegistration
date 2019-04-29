// 6.838 Final Project.cpp : This file contains the 'main' function. Program execution begins and ends there.

#include <iostream>
#include <vector>
#include <array>
#include <eigen3/Eigen/Dense>

#include <pcl/visualization/cloud_viewer.h>  // visualization tool
// #include <pcl/io/io.h>   // idk what this is for
#include <pcl/io/pcd_io.h>
#include <pcl/impl/point_types.hpp>
#include <pcl/features/normal_3d.h>

#include "parseTosca.h"
#include "fpfh.h"


Eigen::MatrixXd point_set_squared_distance(Eigen::MatrixXd X, Eigen::MatrixXd C);

int main()
{
    
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
    catData rawdata = generateCatPointCloud();
    // int ypoints = rawdata.baseline->size() / 3;
    // int xpoints = rawdata.transformed->size() / 3;
    // Eigen::MatrixXd ydata = Eigen::Map<Eigen::MatrixXd>(rawdata.baseline->data(), 3, ypoints).transpose();
    // Eigen::MatrixXd xdata = Eigen::Map<Eigen::MatrixXd>(rawdata.transformed->data(), 3, xpoints).transpose();

    // std::vector<pcl::PointXYZ> points;
    // for (int i=0; i < ypoints; i++){
    //     pcl::PointXYZ curr_point = pcl::PointXYZ(rawdata.baseline->at(3*i), rawdata.baseline->at(3*i+1), rawdata.baseline->at(3*i+2));
    //     points.push_back(curr_point);
    // }

    // // pcl::visualization::CloudViewer::MonochromeCloud<pcl::PointXYZ> cloud;
    // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    // cloud->width = ypoints; cloud->height = 1;
    // cloud->is_dense = true;
    // cloud->points.resize (cloud->width * cloud->height);
    // for (size_t i = 0; i < cloud->points.size (); ++i) {
    //     cloud->points[i] = points[i];
    // }
    
    // check computing normals
    pcl::PointCloud<pcl::Normal>::Ptr baseline_normals = getCloudNormals(rawdata.baseline);
    std::cout << baseline_normals->points.size() << std::endl; // should have the same size as the input cloud->points.size ()*
    std::cout << rawdata.baseline->points.size() << std::endl;

    // check computing fpfh for one cloud
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr data_features = computeFPFH(rawdata.baseline, baseline_normals, 5.0);
    std::cout << data_features->points.size() << std::endl;
    std::cout << data_features->points.at(0).descriptorSize << std::endl; // points to a struct with a 33 point histogram float[33]!
    // float histo[33];
    // for (int i = 0; i < 33; i++){
    //     histo[i] = data_features->points.at(0).histogram[i];
    //     std::cout << histo[i] << ", ";
    // }
    // std::cout << std::endl;
    // std::cout << data_features->points.at(0).descriptorSize << std::endl; // points to histogram object!



    // compute correspondences between the 2 cats
    // for (int i = 1; i <= 10; i++){
    //     pcl::Correspondences correspondences = calculateCorrespondences(rawdata.baseline, rawdata.transformed, float (i));
    // }

    pcl::Correspondences correspondences = calculateCorrespondences(rawdata.baseline, rawdata.transformed, 8.9);
    std::cout << correspondences.size() << endl;
    std::cout << correspondences.at(0) << endl;


    // visualize the point cloud
    pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer");
    // viewer.showCloud (rawdata.baseline);
    // while (!viewer.wasStopped ())
    // {
    // }

    viewer.showCloud (rawdata.transformed);
    while (!viewer.wasStopped ())
    {
    }
}


/*
Takes two matrices of (points by dims) and calculates the
squared Euclidean distance between them. 
This takes O(d*n*m), where d is dimensions, n is data points, and m is centers.
*/
Eigen::MatrixXd point_set_squared_distance(Eigen::MatrixXd X, Eigen::MatrixXd C) {

    const int ndata = X.rows();
    const int dimx = X.cols();
    const int ncenters = C.rows();
    const int dimc = C.cols();

    if (dimx != dimc) {
        throw std::runtime_error("Data dimension does not match the dimension of the centers");
    }

    Eigen::MatrixXd tempDataOnes = Eigen::MatrixXd::Ones(ndata, 1);
    Eigen::MatrixXd tempCentersOnes = Eigen::MatrixXd::Ones(ncenters, 1);
    Eigen::MatrixXd sumDataTemp = X.array().pow(2).transpose().colwise().sum();
    Eigen::MatrixXd sumCentersTemp = C.array().pow(2).transpose().colwise().sum();
    Eigen::MatrixXd firstTerm = tempCentersOnes * sumDataTemp;
    Eigen::MatrixXd secondTerm = tempDataOnes * sumCentersTemp;
    Eigen::MatrixXd thirdTerm = 2 * X * C.transpose();
    return firstTerm.transpose() + secondTerm - thirdTerm;

}
