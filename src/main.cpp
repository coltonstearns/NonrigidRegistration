// 6.838 Final Project.cpp : This file contains the 'main' function. Program execution begins and ends there.


// #include <vector>
// #include <array>
// #include <eigen3/Eigen/Dense>

#include <pcl/visualization/cloud_viewer.h>  // visualization tool
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

    // load wolf data
    catData wolfdata = generateCatPointCloud();
    int npoints = wolfdata.source->width;

    // initialize alignment object
    NonrigidAlign test (wolfdata.source, wolfdata.target, npoints, -1); 
    end = time(NULL);
    std::printf("Loading Data Took %lld seconds.\n", (long long) end-start);
    start = end;

    // run registration
    // lambda .1 .1 == blob of garbage
    // lambda .05 .1 == ?
    // lambda .01 .1 == ?
    // lambda .1 3 == ?
    // lambda .1 100 == 
    // lambda .001, 1 == negative infinity --> check this for debugging!
    test.alignOneiter(.001, 1); // set lambda1 = lambda2 = .1
    end = time(NULL);
    std::printf("Full Pipeline for first alignment took %lld seconds.\n", (long long) end-start);
    start = end;

    // display our correspondences
    // test.displayCorrespondences();

    // compute our kernel matrix!
    pcl::PointCloud<pcl::PointXYZ>::Ptr vis (new pcl::PointCloud<pcl::PointXYZ>);
    for (int i = 0; i < npoints; i++){
        pcl::PointXYZ p (test.transformed_X(i,0), test.transformed_X(i,1), test.transformed_X(i,2));
        vis->push_back(p);
        cout << p << std::endl;
    } 
    pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer Transformed");
    viewer.showCloud (vis);
    while (!viewer.wasStopped ())
    {
    }

    // Visualize original source
    pcl::visualization::CloudViewer viewer2 ("Simple Cloud Viewer Source");
    viewer2.showCloud (wolfdata.source);
    while (!viewer2.wasStopped ())
    {
    }

    // visualize target
    pcl::visualization::CloudViewer viewer3 ("Simple Cloud Viewer Target");
    viewer3.showCloud (wolfdata.target);
    while (!viewer3.wasStopped ())
    {
    }
}

