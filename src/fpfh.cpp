#include <pcl/io/pcd_io.h>  // for working with point cloud data structures
#include <pcl/impl/point_types.hpp>  // for defining point objects
#include <pcl/features/normal_3d.h>
#include <pcl/point_types.h>
#include <pcl/features/fpfh.h>  // fpfh object
#include <pcl/registration/correspondence_estimation.h> // for calculating distance correspondences


/*
Computes and returns the normals of a point cloud
*/
pcl::PointCloud<pcl::Normal>::Ptr getCloudNormals(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud(cloud);

    // Create an empty kdtree representation, and pass it to the normal estimation object.
    // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
    ne.setSearchMethod (tree);

    // Output datasets
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);

    // Use all neighbors in a sphere of radius 3cm
    ne.setRadiusSearch (1.0);

    // Compute the features
    ne.compute (*cloud_normals);

    return cloud_normals;
    // cloud_normals->points.size () should have the same size as the input cloud->points.size ()*
}


pcl::PointCloud<pcl::FPFHSignature33>::Ptr computeFPFH(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr normals, float radius){
    // Create the FPFH estimation class, and pass the input dataset+normals to it
    pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh;
    fpfh.setInputCloud (cloud);
    fpfh.setInputNormals (normals);
    // alternatively, if cloud is of tpe PointNormal, do fpfh.setInputNormals (cloud);

    // Create an empty kdtree representation, and pass it to the FPFH estimation object.
    // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);

    fpfh.setSearchMethod (tree);

    // Output datasets
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhs (new pcl::PointCloud<pcl::FPFHSignature33> ());

    // Use all neighbors in a sphere of radius 5cm
    // IMPORTANT: the radius used here has to be larger than the radius used to estimate the surface normals!!!
    fpfh.setRadiusSearch (radius);

    // Compute the features
    fpfh.compute (*fpfhs);

    return fpfhs;
    // fpfhs->points.size () should have the same size as the input cloud->points.size ()*
}


/*
Calculates the full sample consensus correspondences
TODO: doesn't work. Make a subclass that calculates 
*/ 
pcl::Correspondences calculateCorrespondences(pcl::PointCloud<pcl::PointXYZ>::Ptr data, pcl::PointCloud<pcl::PointXYZ>::Ptr target, float fpfh_radius){
    // gets target cloud and computes its FPFH features
    pcl::PointCloud<pcl::Normal>::Ptr target_normals = getCloudNormals(target);
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr target_features = computeFPFH(target, target_normals, fpfh_radius); 

    // compute data cloud's FPFH features
    pcl::PointCloud<pcl::Normal>::Ptr data_normals = getCloudNormals(data);
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr data_features = computeFPFH(data, data_normals, fpfh_radius); 

    // compute correspondences between 2 data points using L2 distance
    pcl::registration::CorrespondenceEstimation<pcl::FPFHSignature33, pcl::FPFHSignature33> estimatate_correspondences;
    estimatate_correspondences.setInputSource (data_features);
    estimatate_correspondences.setInputTarget (target_features);
    pcl::Correspondences all_correspondences;
    estimatate_correspondences.determineReciprocalCorrespondences (all_correspondences); // Determine all reciprocal correspondences

    // return our correspondences
    return all_correspondences;

}
