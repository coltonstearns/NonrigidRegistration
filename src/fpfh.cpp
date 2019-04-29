#include <pcl/io/pcd_io.h>  // for working with point cloud data structures
#include <pcl/impl/point_types.hpp>  // for defining point objects
#include <pcl/features/normal_3d.h>
#include <pcl/point_types.h>
#include <pcl/features/fpfh.h>  // fpfh object
#include <pcl/registration/ia_ransac.h>  // for sample consensus alignment object
#include <pcl/common/transforms.h>  // for transforming point cloud to target with 6 degree of freedom matrix
#include <pcl/registration/correspondence_estimation.h> // for calculating distance correspondences

#include <pcl/visualization/cloud_viewer.h>  // visualization tool


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
    // define sample parameters
    float min_sample_distance_ = 1.0f;  // distance between sample points that we are taking on our cat: need to know dims of cat
    float max_correspondence_distance_ = 0.1f;  // max distance at which point we don't count it as a correspondence to be randomly sampled from
    int nr_iterations_ = 200;  // how many models we check; we want a large number to eventually find a good correspondence
    int nr_samples = 10;  // 10 samples randomly matched


    // set up sample consensus alignment object
    // sets default of 10 neighbors for fpfh
    // std::cout << "Checkpoint 1" << std::endl;
    pcl::SampleConsensusInitialAlignment<pcl::PointXYZ, pcl::PointXYZ, pcl::FPFHSignature33> initial_alignment;
    initial_alignment.setMinSampleDistance (min_sample_distance_);
    initial_alignment.setMaxCorrespondenceDistance (max_correspondence_distance_);
    initial_alignment.setMaximumIterations (nr_iterations_);
    initial_alignment.setNumberOfSamples (nr_samples);

    // get target cloud and set it
    // std::cout << "Checkpoint 2" << std::endl;
    pcl::PointCloud<pcl::Normal>::Ptr target_normals = getCloudNormals(target);
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr target_features = computeFPFH(target, target_normals, fpfh_radius); 
    initial_alignment.setInputTarget(target);
    initial_alignment.setTargetFeatures(target_features);

    // get data cloud and set it
    // std::cout << "Checkpoint 3" << std::endl;
    pcl::PointCloud<pcl::Normal>::Ptr data_normals = getCloudNormals(data);
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr data_features = computeFPFH(data, data_normals, fpfh_radius); 
    initial_alignment.setInputCloud(data);
    initial_alignment.setSourceFeatures(data_features);

    // compute correspondences between 2 data points using L2 distance
    pcl::registration::CorrespondenceEstimation<pcl::FPFHSignature33, pcl::FPFHSignature33> estimatate_correspondences;
    estimatate_correspondences.setInputSource (data_features);
    estimatate_correspondences.setInputTarget (target_features);
    pcl::Correspondences all_correspondences;
    // std::cout << "Checkpoint 5.1" << std::endl;
    estimatate_correspondences.determineReciprocalCorrespondences (all_correspondences); // Determine all reciprocal correspondences


    // // compute best alignment and align the point clouds rigidly
    // // std::cout << "Checkpoint 4" << std::endl;
    // pcl::PointCloud<pcl::PointXYZ> registration_output;
    // // std::cout << "Checkpoint 4.1" << std::endl;
    // initial_alignment.align (registration_output);  // SUPER FRICKIN SLOW!
    // // std::cout << "Checkpoint 4.2" << std::endl;
    // Eigen::Matrix4f final_transformation = initial_alignment.getFinalTransformation ();
    // pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    // pcl::transformPointCloud(*data, *transformed_cloud, final_transformation);

    // // display score
    // float fitness_score = (float) initial_alignment.getFitnessScore (max_correspondence_distance_);
    // std::cout << "The Score of " << fpfh_radius << " is " << fitness_score << std::endl;
    
    // // get correspondences based on closest point in opposite cloud
    // // std::cout << "Checkpoint 5" << std::endl;
    // pcl::registration::CorrespondenceEstimation<pcl::PointXYZ, pcl::PointXYZ> estimatate_correspondences;
    // estimatate_correspondences.setInputSource (transformed_cloud);
    // estimatate_correspondences.setInputTarget (target);
    // pcl::Correspondences all_correspondences;
    // // std::cout << "Checkpoint 5.1" << std::endl;
    // estimatate_correspondences.determineReciprocalCorrespondences (all_correspondences); // Determine all reciprocal correspondences
    // // std::cout << "Checkpoint 5.2" << std::endl;

    // // visualize the transformation!
    // pcl::visualization::CloudViewer viewer ("Rigid Transformation");
    // viewer.showCloud (transformed_cloud);
    // while (!viewer.wasStopped ())
    // {
    // }

    // return our correspondences
    return all_correspondences;

}

// /*
// Computes the similar features following paper outlined for FPFH
// */
// void findSimilarFeatures (const pcl::PointCloud<pcl::FPFHSignature33> &input_features,
// const std::vector<int> &sample_indices, std::vector<int> &corresponding_indices)
// {
//     int k_correspondences_ = 20;
//     std::vector<int> nn_indices (k_correspondences_);
//     std::vector<float> nn_distances (k_correspondences_);

//     corresponding_indices.resize (sample_indices.size ());
//     for (size_t i = 0; i < sample_indices.size (); ++i)
//     {
//         // Find the k features nearest to input_features.points[sample_indices[i]]
//         feature_tree_->nearestKSearch (input_features, sample_indices[i], k_correspondences_, nn_indices, nn_distances);

//         // Select one at random and add it to corresponding_indices
//         int random_correspondence = getRandomIndex (k_correspondences_);
//         corresponding_indices[i] = nn_indices[random_correspondence];
//     }
// }
