#include <pcl/io/pcd_io.h>  // for working with point cloud data structures
#include <pcl/impl/point_types.hpp>  // for defining point objects
#include <pcl/features/normal_3d.h>
#include <pcl/point_types.h>
#include <pcl/features/fpfh.h>  // fpfh object
#include <pcl/registration/correspondence_estimation.h> // for calculating distance correspondences

// visualization script
#include <pcl/visualization/pcl_visualizer.h>  // visualizer 
#include <pcl/common/transforms.h>


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




/**
 * Helps visualise the correspondences we will use
 **/
void visualize_correspondences(pcl::PointCloud<pcl::PointXYZ>::Ptr source_raw, pcl::PointCloud<pcl::PointXYZ>::Ptr target_raw, pcl::Correspondences correspondences, float beta, int iter){
    // convert both clouds into RGB
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr source (new pcl::PointCloud<pcl::PointXYZRGB>);
    for (int i=0; i < source_raw->size(); i++){
        pcl::PointXYZRGB point = pcl::PointXYZRGB(255, 255, 255);
        point.x = source_raw->at(i).x; point.y = source_raw->at(i).y; point.z = source_raw->at(i).z;
        source->push_back(point);
    }

    // convert both clouds into RGB
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr target (new pcl::PointCloud<pcl::PointXYZRGB>);
    for (int i=0; i < source_raw->size(); i++){
        pcl::PointXYZRGB point = pcl::PointXYZRGB(255, 255, 255);
        point.x = target_raw->at(i).x; point.y = target_raw->at(i).y; point.z = target_raw->at(i).z;
        target->push_back(point);
    }
    
    // initialize visualization
    pcl::visualization::PCLVisualizer viscorr;
    std::cout << "HERE0" << std::endl;

    // add source cloud to visualizer
    viscorr.addPointCloud(source->makeShared(), "src_points");

    //I will translate the target cloud in the x direction for better viewing the correspondence
    // because my two clouds were //overlapping together you can ignore this if your clouds don't overlap
    // Add it to visualizer
    Eigen::Matrix4f t;
    // t<<1,0,0,100,
    //     0,1,0,0,
    //     0,0,1,0,
    //     0,0,0,1;

    t<<1,0,0,0,
        0,1,0,0,
        0,0,1,0,
        0,0,0,1;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr target_tranformed (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::transformPointCloud(*target, *target_tranformed, t);
    viscorr.addPointCloud(target_tranformed->makeShared(), "tgt_points");
    std::cout << "HERE" << std::endl;

    //I am using this boolean to alter the color of the spheres
    bool alter=false;
    for (size_t i = 0; i < correspondences.size(); i += 1)
    {

        // gives fpfh correspondences
        const pcl::PointXYZRGB & p_src = source->points.at(correspondences.at(i).index_query);
        pcl::PointXYZRGB & p_tgt = target->points.at(correspondences.at(i).index_match);

        // gives ground truth correspondences
        // const pcl::PointXYZRGB & p_src = source->points.at(i);
        // pcl::PointXYZRGB & p_tgt = target->points.at(i);

        // Generate a unique string for each line
        std::stringstream ss ("line");
        ss << i;
        //this is to translate the keypoint location as I translated the original cloud
        // p_tgt.x+=100;
            
        std::stringstream sss ("spheresource");
        sss << i;
    
        std::stringstream ssss ("spheretarget");
        ssss << i;
        if(alter)
        {
            //this is for red lines and spheres
            viscorr.addSphere<pcl::PointXYZRGB>(p_src,0.5,255,0,0,sss.str());
            viscorr.addSphere<pcl::PointXYZRGB>(p_tgt,0.5,255,0,0,ssss.str());
            viscorr.addLine<pcl::PointXYZRGB> (p_src, p_tgt, 50, 0, 155, ss.str ());      
        }
        else
        {
            //this is for yellow ones
            viscorr.addSphere<pcl::PointXYZRGB>(p_src,0.5,255,255,0,sss.str());
            viscorr.addSphere<pcl::PointXYZRGB>(p_tgt,0.5,255,255,0,ssss.str());
            viscorr.addLine<pcl::PointXYZRGB> (p_src, p_tgt, 0, 50, 155, ss.str ());
        }
        alter=!alter;
    }
    viscorr.resetCamera ();
    // viscorr.spin (); 

    std::ostringstream name;
    name << "correspondences" << beta << " " << iter << ".png";
    std::string str_name(name.str());
    viscorr.saveScreenshot(str_name);
}