// 6.838 Final Project.cpp : This file contains the 'main' function. Program execution begins and ends there.

#include <pcl/visualization/cloud_viewer.h>  // visualization tool
#include <eigen3/Eigen/Eigenvalues>

#include <iostream>
#include <stdio.h>
#include <time.h>

#include "alignment.h"
// #include "utils/laplacian.h"
#include "parseTosca.h"

void visualize_result(pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_source_raw, pcl::PointCloud<pcl::PointXYZ>::Ptr target_raw, float beta, float iter);

void visualize_result2(pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_source_raw, pcl::PointCloud<pcl::PointXYZ>::Ptr target_raw, float beta, float iter);


int main(){
    time_t start;
    time_t end;
	start = time(NULL);

    // load wolf data
    catData wolfdata = generateCatPointCloud();
    int npoints = wolfdata.source->width;
    visualize_result(wolfdata.source, wolfdata.target, .00004, -1);


    // test laplacian
    // Eigen::SparseMatrix<float> laplacian = getLaplacian(wolfdata.source, 15, 1);
    // Eigen::VectorXf result = laplacian * Eigen::VectorXf::Ones(npoints);
    // LaplaceEigenInfo info = approximate_laplacian(laplacian, 5);
    // visualize_laplacian(wolfdata.source, info.eigenvectors.col(0));
    // visualize_laplacian(wolfdata.source, info.eigenvectors.col(1));
    // visualize_laplacian(wolfdata.source, info.eigenvectors.col(2));
    // visualize_laplacian(wolfdata.source, info.eigenvectors.col(3));
    // visualize_laplacian(wolfdata.source, info.eigenvectors.col(4));



    // Eigen::EigenSolver<Eigen::MatrixXf> eigen_info;
    // eigen_info.compute(laplacian.toDense(), true);
    // Eigen::MatrixXf eig_vecs = eigen_info.eigenvectors().real();
    // Eigen::VectorXf eig_vals = eigen_info.eigenvalues().real();
    // std::cout << "First Eigenvalue: " << eig_vals(0) << std::endl;
    // visualize_laplacian(wolfdata.source, eig_vecs.col(0));
    
    // betas = [.000005, .00005, .0005, .005, .05]
    // initialize alignment object
    float betas[] = {.000005, .00005, .0005, .005, .05};
    // float beta = .00005;//.5;  // e^{-beta} --> variance of 1
    // float variance = 1/(2*beta);
    float epsilon = 1;  // e^{-1/epsilon} --> variance of 15
    float lambda1 = .05;//0.05;
    float lambda2 = .4;
    int num_samples = 100;
    float sparse_threshold = .000025;

    for (int k = 0; k < 1; k++){
        // float beta = betas[k];
        float beta = .00004;
        NonrigidAlign test (wolfdata.source, wolfdata.target, npoints, num_samples, beta, epsilon, lambda1, lambda2, sparse_threshold); 
        end = time(NULL);
        std::printf("Loading Data Took %lld seconds.\n", (long long) end-start);
        start = end;


        // run registration
        for (int i = 0; i < 5; i++){
            test.displayCorrespondences();
            test.alignOneiter(i);
            end = time(NULL);
            std::printf("Full Pipeline for first alignment took %lld seconds.\n", (long long) end-start);
            start = end;

            // Visualize Entire Output
            pcl::PointCloud<pcl::PointXYZ>::Ptr transformed (new pcl::PointCloud<pcl::PointXYZ>);
            for (int i = 0; i < npoints; i++){
                pcl::PointXYZ p (test.eigen_data.source(i,0), test.eigen_data.source(i,1), test.eigen_data.source(i,2));
                transformed->push_back(p);
            } 
            visualize_result(transformed, wolfdata.target, beta, i);

            visualize_result2(transformed, wolfdata.target, beta, i);
        }
    }

}



//=================================================================================================================================
//=================================================================================================================================
//================================================== VISUALIZATION TOOLS===========================================================
//=================================================================================================================================
//=================================================================================================================================

/**
 * Helps visualise the correspondences we will use
 **/
void visualize_result(pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_source_raw, pcl::PointCloud<pcl::PointXYZ>::Ptr target_raw, float beta, float iter){
    int npoints = target_raw->size();

    // convert both clouds into RGB
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr source (new pcl::PointCloud<pcl::PointXYZRGB>);
    for (int i=0; i < npoints; i++){
        pcl::PointXYZRGB point = pcl::PointXYZRGB(0, 0, 255);
        point.x = transformed_source_raw->at(i).x; point.y = transformed_source_raw->at(i).y; point.z = transformed_source_raw->at(i).z;
        source->push_back(point);
    }

    // convert both clouds into RGB
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr target (new pcl::PointCloud<pcl::PointXYZRGB>);
    for (int i=0; i < npoints; i++){
        pcl::PointXYZRGB point = pcl::PointXYZRGB(255, 0, 0);
        point.x = target_raw->at(i).x; point.y = target_raw->at(i).y; point.z = target_raw->at(i).z;
        target->push_back(point);
    }
    
    // initialize visualization
    pcl::visualization::PCLVisualizer viscorr;

    // add both clouds to visualizer
    viscorr.addPointCloud(source->makeShared(), "src_points");
    viscorr.addPointCloud(target->makeShared(), "tgt_points");

   for (size_t i = 0; i < npoints; i += 1)
    {

        const pcl::PointXYZRGB & p_src = source->points.at(i);
        pcl::PointXYZRGB & p_tgt = target->points.at(i);

        // Generate a unique string for each line
        std::stringstream sss ("spheresource");
        sss << i;
        std::stringstream ssss ("spheretarget");
        ssss << i;

        //this is for red lines and spheres
        viscorr.addSphere<pcl::PointXYZRGB>(p_src,0.4,0,0,255,sss.str());
        // viscorr.addSphere<pcl::PointXYZRGB>(p_tgt,0.4,255,0,0,ssss.str());
    }

    viscorr.resetCamera ();  // sets to default view
    // viscorr.spin (); // runs the window

    std::ostringstream name;
    name << "final" << beta << " " << iter << ".png";
    std::string str_name(name.str());
    viscorr.saveScreenshot(str_name);

}




/**
 * Helps visualise the correspondences we will use
 **/
void visualize_result2(pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_source_raw, pcl::PointCloud<pcl::PointXYZ>::Ptr target_raw, float beta, float iter){
    int npoints = target_raw->size();

    // convert both clouds into RGB
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr source (new pcl::PointCloud<pcl::PointXYZRGB>);
    for (int i=0; i < npoints; i++){
        pcl::PointXYZRGB point = pcl::PointXYZRGB(0, 0, 255);
        point.x = transformed_source_raw->at(i).x; point.y = transformed_source_raw->at(i).y; point.z = transformed_source_raw->at(i).z;
        source->push_back(point);
    }

    // convert both clouds into RGB
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr target (new pcl::PointCloud<pcl::PointXYZRGB>);
    for (int i=0; i < npoints; i++){
        pcl::PointXYZRGB point = pcl::PointXYZRGB(255, 0, 0);
        point.x = target_raw->at(i).x; point.y = target_raw->at(i).y; point.z = target_raw->at(i).z;
        target->push_back(point);
    }
    
    // initialize visualization
    pcl::visualization::PCLVisualizer viscorr;

    // add both clouds to visualizer
    viscorr.addPointCloud(source->makeShared(), "src_points");
    viscorr.addPointCloud(target->makeShared(), "tgt_points");

    //I am using this boolean to alter the color of the spheres
    bool alter=false;
    for (size_t i = 0; i < npoints; i += 1)
    {
        // get respective points from each cloud
        const pcl::PointXYZRGB & p_src = source->points.at(i);
        pcl::PointXYZRGB & p_tgt = target->points.at(i);

        // Generate a unique string for each line
        std::stringstream ss ("line");
        ss << i;
        std::stringstream sss ("spheresource");
        sss << i;
        std::stringstream ssss ("spheretarget");
        ssss << i;

        // draw the line objects onto the point clouds
        if(alter)
        {
            //this is for red lines and spheres
            viscorr.addSphere<pcl::PointXYZRGB>(p_src,0.25,255,0,0,sss.str());
            viscorr.addSphere<pcl::PointXYZRGB>(p_tgt,0.25,255,0,0,ssss.str());
            viscorr.addLine<pcl::PointXYZRGB> (p_src, p_tgt, 50, 0, 155, ss.str ());      
        }
        else
        {
            //this is for yellow ones
            viscorr.addSphere<pcl::PointXYZRGB>(p_src,0.25,255,255,0,sss.str());
            viscorr.addSphere<pcl::PointXYZRGB>(p_tgt,0.25,255,255,0,ssss.str());
            viscorr.addLine<pcl::PointXYZRGB> (p_src, p_tgt, 0, 50, 155, ss.str ());
        }

        // switch color each point for better visibility
        alter = !alter;
    }

    viscorr.resetCamera ();  // sets to default view
    // viscorr.spin (); // runs the window

    std::ostringstream name;
    name << "final_corrs" << beta << " " << iter << ".png";
    std::string str_name(name.str());
    viscorr.saveScreenshot(str_name);

}