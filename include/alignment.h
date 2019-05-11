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

// Eigen Dependencies
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Sparse>

// my packages
#include "fpfh.h"
#include "register.h"
#include "rkhs.h"
#include "als.h"


struct EigenData {
    Eigen::MatrixXf source;
    Eigen::MatrixXf target;
    Eigen::MatrixXf putative_source;
    Eigen::MatrixXf putative_target;
    Eigen::MatrixXf correspondences;
};

struct PclData {
    pcl::PointCloud<pcl::PointXYZ>::Ptr source;
    pcl::PointCloud<pcl::PointXYZ>::Ptr target;
    pcl::PointCloud<pcl::PointXYZ>::Ptr putative_source;
    pcl::PointCloud<pcl::PointXYZ>::Ptr putative_target;
    pcl::Correspondences correspondences;
};



class NonrigidAlign {
    private:
        EigenData eigen_data;
        PclData pcl_data;
        Eigen::SparseMatrix<float> laplacian;
        Eigen::MatrixXf Tau; // should change this to sparse
        int num_samples;
        int npoints;
        int ncorrs;
        int dims = 3;
        float volume = 1. / (200. * 200. * 200); // adjust to double if not accurate enough

    /**
     * Computes the correspondences between the source and target using
     * FPFH. These correspondences will determine the points in the 
     * putative source and putative target.
     */
    void update_correspondences(float fpfh_distance);

    /**
     * Updates putative point sets based on correspondences
     */
    void update_putative_sets();

    /**
     * Computes and updates the graph laplacian for the current source points
     * This is the matrix A in the paper
     */
    void update_laplacian();

    /**
     * Computes and updates the Gaussian Kernel matrix for the current
     * source points
     */
    void update_Tau();

    /**
     * sets X -> T(X) for some given RKHS C matrix (gaussina kernel).
     */
    void transform_source(Eigen::MatrixXf C);


    public:

        Eigen::MatrixXf transformed_X;

        /**
         * Constructor. source and target are self explanatory.
         * num_samples represents how many points to be randomly sampled in computing the alignment (-1 means use all points)
         */
        NonrigidAlign(pcl::PointCloud<pcl::PointXYZ>::Ptr source, pcl::PointCloud<pcl::PointXYZ>::Ptr target,
            int npoints, int num_samples);

        /**
         * solve one iteration of the non-rigid alignment
         */
        void alignOneiter(float lambda1, float lambda2);


        /**
         * Displays 3D view of correspondences
         */
        void displayCorrespondences();
};