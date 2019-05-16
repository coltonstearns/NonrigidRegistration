#include <iostream>
#include <vector>
#include <eigen3/Eigen/Dense>
#include <math.h>
#include <stdio.h>
#include <unordered_set>

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
#include "utils/fpfh.h"
#include "utils/laplacian.h"
#include "als.h"


struct PclData {
    pcl::PointCloud<pcl::PointXYZ>::Ptr source;
    pcl::PointCloud<pcl::PointXYZ>::Ptr target;
    pcl::PointCloud<pcl::PointXYZ>::Ptr putative_source;
    pcl::PointCloud<pcl::PointXYZ>::Ptr putative_target;
    pcl::Correspondences correspondences;
};


class NonrigidAlign {
    private:
        // acual data
        PclData pcl_data;

        // point set values
        int nsamples;
        int npoints;
        int ncorrs;
        int dims = 3;
        float volume = (200. * 200. * 100.); // adjust to double if not accurate enough
        float tosca_fpfh_distance = 8.9;

        // hyperparameters
        float epsilon;
        float beta;
        float lambda1;
        float lambda2;
        float sparsity_threshold;
        int round = 0;

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


    public:

        // computed properties of the data
        Eigen::MatrixXf transformed_X;
        Eigen::SparseMatrix<float> laplacian;
        EigenData eigen_data;

        /**
         * Constructor. source and target are self explanatory.
         * nsamples represents how many points to be randomly sampled in computing the alignment (-1 means use all points)
         */
        NonrigidAlign(pcl::PointCloud<pcl::PointXYZ>::Ptr source, pcl::PointCloud<pcl::PointXYZ>::Ptr target,
            int npoints, int nsamples, float beta = .15, float epsilon = .15, float lambda1 = .4, float lambda2 = .4,
            float sparsity_threshold = .0000001);

        /**
         * solve one iteration of the non-rigid alignment
         */
        void alignOneiter(int iteration);


        /**
         * Displays 3D view of correspondences
         */
        void displayCorrespondences();

};

