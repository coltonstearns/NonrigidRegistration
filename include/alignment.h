#include <iostream>
#include <vector>
#include <eigen3/Eigen/Dense>
#include <math.h>

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
#include "parseTosca.h"
#include "fpfh.h"
#include "register.h"

// boost
#include <boost/smart_ptr/shared_ptr.hpp>


class NonrigidAlign {
    private:
        catData data;
        pcl::Correspondences correspondences;
        Eigen::SparseMatrix<float> laplacian;
        pcl::PointCloud<pcl::PointXYZ>::Ptr putative_source;
        pcl::PointCloud<pcl::PointXYZ>::Ptr putative_target;


    public:
        /**
         * Constructor for default cat data
         */
        NonrigidAlign();

        /**
         * Computes FPFH Correspondences that are valid between the source and target
         * of the defined data
         */
        void getCorrespondences();

        /**
         * Creates two points clouds that only contain the corresponding points from the
         * FPFH matching
         */
        void getPutativeCorrespondenceSets();

        /**
         * Displays 3D view of correspondences
         */
        void displayCorrespondences();

        /**
         * computes the Graph Laplacian of the ALL source points
         * This is the matrix A from the paper
         */
        void computeLaplacian();


};