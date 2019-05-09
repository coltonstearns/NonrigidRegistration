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
#include "rkhs.h"

// boost
#include <boost/smart_ptr/shared_ptr.hpp>

struct MatrixData {
    Eigen::MatrixXf source;
    Eigen::MatrixXf target;
    Eigen::MatrixXf putative_source;
    Eigen::MatrixXf putative_target;
    Eigen::MatrixXf correspondences;
};



class NonrigidAlign {
    private:
        catData data;
        pcl::Correspondences correspondences;
        Eigen::SparseMatrix<float> laplacian;
        MatrixData matrix_data; 
        // pcl::PointCloud<pcl::PointXYZ>::Ptr putative_source;
        // pcl::PointCloud<pcl::PointXYZ>::Ptr putative_target;


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
         * Once correspondences are calculated, transfers all matrices to eigen format instead of
         * pcl point cloud.
         */
        void generateEigenMatrices();

        /**
         * Displays 3D view of correspondences
         */
        void displayCorrespondences();

        /**
         * computes the Graph Laplacian of the ALL source points
         * This is the matrix A from the paper
         */
        void computeLaplacian();

        /**
         * solve one iteration of the non-rigid alignment
         */
        void alignOneiter(float lambda1, float lambda2);


};


void get_correspondence_matrix(Eigen::MatrixXf X, Eigen::MatrixXf correspondences, Eigen::MatrixXf putative_X);
