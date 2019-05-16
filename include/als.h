/**
 * This file contains a class that performs the inner loop of alternating least 
 * squares optimization described in the paper
 */

#include <iostream>
#include <vector>
#include <math.h>
#include <ctime>    // For time()
#include <cstdlib>  // For srand() and rand()


// PCL dependencies
#include <pcl/io/pcd_io.h>  // for working with point cloud data structures
#include <pcl/impl/point_types.hpp>  // for defining point objects
#include <pcl/point_types.h>
#include <pcl/registration/correspondence_estimation.h> // for calculating distance correspondences
#include <pcl/visualization/cloud_viewer.h>  // visualization tool


// Eigen Dependencies
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Sparse>
#include <eigen3/Eigen/Eigenvalues>

// my packages
#include "utils/kernel.h"


struct EigenData {
    Eigen::MatrixXf source;
    Eigen::MatrixXf target;
    Eigen::MatrixXf putative_source;
    Eigen::MatrixXf putative_target;
    Eigen::MatrixXf correspondences;
};



class AlternatingLeastSquares {
    private:
        // passed in data / attributes
        Eigen::SparseMatrix<float> laplacian; // sparse approx of laplacian of data
        EigenData eigen_data; // contains all eigen info we might need

        // passed in point set info
        double a;
        int npoints;
        int ncorrs;
        int nsamples;
        float dims = 3.0;

        // metainfo
        int iteration;

        // passed in hyperparameters
        float lambda1;
        float lambda2;
        float beta;

        // defined later data attributes
        // Eigen::MatrixXf laplacian_eigenvecs; 
        // Eigen::MatrixXf laplacian_eigenvals; 
        Eigen::MatrixXf Tau;  // gram kernel matrix of all points we are using
        Eigen::MatrixXf P;
        Eigen::MatrixXf current_X;


        /**
         * Updates P matrix based on equation 12
         */
        void update_P(float gamma, float variance);

        /**
         * Updates gamma and variance based on equations 13 and 14
         */
        float update_variance(Eigen::MatrixXf P);

        /**
         * Computes C using linear system solver following equation 19
         */
        void update_C(Eigen::MatrixXf &U, Eigen::MatrixXf &V, float variance);

        /**
         * Updates X_downsampled matrix to reflect nsamples randomly downsampled points
         */
        void compute_X_downsampled();


    public:

        // describes our optimal transformation
        Eigen::MatrixXf C; // coefficients with each respective X
        Eigen::MatrixXf X_downsampled; // copy of randomly sampled X's

        /**
         * Constructor for ALS optimization. 
         * Tau and the laplacian are based on the entire set of points, and are passsed in
         * X and Y are the set of putative correspondences, and are used in the optimization
         * volume is $a$ in the paper and represents the 1/volume uniform distribution
         * lambda1 is the regularization weight for the complexity of the function T(x)
         * lambda2 is the regularization weight for deviating from the laplacian shape description 
         */
        AlternatingLeastSquares(EigenData eigen_data, Eigen::SparseMatrix<float> laplacian, float lambda1,
         float lambda2, float beta, float a, int npoints, int ncorrs, int nsamples, int iteration);

        /**
         * Runs the optimization algorithm to compute the best C matrix over a specified
         * number of iterations. Updates inner rep to reflect the new C matrix.
         */
        void optimize(int niters);

        /**
         * Visualize our randomly downsampled points
         */
        void visualize_downsampled_points();


        void visualize_putative_points();

    
};

