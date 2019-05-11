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

// Eigen Dependencies
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Sparse>
#include <eigen3/Eigen/Eigenvalues>

// Spectra Dependency
#include <Spectra/GenEigsSolver.h>
#include <Spectra/MatOp/SparseGenMatProd.h>

// my packages
#include "parseTosca.h"
#include "fpfh.h"


struct EigenData {
    Eigen::MatrixXf source;
    Eigen::MatrixXf target;
    Eigen::MatrixXf putative_source;
    Eigen::MatrixXf putative_target;
    Eigen::MatrixXf correspondences;
};



class AlternatingLeastSquares {
    private:
    // initialized in constructor
    Eigen::SparseMatrix<float> full_laplacian; // entire laplacian of data
    EigenData eigen_data; // contains all eigen info we might need
    double a;
    float lambda1;
    float lambda2;
    int npoints;
    int ncorrs;
    int nsamples;
    float dims = 3.0;

    // defined later through functions
    Eigen::MatrixXf laplacian_eigenvecs; 
    Eigen::MatrixXf laplacian_eigenvals; 


    /**
     * Updates P matrix based on equation 12
     */
    Eigen::MatrixXf update_P(float gamma, float variance);

    /**
     * Updates gamma and variance based on equations 13 and 14
     */
    float update_variance(Eigen::MatrixXf P);

    /**
     * Computes C using linear system solver following equation 19
     */
    void update_C(Eigen::MatrixXf U, Eigen::MatrixXf V, Eigen::MatrixXf P, float variance);

    /**
     * Updates X_downsampled matrix to reflect nsamples randomly downsampled points
     */
    void compute_X_downsampled();

    /**
     * Computes Tau based on an appropriate random sampling method for $k$ samples.
     * Tau is sampled randomly from the ENTIRE set of X
     */
    void compute_Tau();

    /**
     * decomposes and approximates the laplacian
     */
    void approximate_laplacian();

    /**
     * Visualizes the laplacian on all the points
     */
    void visualize_laplaican();

    /**
     * Computes U as shown in paper
     */
    Eigen::MatrixXf compute_U();

    Eigen::MatrixXf AlternatingLeastSquares::compute_V();


    public:

    // describes our optimal transformation
    Eigen::MatrixXf C; // coefficients with each respective X
    Eigen::MatrixXf X_downsampled; // copy of randomly sampled X's
    Eigen::MatrixXf Tau;  // gram kernel matrix of all points we are using

    /**
     * Constructor for ALS optimization. 
     * Tau and the laplacian are based on the entire set of points, and are passsed in
     * X and Y are the set of putative correspondences, and are used in the optimization
     * volume is $a$ in the paper and represents the 1/volume uniform distribution
     * lambda1 is the regularization weight for the complexity of the function T(x)
     * lambda2 is the regularization weight for deviating from the laplacian shape description 
     */
    AlternatingLeastSquares(EigenData eigen_data, Eigen::SparseMatrix<float> full_laplacian, float lambda1, float lambda2,
     float a, int npoints, int ncorrs, int nsamples);

    /**
     * Runs the optimization algorithm to compute the best C matrix over a specified
     * number of iterations. Updates inner rep to reflect the new C matrix.
     */
    void optimize(int niters);

};


/**
 * Compute the Gram Matrix of all the SOURCE points.
 * This is called \Tau in the paper
 * gramKernel needs to be of size (points x points)
 */
Eigen::MatrixXf computeGramKernel(Eigen::MatrixXf source, float beta);

/**
 * Computes Kernel Matrix for any 2 data sets
 */
Eigen::MatrixXf computeGeneralKernel(Eigen::MatrixXf pointset1, Eigen::MatrixXf pointset2, float beta);