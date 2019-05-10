/**
 * This file contains a class that performs the inner loop of alternating least 
 * squares optimization described in the paper
 */

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

class AlternatingLeastSquares {
    private:
    Eigen::MatrixXf Tau;  // gram kernel matrix of all points we are using
    Eigen::MatrixXf laplacian_approx; // k largest eigenval approximation of laplacian
    Eigen::MatrixXf X;
    Eigen::MatrixXf Y;
    double a;
    float lambda1;
    float lambda2;
    int npoints;
    int ncorrs;
    float dims = 3.0;

    /**
     * Updates P matrix based on equation 12
     */
    void update_P(Eigen::MatrixXf P, Eigen::MatrixXf X_new, float gamma, float variance);

    /**
     * Updates gamma and variance based on equations 13 and 14
     */
    float update_variance(Eigen::MatrixXf P, Eigen::MatrixXf X_new);

    /**
     * Computes C using linear system solver following equation 19
     */
    void update_C(Eigen::MatrixXf J, Eigen::MatrixXf P, float variance);


    public:

    // describes our optimal transformation
    Eigen::MatrixXf C; 

    /**
     * Constructor for ALS optimization. 
     * Tau and the laplacian are based on the entire set of points, and are passsed in
     * X and Y are the set of putative correspondences, and are used in the optimization
     * volume is $a$ in the paper and represents the 1/volume uniform distribution
     * lambda1 is the regularization weight for the complexity of the function T(x)
     * lambda2 is the regularization weight for deviating from the laplacian shape description 
     */
    AlternatingLeastSquares(Eigen::MatrixXf Tau, Eigen::MatrixXf laplacian_approx,
     Eigen::MatrixXf X, Eigen::MatrixXf Y, float lambda1, float lambda2, double a, int npoints, int ncorrs):
    Tau(Tau), laplacian_approx(laplacian_approx), X(X), Y(Y), lambda1(lambda1), lambda2(lambda2), a(a),
     npoints(npoints), ncorrs(ncorrs), C(Eigen::MatrixXf(npoints, 3)) {};

    /**
     * Runs the optimization algorithm to compute the best C matrix over a specified
     * number of iterations. Updates inner rep to reflect the new C matrix.
     */
    void optimize(int niters);

    /**
     * Computes T(X) given the matrix C stored in its inner rep.
     */
    // void computeOptimalTransformation(Eigen::MatrixXf X_transformed);
};