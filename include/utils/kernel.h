#include <iostream>
#include <vector>
#include <math.h>
#include <ctime>    // For time()
#include <cstdlib>  // For srand() and rand()


// Eigen Dependencies
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Sparse>
#include <eigen3/Eigen/Eigenvalues>



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


void transform_source(Eigen::MatrixXf &C, Eigen::MatrixXf &downsampled_X, Eigen::MatrixXf &result, Eigen::MatrixXf &source, float beta);
