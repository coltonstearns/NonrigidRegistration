#include "rkhs.h"

/**
 * How Matrix Kernel Works in this application:
 * Takes in all source points and computes
 * the Gaussian Matrix-kernel (takes in 2 vectors and outputs a matrix, such that
 * when that matrix is multiplied by the "c" vector, it returns the target VECTOR). 
 * 
 */



/**
 * Compute the Gram Matrix of all the SOURCE points.
 * This is called \Tau in the paper
 * gramKernel needs to be of size (points x points)
 */
void computeGramKernel(Eigen::MatrixXf source, Eigen::MatrixXf gramKernel, float beta){
    // check for correctly formatted input
    const int ndata = source.rows();
    const int gramwidth = gramKernel.rows();
    const int gramheight = gramKernel.cols();
    if ((ndata != gramwidth) && (ndata != gramheight)) {
        throw std::runtime_error("Data dimension does not match Gram Matrix Dimensions");
    }

    float dist;
    float kernel_val;
    for (int i = 0; i < ndata; i++) {
        for (int j = 0; j <= i; j++){
            dist = pow((source(i,0) - source(j,0)), 2) + pow((source(i,1) - source(j,1)), 2) + pow((source(i,2) - source(j,2)), 2);
            kernel_val = exp(-beta * dist);
            gramKernel(i,j) = kernel_val;
            gramKernel(j,i) = kernel_val;
        }
    }

}