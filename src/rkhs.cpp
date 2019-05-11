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
Eigen::MatrixXf computeGramKernel(Eigen::MatrixXf source, float beta){
    // check for correctly formatted input
    const int ndata = source.rows();
    Eigen::MatrixXf gramKernel (ndata, ndata);
    const int gramwidth = gramKernel.rows();
    const int gramheight = gramKernel.cols();
    if ((ndata != gramwidth) && (ndata != gramheight)) {
        throw std::runtime_error("Data dimension does not match Gram Matrix Dimensions");
    }

    float dist;
    float kernel_val;
    for (int i = 0; i < ndata; i++) {
        for (int j = 0; j <= i; j++){
            dist = (source.row(i) - source.row(j)).squaredNorm();
            kernel_val = exp(-beta * dist);
            gramKernel(i,j) = kernel_val;
            gramKernel(j,i) = kernel_val;
        }
    }
    
    std::cout << gramKernel(16,0) << std::endl;
    std::cout << gramKernel(17,0) << std::endl;
    std::cout << gramKernel(18,0) << std::endl;
    std::cout << gramKernel(19,0) << std::endl;
    std::cout << gramKernel(20,0) << std::endl;
    std::cout << gramKernel(21,0) << std::endl;
    return gramKernel;


}