#include "utils/kernel.h"


/**
 * Compute the Gram Matrix of all the SOURCE points.
 * This is called \Tau in the paper
 * gramKernel needs to be of size (points x points)
 * DEBUGGED MOSTLY
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
    
    return gramKernel;


}


/**
 * DEBUGGED. This function is good.
 */
Eigen::MatrixXf computeGeneralKernel(Eigen::MatrixXf pointset1, Eigen::MatrixXf pointset2, float beta){
    // check for correctly formatted input
    const int ndata = pointset1.rows();
    const int samples = pointset2.rows();
    Eigen::MatrixXf kernel (ndata, samples);

    float dist;
    float kernel_val;
    for (int i = 0; i < ndata; i++) {
        for (int j = 0; j < samples; j++){
            dist = (pointset1.row(i) - pointset2.row(j)).squaredNorm();
            kernel_val = exp(-beta * dist);
            kernel(i,j) = kernel_val;
        }
    }
    
    return kernel;

}


/**
 * C: nsamples by 3; represents the c components
 * downsampled_X: randomly sampled points (nsamples by 3)
 * result: empty npoints by 3 matrix which points will be put into; MUST CONTAIN ALL ZEROS
 * source: original npoints by 3 source
 * beta: regularization hyperparameter
 */
void transform_source(Eigen::MatrixXf &C, Eigen::MatrixXf &downsampled_X, Eigen::MatrixXf &result, Eigen::MatrixXf &source, float beta) {
    int npoints = result.rows();
    int nsamples = downsampled_X.rows();
    std::cout << "transforming!" << std::endl;

    // compute the kernel with our randomly sampled points
    Eigen::MatrixXf kernel_vals (npoints, nsamples);
    kernel_vals = computeGeneralKernel(source, downsampled_X, beta);

    // compute each T(X) based on RKHS kernel
    for (int i = 0; i < npoints; i++){
        Eigen::Block<Eigen::MatrixXf, 1, -1, false> precomputed_kernel_vals = kernel_vals.row(i);
        for (int j = 0; j < nsamples; j++) {
            // Kernel(x_i, x_{all}) * C_{all} to get the transformed point T(x_i)
            result.row(i) += C.row(j) * (precomputed_kernel_vals(j) * Eigen::MatrixXf::Identity(3,3)); // FOUND A BUG HERE!
        }
    }    

    std::cout << "====================== Computed X =====================" << std::endl << std::endl << 
    result.row(0) << result.row(1) << std::endl;
    std::cout << "tada!" << std::endl;
}