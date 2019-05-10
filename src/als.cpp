/**
 * Implementation of Alternating Least Squares class, which performs ALS
 * to compute the optimal transformation T* for a round
 */

#include "als.h"

void AlternatingLeastSquares::update_P(Eigen::MatrixXf P, Eigen::MatrixXf X_new, float gamma, float variance){
    Eigen::MatrixXf Z = Y - X_new;
    for (int i = 1; i < ncorrs; i++){
        float squared_norm = Z.row(i).squaredNorm();
        float part1 = gamma * std::exp(- squared_norm / (2 * variance));
        float part2 = (1 - gamma) * pow(2 * M_PI * variance, dims/2) / a;
        float p_i = part1 / (part1 + part2);
        P(i,i) = p_i;
    }
};

float AlternatingLeastSquares::update_variance(Eigen::MatrixXf P, Eigen::MatrixXf X_new){
    ((Y-X).transpose() * P * (Y-X_new)).trace() / (dims * P.trace());
}


void AlternatingLeastSquares::update_C(Eigen::MatrixXf J, Eigen::MatrixXf P, float variance) {
    // compute the left side A in the Ax=b problem
    Eigen::MatrixXf first_part = J.transpose()*P*J*Tau; // EXTREMELY SLOW BECAUSE (dense n by l)(dense l by l)(dense l by n)(dense n by n) --> O(n^3)!
    Eigen::MatrixXf second_part = lambda1*variance*Eigen::MatrixXf::Identity(npoints, npoints);
    Eigen::MatrixXf third_part = lambda2 * variance * laplacian_approx * Tau;
    Eigen::MatrixXf A = first_part + second_part + third_part; // n x n

    // compute the right side b in the Ax=b problem
    Eigen::MatrixXf b = J.transpose() * P * Y;  // (dense l by n) (dense l by l) (dense l by 3)

    // update C 
    C = A.colPivHouseholderQr().solve(b);
}



void AlternatingLeastSquares::optimize(int niters){
    // define initial values for gamma and variance
    Eigen::MatrixXf P = Eigen::MatrixXf::Identity(ncorrs, ncorrs);
    float gamma = 1; // trace(P) / ncorrs = 1
    float variance = ((Y-X).transpose() * P * (Y-X)).trace() / (dims * P.trace());

    // define initial value for J based on paper
    Eigen::MatrixXf J (ncorrs, npoints);
    J << Eigen::MatrixXf::Identity(ncorrs, ncorrs), Eigen::MatrixXf::Zero(ncorrs, npoints-ncorrs);

    // define initial value for P based on paper


    Eigen::MatrixXf X_new = X; // initialize T(X) as X to start
    for (int i = 0; i < niters; i++){
        // step 1: update P
        update_P(P, X_new, gamma, variance);

        // step 2: update gamma and variance
        variance = update_variance(P, X_new);
        gamma = P.trace() / ncorrs;

        // step 3: update C
        update_C(J, P, variance);
    }
};

// void AlternatingLeastSquares::computeOptimalTransformation(Eigen::MatrixXf X_transformed){
//     // compute transformation T(X)
//     Eigen::MatrixXf t_of_X_n = Eigen::MatrixXf::Zero(npoints, 3);
//     for (int i = 0; i < npoints; i++){
//         Eigen::Block<Eigen::MatrixXf, 1, -1, false> precomputed_kernel_vals = Tau.row(i);
//         for (int j = 0; j < npoints; j++) {
//             // Kernel(x_i, x_{all}) * C_{all} to get the transformed point T(x_i)
//             if (precomputed_kernel_vals(j) > 0.000001){
//                 t_of_X_n.row(i) += C.row(i) * (precomputed_kernel_vals(j) * Eigen::MatrixXf::Identity(3,3)); // results in 1x3
//             }
//         }
//     }
// };
