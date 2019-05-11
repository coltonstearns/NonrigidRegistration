/**
 * Implementation of Alternating Least Squares class, which performs ALS
 * to compute the optimal transformation T* for a round
 */

#include "als.h"

Eigen::MatrixXf AlternatingLeastSquares::update_P(Eigen::MatrixXf X_new, float gamma, float variance){
    Eigen::MatrixXf P (ncorrs, ncorrs);
    Eigen::MatrixXf Z = Y - X_new;
    for (int i = 1; i < ncorrs; i++){
        float squared_norm = Z.row(i).squaredNorm();
        float part1 = gamma * std::exp(- squared_norm / (2 * variance));
        float part2 = (1 - gamma) * pow(2 * M_PI * variance, dims/2) / a;
        float p_i = part1 / (part1 + part2);
        P(i,i) = p_i;
    }
    return P;
};

float AlternatingLeastSquares::update_variance(Eigen::MatrixXf P, Eigen::MatrixXf X_new){
    return ((Y-X).transpose() * P * (Y-X_new)).trace() / (dims * P.trace());
}


void AlternatingLeastSquares::update_C(Eigen::MatrixXf J, Eigen::MatrixXf P, float variance) {
    // compute the left side A in the Ax=b problem
    // std::printf("Rows: %ld, Cols: %ld\n", J.rows(), J.cols());
    // std::printf("Rows: %ld, Cols: %ld\n", P.rows(), P.cols());
    // std::printf("Rows: %ld, Cols: %ld\n", Tau.rows(), Tau.cols());

    Eigen::MatrixXf first_part = J.transpose()*P*J*Tau; // EXTREMELY SLOW BECAUSE (dense n by l)(dense l by l)(dense l by n)(dense n by n) --> O(n^3)!
    // cout << first_part(1,0) << " First" << std::endl;
    std::cout << "======================" << std::endl;
    std::cout << Tau(16,0) << std::endl;
    std::cout << Tau(17,0) << std::endl;
    std::cout << Tau(18,0) << std::endl;
    std::cout << Tau(19,0) << std::endl;
    std::cout << Tau(20,0) << std::endl;
    std::cout << Tau(21,0) << std::endl;    // std::printf("2.61\n");
    Eigen::MatrixXf second_part = lambda1 * variance * Eigen::MatrixXf::Identity(npoints, npoints);
    // cout << second_part(1,0) << " First" << std::endl;

    // std::printf("2.62\n");    
    // std::printf("Rows: %ld, Cols: %ld\n", laplacian_approx.rows(), laplacian_approx.cols());
    Eigen::MatrixXf third_part = lambda2 * variance * laplacian_approx * Tau;
    // cout << third_part(1,0) << " First" << std::endl;
    // std::printf("2.63\n");
    Eigen::MatrixXf A = first_part + second_part + third_part; // n x n
    // std::printf("2.64\n");

    // compute the right side b in the Ax=b problem
    Eigen::MatrixXf b = J.transpose() * P * Y;  // (dense l by n) (dense l by l) (dense l by 3)

    // cout << A(1,0) << std::endl;
    // cout << A(2,0) << std::endl;
    // cout << A(3,0) << std::endl;
    // cout << A(5,0) << std::endl;

    // std::printf("2.65\n");
    // update C 
    this->C = A.colPivHouseholderQr().solve(b);
}



void AlternatingLeastSquares::optimize(int niters){
    // define initial values for gamma and variance
    // std::printf("2.1\n");
    Eigen::MatrixXf P = Eigen::MatrixXf::Identity(ncorrs, ncorrs);
    float gamma = 1; // trace(P) / ncorrs = 1
    float variance = ((Y-X).transpose() * P * (Y-X)).trace() / (dims * P.trace());

    // std::printf("2.2\n");
    // define initial value for J based on paper
    Eigen::MatrixXf J (ncorrs, npoints);
    J << Eigen::MatrixXf::Identity(ncorrs, ncorrs), Eigen::MatrixXf::Zero(ncorrs, npoints-ncorrs);

    // define initial value for P based on paper


    // std::printf("2.3\n");
    Eigen::MatrixXf X_new = X; // initialize T(X) as X to start
    for (int i = 0; i < niters; i++){
        printf("ALS: Iter %d out of %d\n", i+1, niters);
        // step 1: update P
        // std::printf("2.4\n");
        P = update_P(X_new, gamma, variance);


        // step 2: update gamma and variance
        // std::printf("2.5\n");
        variance = update_variance(P, X_new);
        // std::printf("2.55\n");
        gamma = P.trace() / ncorrs;

        // step 3: update C
        // std::printf("2.6\n");
        update_C(J, P, variance);
        // std::printf("2.7\n");
    }
};

