/**
 * Implementation of Alternating Least Squares class, which performs ALS
 * to compute the optimal transformation T* for a round
 */

#include "als.h"


// =========================== Private Methods ============================

Eigen::MatrixXf AlternatingLeastSquares::update_P(float gamma, float variance){
    Eigen::MatrixXf P (ncorrs, ncorrs);
    Eigen::MatrixXf Z = eigen_data.putative_target - eigen_data.putative_source;
    for (int i = 1; i < ncorrs; i++){
        float squared_norm = Z.row(i).squaredNorm();
        float part1 = gamma * std::exp(- squared_norm / (2 * variance));
        float part2 = (1 - gamma) * pow(2 * M_PI * variance, dims/2) / a;
        float p_i = part1 / (part1 + part2);
        P(i,i) = p_i;
    }
    return P;
};

float AlternatingLeastSquares::update_variance(Eigen::MatrixXf P){
    return ((eigen_data.putative_target-eigen_data.putative_source).transpose() * P *
     (eigen_data.putative_target-eigen_data.putative_source)).trace() / (dims * P.trace());
}


void AlternatingLeastSquares::update_C(Eigen::MatrixXf U, Eigen::MatrixXf V, Eigen::MatrixXf P, float variance) {
    // compute the left side A in the Ax=b problem
    // std::printf("Rows: %ld, Cols: %ld\n", J.rows(), J.cols());
    // std::printf("Rows: %ld, Cols: %ld\n", P.rows(), P.cols());
    // std::printf("Rows: %ld, Cols: %ld\n", Tau.rows(), Tau.cols());

    Eigen::MatrixXf first_part = U.transpose()*P*U; 
    // cout << first_part(1,0) << " First" << std::endl;
    std::cout << "======================" << std::endl;
    std::cout << Tau(16,0) << std::endl;
    std::cout << Tau(17,0) << std::endl;
    std::cout << Tau(18,0) << std::endl;
    std::cout << Tau(19,0) << std::endl;
    std::cout << Tau(20,0) << std::endl;
    std::cout << Tau(21,0) << std::endl;    // std::printf("2.61\n");
    Eigen::MatrixXf second_part = lambda1 * variance * Tau;
    // cout << second_part(1,0) << " First" << std::endl;

    // std::printf("2.62\n");    
    // std::printf("Rows: %ld, Cols: %ld\n", laplacian_approx.rows(), laplacian_approx.cols());
    Eigen::MatrixXf third_part = lambda2 * variance * V.transpose() * full_laplacian * V; //Todo: approximate!
    // cout << third_part(1,0) << " First" << std::endl;
    // std::printf("2.63\n");
    Eigen::MatrixXf A = first_part + second_part + third_part; // n x n
    // std::printf("2.64\n");

    // compute the right side b in the Ax=b problem
    Eigen::MatrixXf b = U.transpose() * P * eigen_data.putative_target;  // (dense l by n) (dense l by l) (dense l by 3)

    // cout << A(1,0) << std::endl;
    // cout << A(2,0) << std::endl;
    // cout << A(3,0) << std::endl;
    // cout << A(5,0) << std::endl;

    // std::printf("2.65\n");
    // update C 
    this->C = A.colPivHouseholderQr().solve(b);
}

/**
 * Computes nsamples completely random points in X_downsampled from
 * the source.
 */
void AlternatingLeastSquares::compute_X_downsampled() {
    if (nsamples == -1) {
        nsamples = npoints;
    }

    // this algorithm may be weird, but note that it generates
    // nsample unique values!
    int rand_indices[nsamples];
    int used_indices[nsamples];
    for (int i = 0; i< npoints; i++){
        used_indices[i] = i;
    }
    srand(time(0));  // Initialize random number generator.
    for (int i = 0; i < nsamples; i++){
        int r = (rand() % (npoints - i)); //npoints-1 represents max index val
        rand_indices[i] = used_indices[r];
        used_indices[r] = (npoints-1);
    }

    // now update our X to be the random number
    X_downsampled(nsamples, dims); // construct X_downsampled
    for (int i = 0; i < nsamples; i++){
        X_downsampled.row(i) = eigen_data.source.row(rand_indices[i]);
        std::cout << X_downsampled.row(i) << std::endl;
        std::cout << eigen_data.source.row(rand_indices[i]) << std::endl;
    }
}

/**
 * Computes Tau based on downsampled X
 */
void AlternatingLeastSquares::compute_Tau() {
    Eigen::MatrixXf t = computeGramKernel(X_downsampled, 1); //set beta = .1
    Tau = t;
}

void AlternatingLeastSquares::approximate_laplacian() {
    // Construct matrix operation object using the wrapper class SparseGenMatProd
    Spectra::SparseGenMatProd<float> op(full_laplacian);

    // Construct eigen solver object, requesting the largest three eigenvalues
    // the 6 indicates how fast we want to compute them
    Spectra::GenEigsSolver<float, Spectra::LARGEST_MAGN, Spectra::SparseGenMatProd<float> > eigs(&op, nsamples, 6);

    // Initialize and compute
    eigs.init();
    int nconv = eigs.compute();

    // Retrieve results
    Eigen::VectorXcd evalues;
    if(eigs.info() == Spectra::SUCCESSFUL)
        evalues = eigs.eigenvalues();
}

Eigen::MatrixXf AlternatingLeastSquares::compute_U() {
    Eigen::MatrixXf U (ncorrs, nsamples); // U is a (L by k) matrix
    // U_{ij} = k(X_{i}, Samples_{j})
    U = computeGeneralKernel(eigen_data.putative_source, X_downsampled, 1); //beta = 1
}

Eigen::MatrixXf AlternatingLeastSquares::compute_V() {
    Eigen::MatrixXf V (npoints, nsamples);
    V = computeGeneralKernel(eigen_data.source, X_downsampled, 1);
}

// ============================ Public Methods ============================

AlternatingLeastSquares::AlternatingLeastSquares(EigenData eigen_data, Eigen::SparseMatrix<float> full_laplacian, float lambda1, float lambda2,
     float a, int npoints, int ncorrs, int nsamples): eigen_data(eigen_data), full_laplacian(full_laplacian), lambda1(lambda1),
     lambda2(lambda2), a(a), npoints(npoints), ncorrs(ncorrs), nsamples(nsamples) {
         // get the appropriate downsample for X
         compute_X_downsampled();

         // compute Tau based on downsampled X
         compute_Tau();

         // compute our laplacian approximation
         approximate_laplacian();
     }


void AlternatingLeastSquares::optimize(int niters){
    // define initial values for gamma and variance
    // std::printf("2.1\n");
    Eigen::MatrixXf P = Eigen::MatrixXf::Identity(ncorrs, ncorrs);
    float gamma = 1; // trace(P) / ncorrs = 1
    float variance = update_variance(P);

    // std::printf("2.2\n");
    // define initial value for J based on paper
    Eigen::MatrixXf U = compute_U();
    Eigen::MatrixXf V = compute_V();

    // std::printf("2.3\n");
    for (int i = 0; i < niters; i++){
        printf("ALS: Iter %d out of %d\n", i+1, niters);
        // step 1: update P
        // std::printf("2.4\n");
        P = update_P(gamma, variance);


        // step 2: update gamma and variance
        // std::printf("2.5\n");
        variance = update_variance(P);
        // std::printf("2.55\n");
        gamma = P.trace() / ncorrs;

        // step 3: update C
        // std::printf("2.6\n");
        update_C(U, V, P, variance);
        // std::printf("2.7\n");
    }
};





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



Eigen::MatrixXf computeGeneralKernel(Eigen::MatrixXf pointset1, Eigen::MatrixXf pointset2, float beta){
    // check for correctly formatted input
    const int ndata = pointset1.rows();
    const int nsample = pointset2.rows();
    Eigen::MatrixXf kernel (ndata, nsample);

    float dist;
    float kernel_val;
    for (int i = 0; i < ndata; i++) {
        for (int j = 0; j <= nsample; j++){
            dist = (pointset1.row(i) - pointset2.row(j)).squaredNorm();
            kernel_val = exp(-beta * dist);
            kernel(i,j) = kernel_val;
        }
    }
    
    return kernel;


}