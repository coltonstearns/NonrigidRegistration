// class header file
#include "alignment.h"


// ============================= Public Methods =================================

/**
 * Constructor for default cat data
 */
NonrigidAlign::NonrigidAlign(){
    data = generateCatPointCloud();
    // pcl::PointCloud<pcl::PointXYZ>::Ptr putative_source_ptr (new pcl::PointCloud<pcl::PointXYZ>);
    // pcl::PointCloud<pcl::PointXYZ>::Ptr putative_target_ptr (new pcl::PointCloud<pcl::PointXYZ>);
    // putative_source = putative_source_ptr;
    // putative_target = putative_target_ptr;
};

void NonrigidAlign::getCorrespondences(){
    correspondences = calculateCorrespondences(data.source, data.target, 8.9); // 8.9 is optimal value
}

/**
 * Translates all point cloud data into eigen data for easy use. This MUST
 * be called AFTER correspondences have been generated
 */
void NonrigidAlign::generateEigenMatrices(){
    // translate all from point cloud to eigen matrices
    int npoints = data.source->width;
    matrix_data.source = Eigen::MatrixXf(npoints, 3);
    matrix_data.target = Eigen::MatrixXf(npoints, 3);
    for (int i = 0; i < npoints; i++){
        matrix_data.source(i,0) = data.source->at(i).x;
        matrix_data.source(i,1) = data.source->at(i).y;
        matrix_data.source(i,2) = data.source->at(i).z;
        matrix_data.target(i,0) = data.target->at(i).x;
        matrix_data.target(i,1) = data.target->at(i).y;
        matrix_data.target(i,2) = data.target->at(i).z;
    }

    // translate putative set to eigen matrices
    int corr_points = correspondences.size();
    matrix_data.correspondences = Eigen::MatrixXf(corr_points, 2);
    matrix_data.putative_source = Eigen::MatrixXf(corr_points, 3);
    matrix_data.putative_target = Eigen::MatrixXf(corr_points, 3);
    for (int i = 0; i < correspondences.size(); i++){
        pcl::Correspondence corr = correspondences.at(i);
        matrix_data.correspondences(i,0) = corr.index_query;
        matrix_data.correspondences(i,1) = corr.index_match;
        matrix_data.putative_source(i,0) = data.source->at(corr.index_query).x;
        matrix_data.putative_source(i,1) = data.source->at(corr.index_query).y;
        matrix_data.putative_source(i,2) = data.source->at(corr.index_query).z;
        matrix_data.putative_target(i,0) = data.source->at(corr.index_match).x;
        matrix_data.putative_target(i,1) = data.source->at(corr.index_match).y;
        matrix_data.putative_target(i,2) = data.source->at(corr.index_match).z;
    }
}

/**
 * Displays 3D view of correspondencess
 */
void NonrigidAlign::displayCorrespondences(){
    visualize_correspondences(data.source, data.target, correspondences);
}

/**
 * computes the Graph Laplacian of the ALL source points
 * This is the matrix A from the paper
 */
void NonrigidAlign::computeLaplacian() {
    int npoints = matrix_data.source.rows();
    laplacian = Eigen::SparseMatrix<float>(npoints, npoints);
    getLaplacian(data.source, laplacian);
}

/**
 * solve one iteration of the non-rigid alignment
 */
void NonrigidAlign::alignOneiter(float lambda1, float lambda2) {
    // ====================================== Initialize Vars ==================================================
    std::printf("Here 1\n");
    int npoints = matrix_data.source.rows();
    Eigen::MatrixXf X_n = matrix_data.source; // copy X matrices
    Eigen::MatrixXf Y_n(matrix_data.target);
    Eigen::MatrixXf X_l = matrix_data.putative_source;
    Eigen::MatrixXf Y_l(matrix_data.putative_target);

    std::printf("Here 2\n");
    // generate Tau
    Eigen::MatrixXf Tau(npoints, npoints);
    computeGramKernel(X_n, Tau, 0.5); //set beta = .5

    // generate J
    int num_correspondences = correspondences.size();
    Eigen::MatrixXf J (num_correspondences, npoints);
    // std::printf("Here 4\n");
    J << Eigen::MatrixXf::Identity(num_correspondences, num_correspondences), Eigen::MatrixXf::Zero(num_correspondences, npoints-num_correspondences);
    // std::printf("Here 5\n");

    // initialize C to make it so that T(X) = X
    Eigen::MatrixXf C (npoints, 3); // 3d set of all vectors that correspond to point-kernels!

    std::printf("Here 3\n");
    // initialize probility parameters based on putative correspondences
    float dims = 3.0;
    Eigen::MatrixXf P = Eigen::MatrixXf::Identity(num_correspondences, num_correspondences);
    float gamma = P.trace() / num_correspondences;
    float var_denominator = dims * P.trace();
    float variance = ((Y_l - X_l).transpose() * P * (Y_l-X_l)).trace() / var_denominator;

    // define a, which is the bound on all volume
    float a = 200. * 200. * 200.; // space ranges from -100 to 100 for all dimensions
    // ========================================================================================================

    // ========================================================================================================
    // ====================== Do First Iteration By Hand to account for T(X) = X ==============================
    // ========================================================================================================

    std::printf("Here 6\n");
    // compute first values of P
    Eigen::MatrixXf Z = matrix_data.putative_target - matrix_data.putative_source;
    for (int i = 1; i < num_correspondences; i++){
        float part1 = gamma * std::exp((- Z.row(i).squaredNorm() / (2 * variance)));
        float part2 = (1 - gamma) * pow(2 * M_PI * variance, dims/2) / a;
        float p_i = part1 / (part1 + part2);
        P(i,i) = p_i;
    }

    std::printf("Here 7\n");

    // compute first variance and gamma
    gamma = P.trace() / num_correspondences;
    var_denominator = dims * P.trace();
    variance = ((Y_l - X_l).transpose() * P * (Y_l - X_l)).trace() / var_denominator;   

    std::printf("Here 8\n");

    std::printf("Here 8.4\n");
    // compute first C values
    // TODO Eigen can't do multiple matmuls mixed or matmuls mixes with scalar muls in the same line!
    // Have to break it up into multiple commands!!
    Eigen::MatrixXf first_part = J.transpose()*P*J*Tau; // EXTREMELY SLOW BECAUSE (dense n by l)(dense l by l)(dense l by n)(dense n by n) --> O(n^3)!
    cout << "Cols " << first_part.cols() << " Rows " << first_part.rows() << std::endl;
    std::printf("Here 8.1\n");
    Eigen::MatrixXf second_part = lambda1*variance*Eigen::MatrixXf::Identity(npoints, npoints);
    cout << "Cols " << second_part.cols() << " Rows " << second_part.rows() << std::endl;
    std::printf("Here 8.2\n");
    Eigen::MatrixXf third_part = lambda2*variance*laplacian*Tau;
    cout << "Cols " << third_part.cols() << " Rows " << third_part.rows() << std::endl;
    std::printf("Here 8.3\n");
    Eigen::MatrixXf left = first_part + second_part + third_part; // n x n


    Eigen::MatrixXf right = J.transpose() * P * Y_l;  // (dense l by n) (dense l by l) (dense l by 3)
    cout << "Rows " << J.rows() << " Cols " << J.cols() << std::endl;
    cout << "Rows " << P.rows() << " Cols " << P.cols() << std::endl;
    cout << "Rows " << Y_l.rows() << " Cols " << Y_l.cols() << std::endl;

    std::printf("Here 8.5\n");
    C = left.colPivHouseholderQr().solve(right);

    std::printf("Here 9\n");
    // I'm here right now; refactor this to subsample for speed
    // ========================================================================================================
    // =============================== Run through until iters converge =======================================
    // ========================================================================================================
    // perform the EM + alternating Least Squres loop until converge to solution for P, C, gamma, and variance
    for (int iter = 0; iter < 10; iter++) {
        // compute transformation T(X) --> TODO: error in here somewhere
        Eigen::MatrixXf t_of_X_n (npoints, 3);
        for (int i = 0; i < npoints; i++){
            Eigen::Block<Eigen::MatrixXf, 1, -1, false> precomputed_kernel_vals = Tau.row(i);
            for (int j = 0; j < npoints; j++) {
                // Kernel(x_i, x_{all}) * C_{all} to get the transformed point T(x_i)
                t_of_X_n.row(i) = C.row(i) * (precomputed_kernel_vals(j) * Eigen::MatrixXf::Identity(3,3)); // results in 1x3
            }
        }
        Eigen::MatrixXf t_of_X_l (num_correspondences, 3);
        get_correspondence_matrix(t_of_X_n, matrix_data.correspondences, t_of_X_l);
        std::printf("Here 10\n");


        // first update P based on equation 12 in the paper
        Z = matrix_data.putative_target - matrix_data.putative_source;
        for (int i = 1; i < num_correspondences; i++){
            float part1 = gamma * std::exp((- Z.row(i).squaredNorm() / (2 * variance)));
            float part2 = (1 - gamma) * pow(2 * M_PI * variance, dims/2) / a;
            float p_i = part1 / (part1 + part2);
            P(i,i) = p_i;
        }
        std::printf("Here 11\n");

        // compute first variance and gamma
        gamma = P.trace() / num_correspondences;
        var_denominator = dims * P.trace();
        variance = ((Y_l - X_l).transpose() * P * (Y_l - X_l)).trace() / var_denominator;   

        // compute first C values
        left = J.transpose()*P*J*Tau + lambda1*variance*Eigen::MatrixXf::Identity(npoints, npoints) + lambda2*variance*laplacian*Tau; // n x n
        right = J.transpose() * P * Y_l;
        C = left.colPivHouseholderQr().solve(right);
    }

}

void get_correspondence_matrix(Eigen::MatrixXf X, Eigen::MatrixXf correspondences, Eigen::MatrixXf putative_X){
    int corr_points = correspondences.rows();
    for (int i = 0; i < corr_points; i++){
        putative_X.row(correspondences(i,0)) = X.row(i);
        putative_X.row(correspondences(i,0)) = X.row(i);
    }
}

