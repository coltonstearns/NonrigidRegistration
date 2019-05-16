/**
 * Implementation of Alternating Least Squares class, which performs ALS
 * to compute the optimal transformation T* for a round
 */

#include "als.h"

// ============================ Public Methods ============================

AlternatingLeastSquares::AlternatingLeastSquares(EigenData eigen_data, Eigen::SparseMatrix<float> laplacian, float lambda1, float lambda2,
     float beta, float a, int npoints, int ncorrs, int nsamples, int iteration): eigen_data(eigen_data), laplacian(laplacian), lambda1(lambda1),
     lambda2(lambda2), beta(beta), a(a), npoints(npoints), ncorrs(ncorrs), nsamples(nsamples), iteration(iteration), P(Eigen::MatrixXf::Identity(ncorrs, ncorrs)) {
         // get the appropriate downsample for X
         compute_X_downsampled();

         // compute Tau based on downsampled X
         this->Tau = computeGramKernel(X_downsampled, beta);
         this->current_X = eigen_data.putative_source;
     }


void AlternatingLeastSquares::optimize(int niters){
    visualize_putative_points();

    // P is already defined
    visualize_downsampled_points(); // sanity check
    // define initial values for gamma and variance
    // std::printf("2.1\n");
    std::cout<< "yep1" << std::endl;
    float gamma = .9; // initialize it to this as a base marginal dist guess??
    float variance = update_variance(P);
    std::cout << "VARIANCE: " << variance << std::endl;

    // std::printf("2.2\n");
    // define initial value for J based on paper
    Eigen::MatrixXf U = computeGeneralKernel(eigen_data.putative_source, X_downsampled, beta); //ncorrs by nsamples
    Eigen::MatrixXf V = computeGeneralKernel(eigen_data.source, X_downsampled, beta); // npoints by nsamples
    std::cout<< "yep3" << std::endl;

    // std::printf("2.3\n");
    for (int i = 0; i < niters; i++){
        for (int j = 0; j < 1; j++){
            // std::cout << "===================== P ===============" << std::endl << std::endl << P.row(3) << std::endl;
            std::cout << "GAMMA must be < 1: " << gamma << std::endl;


            printf("ALS: Iter %d out of %d\n", i+1, niters);

            // step 1: update P
            // std::printf("2.4\n");
            update_P(gamma, variance);
            std::cout << "P: " << std::endl;
            for (int i = 0; i < 15; i++){
            std::cout << P(i,i) << " ";
            }
            std::cout << std::endl;

            // step 2: update gamma and variance
            std::printf("2.5\n");
            variance = update_variance(P);
            std::cout << "VARIANCE: " << variance << std::endl;
            std::printf("2.55\n");
            gamma = P.trace() / ncorrs;

            // step 3: update C
            std::printf("2.6\n");
            update_C(U, V, variance); 
            std::printf("2.7\n");

            // calculate Q, the total energy
            float part1 = (P.cwiseSqrt() * (eigen_data.putative_target - current_X)).squaredNorm() / variance;
            float part2 = lambda1 * (C.transpose() * Tau * C).trace();
            float part3 = lambda2 * (C.transpose() * V.transpose() * laplacian * V * C).trace();
            float error = part1 + part2 + part3;
            cout << "====================== ERROR ====================" << std::endl << std::endl;

            cout << "C size " << C.trace() << std::endl;
            cout << "Error Part 1 " << part1 << std::endl;
            cout << "Error Part 2 " << part2 << std::endl;
            cout << "Error Part 3 " << part3 << std::endl;

            cout << error << std::endl;
            cout << "=================================================" << std::endl;

        }

        // step 4: update our "current X" to allow convergence
        this->current_X = U*C;

        // calculate Q, the total energy
        float part1 = (P.cwiseSqrt() * (eigen_data.putative_target - current_X)).squaredNorm() / variance;
        float part2 = lambda1 * (C.transpose() * Tau * C).trace();
        float part3 = lambda2 * (C.transpose() * V.transpose() * laplacian * V * C).trace();
        float error = part1 + part2 + part3;
        cout << "====================== ERROR ====================" << std::endl << std::endl;
        cout << error << std::endl;
        cout << "=================================================" << std::endl;

        // this->current_X = Eigen::MatrixXf::Zero(ncorrs, dims); // reset current X
        // transform_source(C, X_downsampled, current_X, eigen_data.putative_source, beta);  // get new current X!
        // std::cout << "====================== ACTUAL UPDATED X =====================" << std::endl << std::endl << 
        // current_X.row(0) << " " << current_X.row(1) << std::endl;
        // std::cout << "====================== PUTATIVE X =====================" << std::endl << std::endl << 
        // eigen_data.putative_source.row(0) << " " << eigen_data.putative_source.row(1) << std::endl;
        
        // float Q;
        // Eigen::VectorXf sums = (eigen_data.putative_target - eigen_data.putative_source).rowwise().squaredNorm();
    }
    

};



void AlternatingLeastSquares::visualize_downsampled_points() {
    // convert both clouds into RGB
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr downsample (new pcl::PointCloud<pcl::PointXYZRGB>);
    for (int i=0; i < nsamples; i++){
        pcl::PointXYZRGB point = pcl::PointXYZRGB(0, 0, 255);
        point.x = X_downsampled(i,0); point.y = X_downsampled(i,1); point.z = X_downsampled(i,2);
        downsample->push_back(point);
    }

    // convert both clouds into RGB
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr s (new pcl::PointCloud<pcl::PointXYZRGB>);
    for (int i=0; i < npoints; i++){
        pcl::PointXYZRGB point = pcl::PointXYZRGB(255, 0, 0);
        point.x = eigen_data.source(i,0); point.y = eigen_data.source(i,1); point.z = eigen_data.source(i,2);
        s->push_back(point);
    }
    
    // initialize visualization
    pcl::visualization::PCLVisualizer viscorr;

    // add both clouds to visualizer
    viscorr.addPointCloud(s->makeShared(), "src_points");
    viscorr.addPointCloud(downsample->makeShared(), "tgt_points");

    for (size_t i = 0; i < nsamples; i += 1)
    {
        // get respective points from each cloud
        const pcl::PointXYZRGB & p_sample = downsample->points.at(i);

        // Generate a unique string for each line
        std::stringstream sss ("spheresource");
        sss << i;
        viscorr.addSphere<pcl::PointXYZRGB> (p_sample, .5, 0,0,255,sss.str ());
    }

    viscorr.resetCamera ();  // sets to default view
    // viscorr.spin (); // runs the window

    std::ostringstream name;
    name << "downsampled_points" << beta << " " << iteration  << ".png";
    std::string str_name(name.str());
    viscorr.saveScreenshot(str_name);
}



void AlternatingLeastSquares::visualize_putative_points() {
    std::cout << "HERE!!!!" << std::endl;
    // convert both clouds into RGB
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pu_source (new pcl::PointCloud<pcl::PointXYZRGB>);
    for (int i=0; i < ncorrs; i++){
        pcl::PointXYZRGB point = pcl::PointXYZRGB(0, 0, 255);
        point.x = eigen_data.putative_source(i,0); point.y = eigen_data.putative_source(i,1); point.z = eigen_data.putative_source(i,2);
        pu_source->push_back(point);
    }

    std::cout << "ga!!!!" << std::endl;

    // convert both clouds into RGB
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pu_target (new pcl::PointCloud<pcl::PointXYZRGB>);
    for (int i=0; i < ncorrs; i++){
        pcl::PointXYZRGB point = pcl::PointXYZRGB(255, 0, 0);
        point.x = eigen_data.putative_target(i,0); point.y = eigen_data.putative_target(i,1); point.z = eigen_data.putative_target(i,2);
        pu_target->push_back(point);
    }
    std::cout << "ba!!!!" << std::endl;

    // initialize visualization
    pcl::visualization::PCLVisualizer viscorr;

    // add both clouds to visualizer
    viscorr.addPointCloud(pu_source->makeShared(), "src_points");
    viscorr.addPointCloud(pu_target->makeShared(), "tgt_points");

    for (size_t i = 0; i < ncorrs; i ++)
    {   
        // get respective points from each cloud
        const pcl::PointXYZRGB & p_src = pu_source->points.at(i);
        const pcl::PointXYZRGB & p_tgt = pu_target->points.at(i);

        std::stringstream ss ("line");
        ss << i;
        std::stringstream sss ("spheresource");
        sss << i;
        std::stringstream ssss ("spheretarget");
        ssss << i;

        // draw the line objects onto the point clouds
        float dist = (eigen_data.putative_target.row(i) - eigen_data.putative_source.row(i)).squaredNorm();
        if(dist < 3)
        {
            //this is for red lines and spheres
            viscorr.addSphere<pcl::PointXYZRGB>(p_src,0.25,0,0,255,sss.str());
            viscorr.addSphere<pcl::PointXYZRGB>(p_tgt,0.25,0,0,255,ssss.str());
            viscorr.addLine<pcl::PointXYZRGB> (p_src, p_tgt, 0, 0, 255, ss.str ());      
        }
        else
        {
            //this is for red ones
            viscorr.addSphere<pcl::PointXYZRGB>(p_src,0.25,255,255,0,sss.str());
            viscorr.addSphere<pcl::PointXYZRGB>(p_tgt,0.25,255,255,0,ssss.str());
            viscorr.addLine<pcl::PointXYZRGB> (p_src, p_tgt, 255, 0, 0, ss.str ());
        }

        // Generate a unique string for each line
    }

    std::cout << "sock!!!!" << std::endl;


    viscorr.resetCamera ();  // sets to default view
    // viscorr.spin (); // runs the window


    std::ostringstream name;
    name << "point_matches" << beta << " " << iteration << ".png";
    std::string str_name(name.str());
    viscorr.saveScreenshot(str_name);
}

// =========================== Private Methods ============================

void AlternatingLeastSquares::update_P(float gamma, float variance){
    Eigen::MatrixXf Z = eigen_data.putative_target - current_X;
    for (int i = 1; i < ncorrs; i++){
        double squared_norm = Z.row(i).squaredNorm();
        double part1 = ((double) gamma) * std::exp(- squared_norm / (2. * ((double) variance)));
        double part2 = (1. - ((double) gamma) ) * pow(2. * M_PI * variance, dims/2.) / a;
        // std::cout << "P part squared norm: " << squared_norm << std::endl;
        // std::cout << "P part variance: " << variance << std::endl;
        // std::cout << "P ratio: " << (squared_norm / (2. * variance)) << std::endl;

        double p_i = part1 / (part1 + part2);
        P(i,i) = (float) p_i;
    }

};

/**
 * Updates the variance
 */
float AlternatingLeastSquares::update_variance(Eigen::MatrixXf P){
    return ((eigen_data.putative_target-current_X).transpose() * P *
     (eigen_data.putative_target-current_X)).trace() / (dims * P.trace());
}

/**
 * Updates c
 */
void AlternatingLeastSquares::update_C(Eigen::MatrixXf &U, Eigen::MatrixXf &V, float variance) {
    // compute the left side A in the Ax=b problem
    // std::printf("Rows: %ld, Cols: %ld\n", U.rows(), U.cols());
    // std::printf("Rows: %ld, Cols: %ld\n", V.rows(), V.cols());
    // std::printf("Rows: %ld, Cols: %ld\n", P.rows(), P.cols());
    // std::printf("Rows: %ld, Cols: %ld\n", Tau.rows(), Tau.cols());

    Eigen::MatrixXf first_part = U.transpose()*P*U;
    // std::cout << U(1,0) << " First" << std::endl;
    // std::cout << "======================" << std::endl;
    // std::cout << U(0,0) << std::endl;
    // std::cout << U(18,1) << std::endl;
    // std::cout << U(19,1) << std::endl;
    // std::cout << U(20,0) << std::endl;
    // std::cout << U(21,0) << std::endl;    // std::printf("2.61\n");
    Eigen::MatrixXf second_part = lambda1 * Tau * variance; // THIS IS DEVIATING!!!
    std::cout << second_part(1,0) << " Second" << std::endl;
    std::printf("2.62\n");    
    // std::printf("Rows: %ld, Cols: %ld\n", laplacian_approx.rows(), laplacian_approx.cols());
    Eigen::MatrixXf third_part = lambda2 * V.transpose() * laplacian * V *  variance;
    std::cout << third_part(1,0) << " Third" << std::endl;
    // std::printf("2.63\n");
    std::cout << "First part sum: " << first_part.sum() << std::endl;
    std::cout << "Second part sum: " << second_part.sum() << std::endl;
    std::cout << "Third part sum: " << third_part.sum() << std::endl;

    Eigen::MatrixXf A = first_part + second_part + third_part; // n x n
    std::printf("2.64\n");

    // compute the right side b in the Ax=b problem
    Eigen::MatrixXf b = U.transpose() * P * eigen_data.putative_target;  // (dense l by n) (dense l by l) (dense l by 3)

    // cout << A(1,0) << std::endl;
    // cout << A(2,0) << std::endl;
    // cout << A(3,0) << std::endl;
    // cout << A(5,0) << std::endl;

    // std::printf("2.65\n");
    // update C 

    // // make less of ill posed problem
    // for (int i = 0; i < nsamples; i++){
    //     for (int j = 0; j < nsamples; j++){
    //         A(i,j) += .001;
    //     }
    // }
    // for (int i = 0; i < nsamples; i++){
    //     for (int j = 0; j < 3; j++) {
    //         b(i,j) += .001;
    //     }
    // }
    // this->C = A.fullPivHouseholderQr().solve(b); // SHOULD VERIFY THIS!!'

    // ATTEMPT GRADIENT DESCENT --> GO TO OFFICE HOURS!
    this->C =  A.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(b);
    std::cout << "First 5 rows of C: " << std::endl << std::endl;
    std::cout << C.row(0) << std::endl;
    std::cout << C.row(1) << std::endl;
    std::cout << C.row(2) << std::endl;
    std::cout << C.row(3) << std::endl;
    std::cout << C.row(4) << std::endl;

}

/**
 * Computes nsamples completely random points in X_downsampled from
 * the source.
 * DEBUGGED AND GOOD TO GO
 */
void AlternatingLeastSquares::compute_X_downsampled() {
    if (nsamples == -1) {
        nsamples = npoints;
    }

    // this algorithm may be weird, but note that it generates
    // nsample unique values!
    int *rand_indices = new int[nsamples];
    int *used_indices = new int[npoints];
    for (int i = 0; i< npoints; i++){
        used_indices[i] = i;
    }
    srand(time(0));  // Initialize random number generator.
    for (int i = 0; i < nsamples; i++){
        int r = (rand() % (npoints - i)); //npoints-1 represents max index val
        rand_indices[i] = used_indices[r];
        used_indices[r] = (npoints-i-1); // index of current upper threshold on % operator
    }


    // now update our X to be the random number
    X_downsampled.resize(nsamples, dims); // construct X_downsampled
    for (int i = 0; i < nsamples; i++){
        X_downsampled.row(i) = eigen_data.source.row(rand_indices[i]);
    }

    // // deallocate memory
    // delete [] rand_indices;
    // delete [] used_indices;

    // std::cout <<"entered" <<std::endl;
    // X_downsampled.resize(nsamples, dims); // construct X_downsampled
    // for (int i = 0; i < nsamples; i++){
    //     X_downsampled.row(i) = eigen_data.putative_source.row(i);
    // }
    // std::cout <<"out of it" <<std::endl;

}


