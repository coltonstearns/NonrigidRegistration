#include "utils/laplacian.h"
#include <tgmath.h>


Eigen::SparseMatrix<float> getLaplacian(pcl::PointCloud<pcl::PointXYZ>::Ptr source, float epsilon, float sparsity_threshold){
    // define our ratio epsilon
    

    // get time
    time_t start_time;
	start_time = time(NULL);
    
    // get the matrix of the source points
    int num_source_points = int (source->width);
    std::cout << "Number of data points: " << num_source_points << std::endl;

    // compute symmetric x differences matrix
    // SPEED UP: search through KD tree and get k nearest neighbors; compute graph laplacian based on that
    Eigen::MatrixXf edge_weights = Eigen::MatrixXf::Zero(num_source_points,num_source_points);
    float curr_weight;
    for (int i = 0; i < num_source_points; i++) {
        for (int j = 0; j <= i; j++){
            float dist = pow((source->at(i).x - source->at(j).x), 2) + pow((source->at(i).y - source->at(j).y), 2) + pow((source->at(i).z - source->at(j).z), 2);
            if (dist < epsilon){
                if (i != j){
                    curr_weight = exp(- dist / epsilon);
                    edge_weights(i,j) = curr_weight;
                    edge_weights(j,i) = curr_weight;
                }
            }
        }
    }
    Eigen::SparseMatrix<float> edge_weights_sparse = edge_weights.sparseView();
    std::cout << "Weight Matrix Num Nonzero Entries: " << edge_weights_sparse.nonZeros() << std::endl;

    // compute diagonal vertex weight matrix
    Eigen::SparseMatrix<float> vertex_vals(num_source_points,num_source_points);
    for (int i = 0; i < num_source_points; i++) {
        float vertex_weight = edge_weights_sparse.row(i).sum();
        vertex_vals.insert(i,i) = vertex_weight;
    }

    // calculate Laplacian
    Eigen::SparseMatrix<float> laplacian = vertex_vals - edge_weights_sparse;

    time_t end_time;
	end_time = time(NULL);
    std::cout << "Time Elapsed: " << end_time - start_time << std::endl;

    std::cout << "Num Nonzero entries in Laplacian " << laplacian.nonZeros() << std::endl;

    return laplacian;
}



/**
 * Computes the k largest eigenvalues and eigenvectors of the laplacian.
 * The laplacian must be sparse, symmetric, and positive semidefinite by definition.
 */
LaplaceEigenInfo approximate_laplacian(Eigen::SparseMatrix<float> &laplacian, int neigs) {
    // Construct matrix operation object using the wrapper class SparseGenMatProd
    Spectra::SparseSymMatProd<float> op(laplacian);

    // the 3rd argument is "ncv" which: num_eig_vals + 2 < ncv < npoints; a larger ncv indicates faster computation time
    Spectra::GenEigsSolver<float, Spectra::SMALLEST_MAGN, Spectra::SparseSymMatProd<float> > eigs(&op, neigs, neigs+50);
    std::cout << "hereb" << std::endl;

    // Initialize and compute eigenvalues / vectors
    eigs.init();
    int nconv = eigs.compute();

    // Retrieve results
    LaplaceEigenInfo laplace_info;
    laplace_info.neigs = neigs;

    if (eigs.info() == Spectra::SUCCESSFUL) {
        Eigen::VectorXcf evals = eigs.eigenvalues();
        Eigen::MatrixXcf evecs = eigs.eigenvectors();
        laplace_info.eigenvalues = evals.real();
        laplace_info.eigenvectors = evecs.real();
    }
    else{
        std::cout << "failed" << std::endl;
    }

    return laplace_info;
}


/**
 * Visualizes the laplacian eigenvectors and values on the points cloud
 */
void visualize_laplacian(pcl::PointCloud<pcl::PointXYZ>::Ptr lap_points, Eigen::VectorXf eigenvector) {
    // compute laplacian eigenvalues and eigenvectors
    // LaplaceEigenInfo laplace_info = approximate_laplacian(laplacian, neigs);

    // get min and max vals in the eigenvector
    // std::cout << "rows: " << laplace_info.eigenvectors.rows() << " cols: " << laplace_info.eigenvectors.cols() << std::endl;
    // Eigen::Block<Eigen::MatrixXf, -1, 1, true> fourth_eigvec = laplace_info.eigenvectors.col(eig_number);

    // get min and max for visualizing color scheme
    float max_val = eigenvector.maxCoeff();
    float min_val = eigenvector.minCoeff();
    std::cout << "Max and min: " << max_val << " " << min_val << std::endl;

    // initialize visualization
    pcl::visualization::PCLVisualizer viscorr;

    // put color on the cloud according to laplace
    int npoints = lap_points->size();
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_points (new pcl::PointCloud<pcl::PointXYZRGB>);
    for (int i=0; i < npoints; i++){
        Color color = getColor(eigenvector(i), max_val, min_val);
        uint8_t red = roundf(color.r * 255);
        uint8_t green = roundf(color.g * 255);
        uint8_t blue = roundf(color.b * 255);
        pcl::PointXYZRGB point = pcl::PointXYZRGB(red, green, blue);
        point.x = lap_points->at(i).x; point.y = lap_points->at(i).y; point.z = lap_points->at(i).z;
        colored_points->push_back(point);

        std::stringstream sss ("spheresource");
        sss << i;
        viscorr.addSphere<pcl::PointXYZRGB>(point,0.5,red,blue,green,sss.str());
    }


    // add both clouds to visualizer
    viscorr.addPointCloud(colored_points->makeShared(), "src_points");

    // run the visualization
    viscorr.resetCamera ();  // sets to default view
    viscorr.spin (); // runs the window

    // delete laplace_info;
}

Color getColor(float value, float min_value, float max_value)
{
    Color c;
    c.b = 1;
    c.r = 1;
    c.g = 1;
    double dv;
    // Unknown values in our kernel
    if(abs(value) < 0.000001)
    {
        return c;
    }

    // Interval
    dv = (double) (max_value - min_value);
    // Seismic color map like
    if(value < (min_value + 0.5 * dv))  // if in lower half of distribution
    {
        c.r = (value - min_value) / dv;
        c.g = (value - min_value) / dv;
        c.b = 1; // make it primarily blue
    }
    else
    {
        c.b = (value - min_value) / dv;
        c.g = (value - min_value) / dv;
        c.r = 1.;
    }

    return c;
}