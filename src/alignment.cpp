// class header file
#include "alignment.h"


/**
 * Constructor for default cat data
 */
NonrigidAlign::NonrigidAlign(){
    data = generateCatPointCloud();
    pcl::PointCloud<pcl::PointXYZ>::Ptr putative_source_ptr (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr putative_target_ptr (new pcl::PointCloud<pcl::PointXYZ>);
    putative_source = putative_source_ptr;
    putative_target = putative_target_ptr;
};

void NonrigidAlign::getCorrespondences(){
    correspondences = calculateCorrespondences(data.source, data.target, 8.9); // 8.9 is optimal value
    pcl::PointCloud<pcl::PointXYZ>::Ptr putative_source (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr target (new pcl::PointCloud<pcl::PointXYZ>);
}


/**
 * Creates two points clouds that only contain the corresponding points from the
 * FPFH matching
 */
void NonrigidAlign::getPutativeCorrespondenceSets(){

    for (int i = 0; i < correspondences.size(); i++){
        pcl::Correspondence corr = correspondences.at(i);
        putative_source->push_back(data.source->at(corr.index_query));
        putative_target->push_back(data.target->at(corr.index_match));
    }
}

/**
 * Displays 3D view of correspondencess
 */
void NonrigidAlign::displayCorrespondences(){
    std::cout << data.source->width << std::endl;
    std::cout << data.target->width << std::endl;
    std::cout << correspondences.size() << std::endl;
    visualize_correspondences(data.source, data.target, correspondences);
}

/**
 * computes the Graph Laplacian of the ALL source points
 * This is the matrix A from the paper
 */
void NonrigidAlign::computeLaplacian() {
    getLaplacian(data.source, laplacian);
}



// class NonrigidAlign {
//     private:
//         catData data;
//         pcl::Correspondences correspondences;
//         Eigen::SparseMatrix<float> laplacian;
//         pcl::PointCloud<pcl::PointXYZ>::Ptr putative_source;
//         pcl::PointCloud<pcl::PointXYZ>::Ptr putative_target;


//     public:
//         /**
//          * Constructor for default cat data
//          */
//         NonrigidAlign() {
//             data = generateCatPointCloud();
//         }

//         /**
//          * Computes FPFH Correspondences that are valid between the source and target
//          * of the defined data
//          */
//         void getCorrespondences() {
//             correspondences = calculateCorrespondences(data.source, data.target, 8.9); // 8.9 is optimal value
//             // putative_source (new pcl::PointCloud<pcl::PointXYZ>);
//             // putative_target (new pcl::PointCloud<pcl::PointXYZ>);
//         }

//         /**
//          * Creates two points clouds that only contain the corresponding points from the
//          * FPFH matching
//          */
//         void getPutativeCorrespondenceSets(){
//             for (int i = 0; i < correspondences.size(); i++){
//                 pcl::Correspondence corr = correspondences.at(i);
//                 putative_source->push_back(data.source->at(corr.index_query));
//                 putative_target->push_back(data.target->at(corr.index_match));
//             }
//         }

//         /**
//          * Displays 3D view of correspondences
//          */
//         void displayCorrespondences(){
//             visualize_correspondences(data.source, data.target, correspondences);
//         }

//         /**
//          * computes the Graph Laplacian of the ALL source points
//          * This is the matrix A from the paper
//          */
//         void computeLaplacian() {
//             getLaplacian(data.source, laplacian);
//         }


// };