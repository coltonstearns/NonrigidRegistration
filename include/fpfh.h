#include <pcl/io/pcd_io.h>  // for working with point cloud data structures
#include <pcl/impl/point_types.hpp>  // for defining point objects
#include <pcl/features/normal_3d.h>
#include <pcl/point_types.h>
#include <pcl/features/fpfh.h>  // fpfh object
#include <pcl/registration/correspondence_estimation.h> // for calculating distance correspondences

// visualization script
#include <pcl/visualization/pcl_visualizer.h>  // visualizer 
#include <pcl/common/transforms.h>

pcl::PointCloud<pcl::Normal>::Ptr getCloudNormals(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
pcl::PointCloud<pcl::FPFHSignature33>::Ptr computeFPFH(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr normals, float radius);
pcl::Correspondences calculateCorrespondences(pcl::PointCloud<pcl::PointXYZ>::Ptr data, pcl::PointCloud<pcl::PointXYZ>::Ptr target, float fpfh_radius);
void visualize_correspondences(pcl::PointCloud<pcl::PointXYZ>::Ptr source_raw, pcl::PointCloud<pcl::PointXYZ>::Ptr target_raw, pcl::Correspondences correspondences);
