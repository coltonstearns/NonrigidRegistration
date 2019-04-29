#include <pcl/io/pcd_io.h>  // for working with point cloud data structures
#include <pcl/impl/point_types.hpp>  // for defining point objects
#include <pcl/features/normal_3d.h>
#include <pcl/point_types.h>
#include <pcl/features/fpfh.h>  // fpfh object
#include <pcl/registration/ia_ransac.h>  // for sample consensus alignment object
#include <pcl/common/transforms.h>  // for transforming point cloud to target with 6 degree of freedom matrix
#include <pcl/registration/correspondence_estimation.h> // for calculating distance correspondences

pcl::PointCloud<pcl::Normal>::Ptr getCloudNormals(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
pcl::PointCloud<pcl::FPFHSignature33>::Ptr computeFPFH(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr normals, float radius);
pcl::Correspondences calculateCorrespondences(pcl::PointCloud<pcl::PointXYZ>::Ptr data, pcl::PointCloud<pcl::PointXYZ>::Ptr target, float fpfh_radius);