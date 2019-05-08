// ============ THESE TWO HEADERS NEED TO BE IN THIS ORDER! ===============
#include <pcl/io/pcd_io.h>  // for working with point cloud data structures
#include <pcl/impl/point_types.hpp>  // for defining point objects
// =========================================================================

// struct to store the TOSCA cat point clouds
struct catData {
    pcl::PointCloud<pcl::PointXYZ>::Ptr source;
    pcl::PointCloud<pcl::PointXYZ>::Ptr target;
};

catData generateCatPointCloud();
