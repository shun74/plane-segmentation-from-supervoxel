#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/supervoxel_clustering.h>
#include <opencv2/opencv.hpp>
#include <iostream>

namespace utils
{

    float calDistance(cv::Vec3f u, cv::Vec3f v);

    float cosSimilarity(cv::Vec3f u, cv::Vec3f v);

}

namespace region_manager 
{

    void threshVoxels(std::map<uint32_t, pcl::Supervoxel<pcl::PointXYZRGBA>::Ptr>* supervoxel_clusters, pcl::PointCloud<pcl::PointXYZL>::Ptr cloud, int min_size);

    void updateLabels(std::map<int, int> &merge_map, pcl::PointCloud<pcl::PointXYZL>::Ptr labeled_cloud);

}