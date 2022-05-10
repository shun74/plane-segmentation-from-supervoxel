#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/supervoxel_clustering.h>
#include <opencv2/opencv.hpp>
#include <iostream>

namespace region_manager 
{

  class RegionManager
  {
    private:

      std::map<int, std::tuple<cv::Vec3f, cv::Vec3f, int>> data;

      std::map<int, std::set<std::pair<float, int>>> distance_pair;

      std::map<int, std::vector<int>> children;

      std::vector<int> survived;

      void updateDistances(int update_label, int delete_label);

    public:

      void setInput(std::map<uint32_t, pcl::Supervoxel<pcl::PointXYZRGBA>::Ptr>* supervoxel_clusters);

      std::map<int, int> regionMerge(int max_epoch, int min_clusters, float cos_thresh, float d_thresh);
  };

}