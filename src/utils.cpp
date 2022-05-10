#include <pcl/io/pcd_io.h>
#include <utils.h>


/* Calculate cosine similarity */
float utils::cosSimilarity(cv::Vec3f u, cv::Vec3f v)
{
  float u_l = sqrt(u[0]*u[0] + u[1]*u[1] + u[2]*u[2]);
  float v_l = sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
  return u.dot(v) / (u_l * v_l);
}

/* Calculate distance */
float utils::calDistance(cv::Vec3f u, cv::Vec3f v)
{
  return sqrt(pow(u[0]-v[0],2) + pow(u[1]-v[1],2) + pow(u[2]-v[2],2));
}

/* Delete small size voxel */
void region_manager::threshVoxels(std::map<uint32_t, pcl::Supervoxel<pcl::PointXYZRGBA>::Ptr>* supervoxel_clusters, pcl::PointCloud<pcl::PointXYZL>::Ptr cloud, int min_size) 
{
  std::map<int, int> label_map;

  for (auto supervoxel_iter=supervoxel_clusters->begin(), supervoxel_iter_last=supervoxel_clusters->end(); supervoxel_iter!=supervoxel_iter_last;)
  {
    int label = supervoxel_iter->first;
    pcl::Supervoxel<pcl::PointXYZRGBA>::Ptr cluster = supervoxel_clusters->at(label);
  
    if (cluster->voxels_->size() < min_size)
    {
      supervoxel_iter = supervoxel_clusters->erase(supervoxel_iter);
      supervoxel_iter_last = supervoxel_clusters->end();
  
      for (auto cloud_iter=cloud->begin(), cloud_iter_last=cloud->end(); cloud_iter!=cloud_iter_last;)
      {
        if (cloud_iter->label == label) 
        {
          cloud_iter = cloud->erase(cloud_iter);
          cloud_iter_last = cloud->end();
        }
        
        else ++cloud_iter;
      }
    }
    else ++supervoxel_iter;
  }
  return;
}

/* Update original labels with merge map */
void region_manager::updateLabels(std::map<int, int> &merge_map, pcl::PointCloud<pcl::PointXYZL>::Ptr labeled_cloud)
{
  for (auto labeled_point_iter=labeled_cloud->begin(); labeled_point_iter!=labeled_cloud->end(); labeled_point_iter++)
  {
    if (merge_map.count(labeled_point_iter->label)!=0) labeled_point_iter->label = merge_map[labeled_point_iter->label];
  }
}