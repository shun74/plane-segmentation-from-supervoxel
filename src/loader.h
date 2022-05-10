#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <opencv2/opencv.hpp>
#include <iostream>

namespace loader
{
    void LoadRawImage(std::string file_name, int width, int height, std::vector<float>* depth_image);

    void LoadRGBImage(std::string file_name, cv::Mat* color_image);

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr Image2PointCloud(cv::Mat* color_img, std::vector<float>* depth_img, float* calib, float* clipping);

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr LoadPointCloud(std::string file_path, float* clipping);
}