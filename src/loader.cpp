#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <loader.h>

const size_t get_file_byte_size(std::ifstream& file)
{
    file.seekg(0, std::ios::end);
    const size_t file_size = (size_t)file.tellg();
    file.seekg(0, std::ios::beg);

    return file_size;
}

/* Load 16bit .raw format depth image */
void loader::LoadRawImage(std::string file_name, int width, int height, std::vector<float>* depth_image)
{
    std::vector<uint16_t> raw_data;
    std::ifstream file(file_name, std::ios::in | std::ios::binary);

    if (!file)
    {
      std::cout << "Error: The file path is incorrect. There is no file." << std::endl;
      return;
    }

    const size_t file_size = get_file_byte_size(file);
    raw_data.clear();
    raw_data.resize(file_size / 2); // 16 bit = 2 bytes.
  
    file.read((char*)raw_data.data(), file_size);
    file.close();

    for (int i=0; i<raw_data.size();i++)
    {
        // For Kinect_v2 use 0.001
        depth_image->push_back(raw_data[i]*0.001);
        // For Realsence use 0.00025
        // depth_image->push_back(raw_data[i]*0.00025);
    }
    return;
}

/* Load normal RGB image */
void loader::LoadRGBImage(std::string file_name, cv::Mat* color_image)
{
    *color_image = cv::imread(file_name);
    return;
}

/* Convert depth and rgb image to point cloud */
pcl::PointCloud<pcl::PointXYZRGBA>::Ptr loader::Image2PointCloud(cv::Mat* color_img, std::vector<float>* depth_img, float* calib, float* clipping)
{
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
    int step = color_img->step;
    int elemSize = color_img->elemSize();
    int width = color_img->cols;
    int height = color_img->rows;
    float fx = calib[0];
    float fy = calib[1];
    float cx = calib[2];
    float cy = calib[3];
    float min_x = clipping[0];
    float max_x = clipping[1];
    float min_y = clipping[2];
    float max_y = clipping[3];
    float min_z = clipping[4];
    float max_z = clipping[5];

    for (int py=0; py<height; py++)
    {
        for (int px=0; px<width; px++)
        {
            // Convert pixel coordinate system to camera coordinate system
            float z = depth_img->at(py*width + px);
            float x = z * (px - cx) / fx;
            float y = z * (py - cy) / fy;
            int b = color_img->data[py * step + px * elemSize + 0];
            int g = color_img->data[py * step + px * elemSize + 1];
            int r = color_img->data[py * step + px * elemSize + 2];

            // Constraint to remove redundant points
            if (min_x<x&&x<max_x and min_y<y&&y<max_y and min_z<z&&z<max_z)
            {
                pcl::PointXYZRGBA point;
                point.x = x;
                point.y = y;
                point.z = z;
                point.r = r;
                point.g = g;
                point.b = b;
                point.a = 255;
                cloud->push_back(point);
            };
        }
    }
    return cloud;
}

pcl::PointCloud<pcl::PointXYZRGBA>::Ptr loader::LoadPointCloud(std::string file_path, float* clipping)
{
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr full_cloud (new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGBA>);
    
    pcl::io::loadPCDFile(file_path, *full_cloud);

    float min_x = clipping[0];
    float max_x = clipping[1];
    float min_y = clipping[2];
    float max_y = clipping[3];
    float min_z = clipping[4];
    float max_z = clipping[5];

    for (auto point_iter=full_cloud->begin(); point_iter!=full_cloud->end(); point_iter++)
    {
        float x = point_iter->x;
        float y = point_iter->y;
        float z = point_iter->z;
        if (min_x<x&&x<max_x and min_y<y&&y<max_y and min_z<z&&z<max_z)
        {
            cloud->push_back(*point_iter);
        }
    }

    return cloud;
}