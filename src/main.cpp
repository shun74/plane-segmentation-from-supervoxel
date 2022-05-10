#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <chrono>
#include <region_manager.h>
#include <loader.h>
#include <utils.h>
#include <configuration.h>

int main (int argc, char *argv[]) {
    
    auto start = std::chrono::system_clock::now();
    // Load parameters config file
    config::Configs configs;
    configs.loadConfigs(argv[argc-1]);
    // Get some flags
    bool is_pcd_data;
    bool show_original_supervoxel;
    bool use_reduced_pcd;
    bool save_pcd;
    configs.getFlags(&is_pcd_data, &show_original_supervoxel, &use_reduced_pcd, &save_pcd);
    // Clipping parameters
    float clipping[6];
    configs.getClippingParams(clipping);
    // Supervoxel parameters
    bool disable_transform;
    float supervoxel_parameters[5];
    configs.getSupervoxelParams(&disable_transform, supervoxel_parameters);
    // Region merge parameters
    int min_voxel_size;
    int max_epochs;
    int min_clusters;
    float cos_threshold;
    float distance_threshold;
    configs.getRegionMergeParams(&min_voxel_size, &max_epochs, &min_clusters, &cos_threshold, &distance_threshold);

    // Print config details
    std::cout << "Configs:########################" << std::endl;
    if (is_pcd_data) std::cout << "-pcd format data" << std::endl;
    else std::cout << "-2d format data" << std::endl;
    if (show_original_supervoxel) std::cout << "-show original super voxel" << std::endl;
    if (use_reduced_pcd) std::cout << "-using reduced pcd for visualizer" << std::endl;
    if (save_pcd) std::cout << "-save labeled pcd" << std::endl;
    std::cout << "Clipping range: " << clipping[0] << "<x<" << clipping[1] << ", " << clipping[2] << "<y<" << clipping[3] << ", " << clipping[4] << "<z<" << clipping[5] << std::endl;
    std::cout << "Supervoxel: " << "voxel_resolution=" << supervoxel_parameters[0] << ", " << "seed_resolution=" << supervoxel_parameters[1] << std::endl;
    std::cout << "            " << "color_importance=" << supervoxel_parameters[2] << ", " << "spatial_importance=" << supervoxel_parameters[3] << ", " << "normal_importance=" << supervoxel_parameters[4] << std::endl; 
    std::cout << "Region merge: " << "min_voxel_size=" << min_voxel_size << ", " << "max_epochs=" << max_epochs << ", " << "min_clusters=" << min_clusters << std::endl;
    std::cout << "              " << "cosine_threhsold=" << cos_threshold << ", " << "distance_threshold=" << distance_threshold << std::endl;


    // Convert depth to PointCloud
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);

    if (is_pcd_data)
    {
        std::string pcd_data_file = argv[1];

        cloud = loader::LoadPointCloud(pcd_data_file, clipping);
    }
    else
    {
        std::string color_image_file = argv[1];
        std::string depth_image_file = argv[2];
        
        // Calibration parameters
        int width, height;
        float calib[4];
        configs.getCalibrationParams(&width, &height, calib);
        std::cout << "Calibration: " << "fx=" << calib[0] << ", " << "fy=" << calib[1] << ", " << "cx=" << calib[2] << ", " << "cy=" << calib[3] << std::endl;

        cv::Mat color_image;
        std::vector<float> depth_image;
        std::vector<std::string> abc;
        loader::LoadRGBImage(color_image_file, &color_image);
        loader::LoadRawImage(depth_image_file, width, height, &depth_image);

        cloud = loader::Image2PointCloud(&color_image, &depth_image, calib, clipping);
    }
    std::cout << "################################" << std::endl;


    // Supervoxel
    pcl::SupervoxelClustering<pcl::PointXYZRGBA> super (supervoxel_parameters[0], supervoxel_parameters[1]);
    if (disable_transform) super.setUseSingleCameraTransform (false);
    super.setInputCloud (cloud);
    super.setColorImportance (supervoxel_parameters[2]);
    super.setSpatialImportance (supervoxel_parameters[3]);
    super.setNormalImportance (supervoxel_parameters[4]);

    // Run supoervoxel clustering
    std::map<uint32_t, pcl::Supervoxel<pcl::PointXYZRGBA>::Ptr > supervoxel_clusters;
    super.extract (supervoxel_clusters);

    // Get full labeled point cloud
    pcl::PointCloud<pcl::PointXYZL>::Ptr labeled_cloud;
    if (use_reduced_pcd) labeled_cloud = super.getLabeledVoxelCloud();
    else labeled_cloud = super.getLabeledCloud();
    std::cout << "Original supervoxel clusters: " << supervoxel_clusters.size() << std::endl;
    
    // Visualize original labeled point cloud
    if (show_original_supervoxel)
    {
        pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("Original voxel"));
        viewer->setBackgroundColor (0, 0, 0);
        viewer->addPointCloud (labeled_cloud, "original voxel");
        viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_OPACITY,0.8, "original voxel");
        while (!viewer->wasStopped ())
        {
            viewer->spinOnce (100);
        }
    }

    // Region merge
    region_manager::threshVoxels(&supervoxel_clusters, labeled_cloud, min_voxel_size);
    region_manager::RegionManager manager;
    manager.setInput(&supervoxel_clusters);
    std::map<int, int> merge_map;
    merge_map = manager.regionMerge(max_epochs, min_clusters, cos_threshold, distance_threshold);
    region_manager::updateLabels(merge_map, labeled_cloud);

    // Elapsed time
    auto end = std::chrono::system_clock::now();
    double  elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end-start).count();
    std::cout << "Region merge complete. " << std::endl;
    std::cout << "Elapsed time: " << elapsed << "ms." << std::endl;

    // Visualize labeled point cloud
    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("Merged point cloud"));
    viewer->setBackgroundColor (0, 0, 0);
    viewer->addPointCloud (labeled_cloud, "merged voxel");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_OPACITY,0.8, "merged voxel");
    while (!viewer->wasStopped ())
    {
        viewer->spinOnce (100);
    }

    // Save labeled pcd
    if (save_pcd) pcl::io::savePCDFileASCII("../outputs/labeled.pcd", *labeled_cloud);
}