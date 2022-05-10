#include <iostream>

namespace config
{
    class Configs
    {
        private:
            // Calibration data
            int width, height;
            float fx, fy, cx, cy;
            // Clipping parameter
            float min_x, max_x, min_y, max_y, min_z, max_z;

            // Supervoxel parameters
            bool disable_transform;
            float voxel_resolution;
            float seed_resolution;
            float color_importance;
            float spatial_importance;
            float normal_importance;

            // Reigion merge parameters
            int min_voxel_size;
            int max_epochs;
            int min_clusters;
            float cos_threshold;
            float distance_threshold;

        public:
            bool is_pcd_data;
            bool show_original_supervoxel;
            bool use_reduced_pcd;
            bool save_pcd;

            // Configuration file parser
            void loadConfigs(std::string fine_path);

            void getFlags(
                bool* _is_pcd_data,
                bool* _show_original_supervoxel,
                bool* _use_reduced_pcd,
                bool* _save_pcd
            );
            
            // Configuration getters
            void getCalibrationParams(
                int* _width,
                int* _height,
                float* calib
            );

            void getClippingParams(
                float* clipping
            );
            
            void getSupervoxelParams(
                bool* _disable_transform,
                float* supervoxel_parameters
            );

            void getRegionMergeParams(
                int* _min_voxel_size,
                int* _max_epochs,
                int* _min_clusters,
                float* _cos_threshold,
                float* _distance_threshold
            );
    };
}