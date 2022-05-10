#include <fstream>
#include <vector>
#include <string>
#include <algorithm>
#include <sstream>
#include <configuration.h>

void config::Configs::loadConfigs(std::string file_path)
{
    std::ifstream ifs(file_path);
    std::string str;

    if (ifs.fail())
    {
        std::cerr << "Failed to open file." << std::endl;
        return;
    }

    int line=0;

    while (getline(ifs, str))
    {
        line++;

        if (str.length()==0 or str[0]=='#') continue;

        std::vector<std::string> block;
        std::stringstream ss{str};
        std::string s;

        while(getline(ss, s, '='))
        {
            s.erase(std::remove(s.begin(), s.end(), ' '), s.end());
            block.push_back(s);
        }

        if (block.size()!=2)
        {
            std::cout << "Invalid config in line: " << line << ". " << "Config file: \"" << file_path << "\"" << std::endl;
            return;
        }

        // Calibration
        if (block[0] == "PCD_DATA") is_pcd_data = (block[1]=="true")? true:false;
        else if (block[0] == "SHOW_ORIGINAL") show_original_supervoxel = (block[1]=="true")? true:false;
        else if (block[0] == "USE_REDUCED_PCD") use_reduced_pcd = (block[1]=="true")? true:false;
        else if (block[0] == "SAVE_PCD") save_pcd = (block[1]=="true")? true:false;
        else if (block[0] == "WIDTH") width = std::stoi(block[1]);
        else if (block[0] == "HEIGHT") height = std::stoi(block[1]);
        else if (block[0] == "FX") fx = std::stof(block[1]);
        else if (block[0] == "FY") fy = std::stof(block[1]);
        else if (block[0] == "CX") cx = std::stof(block[1]);
        else if (block[0] == "CY") cy = std::stof(block[1]);
        // Clipping
        else if (block[0] == "MIN_X") min_x = std::stof(block[1]);
        else if (block[0] == "MAX_X") max_x = std::stof(block[1]);
        else if (block[0] == "MIN_Y") min_y = std::stof(block[1]);
        else if (block[0] == "MAX_Y") max_y = std::stof(block[1]);
        else if (block[0] == "MIN_Z") min_z = std::stof(block[1]);
        else if (block[0] == "MAX_Z") max_z = std::stof(block[1]);
        // Supervoxel
        else if (block[0] == "DISABLE_TRANSFORM") disable_transform = (block[1]=="true")? true:false;
        else if (block[0] == "VOXEL_RESOLUTION") voxel_resolution = std::stof(block[1]);
        else if (block[0] == "SEED_RESOLUTION") seed_resolution = std::stof(block[1]);
        else if (block[0] == "COLOR_IMPORTANCE") color_importance = std::stof(block[1]);
        else if (block[0] == "SPATIAL_IMPORTANCE") spatial_importance = std::stof(block[1]);
        else if (block[0] == "NORMAL_IMPORTANCE") normal_importance = std::stof(block[1]);
        // Region merge
        else if (block[0] == "MIN_VOXEL_SIZE") min_voxel_size = std::stoi(block[1]);
        else if (block[0] == "MAX_EPOCHS") max_epochs = std::stoi(block[1]);
        else if (block[0] == "MIN_CLUSTERS") min_clusters = std::stoi(block[1]);
        else if (block[0] == "COS_THRESHOLD") cos_threshold = std::stof(block[1]);
        else if (block[0] == "DISTANCE_THRESHOLD") distance_threshold = std::stof(block[1]);
        else std::cout << block[0] << "is not a config param name. Line: "<< line << ". " << "Config file: \"" << file_path << "\"" << std::endl;
    }
    
    std::cout << "Load config completed." << std::endl;
    return;
};

void config::Configs::getFlags(
    bool* _is_pcd_data,
    bool* _show_original_supervoxel,
    bool* _use_reduced_pcd,
    bool* _save_pcd
)
{
    *_is_pcd_data = is_pcd_data;
    *_show_original_supervoxel = show_original_supervoxel;
    *_use_reduced_pcd = use_reduced_pcd;
    *_save_pcd = save_pcd;
};

void config::Configs::getCalibrationParams(
                int* _width,
                int* _height,
                float* calib
            )
{
    *_width = width;
    *_height = height;
    calib[0] = fx;
    calib[1] = fy;
    calib[2] = cx;
    calib[3] = cy;
};

void config::Configs::getClippingParams(
                float* clipping
            )
{
    clipping[0] = min_x;
    clipping[1] = max_x;
    clipping[2] = min_y;
    clipping[3] = max_y;
    clipping[4] = min_z;
    clipping[5] = max_z;
}

void config::Configs::getSupervoxelParams(
                bool* _disable_transform,
                float* supoervoxel_params
            )
{
    *_disable_transform = disable_transform;
    supoervoxel_params[0] = voxel_resolution;
    supoervoxel_params[1] = seed_resolution;
    supoervoxel_params[2] = color_importance;
    supoervoxel_params[3] = spatial_importance;
    supoervoxel_params[4] = normal_importance;
};

void config::Configs::getRegionMergeParams(
                int* _min_voxel_size,
                int* _max_epochs,
                int* _min_clusters,
                float* _cos_threshold,
                float* _distance_threshold
            )
{
    *_min_voxel_size = min_voxel_size;
    *_max_epochs = max_epochs;
    *_min_clusters = min_clusters;
    *_cos_threshold = cos_threshold;
    *_distance_threshold = distance_threshold;
};