#include <region_manager.h>
#include <utils.h>

/* Update distance_pair information when region merge occur */
void region_manager::RegionManager::updateDistances(int update_label, int delete_label)
{
    for (auto label_iter=survived.begin(); label_iter!=survived.end(); label_iter++)
    {
        std::set<std::pair<float, int>> d_pair = distance_pair[*label_iter];

        for (auto d_pair_iter=d_pair.begin(), d_last=d_pair.end(); d_pair_iter!=d_last;)
        {
            // Delete two merged region labels
            if (d_pair_iter->second==update_label or d_pair_iter->second==delete_label) d_pair_iter = d_pair.erase(d_pair_iter);
            else d_pair_iter++;
        }
        // Register new region distance information
        float distance = utils::calDistance(get<0>(data[update_label]), get<0>(data[*label_iter]));
        d_pair.insert(std::make_pair(distance, update_label));
    }
    return;
}

/* Exracting data from given supervoxel clusters */
void region_manager::RegionManager::setInput(std::map<uint32_t, pcl::Supervoxel<pcl::PointXYZRGBA>::Ptr>* supervoxel_clusters)
{
    for (auto supervoxel_iter=supervoxel_clusters->begin(); supervoxel_iter!=supervoxel_clusters->end(); supervoxel_iter++)
    {
        int label = supervoxel_iter->first;
        
        survived.push_back(label);

        pcl::Supervoxel<pcl::PointXYZRGBA>::Ptr cluster = supervoxel_clusters->at(label);
        
        pcl::PointNormal point_normal;
        cluster->getCentroidPointNormal(point_normal);
        
        cv::Vec3f centroid(point_normal.x, point_normal.y, point_normal.z);
        cv::Vec3f normal(point_normal.normal_x, point_normal.normal_y, point_normal.normal_z);
        
        // data: (centroid, normal, voxel size)
        data[label] = std::make_tuple(centroid, normal, cluster->voxels_->size());
    }
    
    // Initalize distance_pair
    for (auto label_iter1=survived.begin(); label_iter1!=survived.end(); label_iter1++)
    {
        for (auto label_iter2=survived.begin(); label_iter2!=survived.end(); label_iter2++)
        {
            if (*label_iter1==*label_iter2) continue;
            
            float distance = utils::calDistance(get<0>(data[*label_iter1]), get<0>(data[*label_iter2]));
            distance_pair[*label_iter1].insert(std::make_pair(distance, *label_iter2));
        }
    }
    return;
};


std::map<int, int> region_manager::RegionManager::regionMerge(int max_epoch, int min_clusters, float cos_threshold, float distance_threshold)
{
    bool merged = true;
    int epoch = 0;

    while(merged && epoch<max_epoch && survived.size()>min_clusters)
    {
        merged=false;

        for (auto label_iter=survived.begin(); label_iter!=survived.end(); label_iter++)
        {
            int label = *label_iter;
            
            // Distance and label pairs iteration
            for (auto distance_pair_iter=distance_pair[label].begin(); distance_pair_iter!=distance_pair[label].end(); distance_pair_iter++)
            {
            int target_label = distance_pair_iter->second;
            
            // Go to the next label when distance is farther than threshold
            if (distance_pair_iter->first > distance_threshold) break;

            epoch++;
            
            // Path the pair if its label is not included in survived
            if (!std::any_of(survived.begin(), survived.end(), [target_label](const int& x) {return x==target_label;})) continue;

            // If cosine similarity is lager than threshold, run region merge
            float cos_similarity = utils::cosSimilarity(get<1>(data[label]), get<1>(data[target_label]));
            if (abs(cos_similarity) < cos_threshold) continue;

            int size1 = get<2>(data[label]);
            int size2 = get<2>(data[target_label]);

            // Calculate normal and centroid by weighted avarage
            if (cos_similarity < 0) get<1>(data[label]) = -get<1>(data[label]);
            cv::Vec3f new_normal = (size1*get<1>(data[label]) + size2*get<1>(data[target_label])) / (size1 + size2);
            cv::Vec3f new_centroid = (size1*get<0>(data[label]) + size2*get<0>(data[target_label])) / (size1 + size2);

            data[target_label] = std::make_tuple(new_centroid, new_normal, size1+size2);

            // Merge children information
            children[target_label].push_back(label);
            children[target_label].insert(children[target_label].end(), children[label].begin(), children[label].end());

            // Delete merged region data
            survived.erase(label_iter);
            distance_pair.erase(label);

            // Update distance pair for new region and deleted region
            updateDistances(target_label, label);

            merged = true;
            break;
            }
            // Once merge occur, create new iteration
            if (merged) break;
        }
    }

    // Create merge relationship map to easily update original labels
    std::map<int, int> merge_map;

    for (auto children_iter=children.begin(); children_iter!=children.end(); children_iter++)
    {
        int parent_label = children_iter->first;
        if (!std::any_of(survived.begin(), survived.end(), [parent_label](const int& x) {return x==parent_label;})) continue;
        
        std::vector<int> child_label_list = children_iter->second;
        
        for (auto child_label_iter=child_label_list.begin(); child_label_iter!=child_label_list.end(); child_label_iter++)
        {
            merge_map[*child_label_iter] = parent_label;
        }
    }
    return merge_map;
}