#ifndef MAPGEN_H
#define MAPGEN_H

#include <erasor/node.h>
#include "tools/erasor_utils.hpp"
#include <math.h>

#define CAR_BODY_SIZE 2.7
#define NUM_MAP_PC_LARGE_ENOUGH 30000000

struct Cluster {
    double   x;
    double   y;
    double   z;
    int      num_pt = 0;
    uint32_t class_num;
};

const char separator    = ' ';
const int nameWidth     = 10;

double calc_dist(Cluster d0, Cluster d1) {
    return sqrt(pow(d0.x - d1.x, 2) + pow(d0.y - d1.y, 2) + pow(d0.z - d1.z, 2));
}

class mapgen {
private:
    bool                            is_initial  = true;
    int                             count       = 0;
    int                             accum_count = 0;
    int                             interval, last_ts, init_ts;
    pcl::PointCloud<pcl::PointXYZI> cloud_curr;
    pcl::PointCloud<pcl::PointXYZI> cloud_map;

    // For large-scale map building
    // ROS can publish point cloud whose volume is under the 1 GB
    std::vector<pcl::PointCloud<pcl::PointXYZI> > cloud_maps;

    nav_msgs::Path odom_path;
    float          leafsize;
    std::string    seq, init_stamp, final_stamp, save_path;

    bool is_large_scale;

    // --- check dynamic objects ---
    double                               movement_thr    = 1.0;
    std::map<int, std::vector<Cluster> > not_moving_object_candidates; // <id, Cluster>
    std::vector<int>                     dynamic_classes = {252, 253, 254, 255, 256, 257, 258, 259};
    std::map<int, int>                   dynamic_objects; // <id, class_num> These indicate the id of moving objects
    std::vector<int>                     all_dynamic_ids;
    std::vector<int>                     moving_dynamic_ids;

    // In summary, static: {not_moving_object_candidates} - {dynamic_objects}
    void check_movement(const pcl::PointCloud<pcl::PointXYZI> &cloud) {
        std::map<int, Cluster> clusters_tmp; // <id, Cluster>
        // 1. points to <id, centroid>
        for (const auto        &pt: cloud.points) {
            uint32_t float2int      = static_cast<uint32_t>(pt.intensity);
            uint32_t semantic_label = float2int & 0xFFFF;
            uint32_t inst_label     = float2int >> 16;

            for (int class_num: dynamic_classes) {
                if (semantic_label == class_num) { // 1. check it is in the moving object classes
                    if (!clusters_tmp.empty()) {
                        for (auto it = clusters_tmp.begin(); it != clusters_tmp.end(); it++) {
                            if (it->first == inst_label) { // already exist
                                it->second.x      = it->second.x + pt.x;
                                it->second.y      = it->second.y + pt.y;
                                it->second.z      = it->second.z + pt.z;
                                it->second.num_pt = it->second.num_pt + 1;
                            } else {
                                Cluster instance = {pt.x, pt.y, pt.z, 1, semantic_label};
                                clusters_tmp[inst_label] = instance;
                            }
                        }
                    } else { // Initialization
                        Cluster instance = {pt.x, pt.y, pt.z, 1, semantic_label};
                        clusters_tmp[inst_label] = instance;
                    }
                }
            }
        }
        // 2. Set points to centroid!
        for (auto              it = clusters_tmp.begin(); it != clusters_tmp.end(); it++) {
            it->second.x = it->second.x / static_cast<double>(it->second.num_pt);
            it->second.y = it->second.y / static_cast<double>(it->second.num_pt);
            it->second.z = it->second.z / static_cast<double>(it->second.num_pt);

            all_dynamic_ids.push_back(it->first);
            for (auto it_id = all_dynamic_ids.begin(); it_id != (all_dynamic_ids.end() - 1); it_id++) {
                if (it->first == *it_id) {
                    all_dynamic_ids.pop_back();
                    break;
                }
            }
        }
        // Debug session
        std::cout << "\033[1;32m=========[Dynamic Ids]===========" << std::endl;
        for (auto it = all_dynamic_ids.begin(); it != all_dynamic_ids.end(); it++) {
            std::cout << *it << ", ";
        }
        std::cout << "are dynamic objects!" << std::endl;
        std::cout << "=================================\033[0m" << std::endl;
        std::cout << "[Cluster size]: " << clusters_tmp.size() << std::endl;

        for (auto it = clusters_tmp.begin(); it != clusters_tmp.end(); it++) {
            std::cout << it->first << ", " << it->second.x << ", " << it->second.y << ", " << it->second.z << std::endl;
        }

        // 3. Update to not moving_object_candidates;
        if (not_moving_object_candidates.empty()) { // init
            for (auto it = clusters_tmp.begin(); it != clusters_tmp.end(); it++) {
                std::vector<Cluster> obj_trajectories = {it->second};
                not_moving_object_candidates.insert(std::make_pair(it->first, obj_trajectories));
            }
            return;
        }

        for (auto it_not_mv = not_moving_object_candidates.begin(); it_not_mv != not_moving_object_candidates.end(); it_not_mv++) {
            std::cout << it_not_mv->first << ", " << it_not_mv->second.size() << std::endl;
        }

        // In General cases
        for (auto        it        = clusters_tmp.begin(); it != clusters_tmp.end(); it++) {
            bool      is_in     = false;
            for (auto it_not_mv = not_moving_object_candidates.begin(); it_not_mv != not_moving_object_candidates.end(); it_not_mv++) {
                if (it->first == it_not_mv->first) { // push back is needed
                    it_not_mv->second.push_back(it->second);
                    is_in = true;
                }
            }
            if (!is_in) { // new object
                std::vector<Cluster> obj_trajectories = {it->second};
                not_moving_object_candidates.insert(std::make_pair(it->first, obj_trajectories));
            }
        }
        // 4. check movement
        for (auto        it_not_mv = not_moving_object_candidates.begin(); it_not_mv != not_moving_object_candidates.end(); it_not_mv++) {
            if (it_not_mv->second.size() > 1) {
                Cluster first_obj = it_not_mv->second.front();
                Cluster last_obj  = it_not_mv->second.back();
                double  movement  = calc_dist(first_obj, last_obj);
                if (movement_thr < movement) { // It's dynamic object!
                    dynamic_objects.insert(std::make_pair(it_not_mv->first, it_not_mv->second.front().class_num));
                    moving_dynamic_ids.push_back(it_not_mv->first);
                }
            }
        }
        std::vector<int> static_objects;
        for (auto        it_all    = all_dynamic_ids.begin(); it_all != all_dynamic_ids.end(); it_all++) {
            bool      moved = false;
            for (auto it    = moving_dynamic_ids.begin(); it != moving_dynamic_ids.end(); it++) {
                if (*it_all == *it) {
                    moved = true;
                }
            }
            if (!moved) {
                static_objects.push_back(*it_all);
            }

        }
        std::cout << "\033[2;32m========================" << std::endl;
        if (!static_objects.empty()) {
            for (auto id:static_objects) {
                std::cout << id << ", ";
            }
            std::cout << "are static!" << std::endl;
        }
        std::cout << "========================\033[0m" << std::endl;
    }

public:
    mapgen() {
        cloud_map.reserve(NUM_MAP_PC_LARGE_ENOUGH);
    }
    ~mapgen() {}

    void setValue(std::string pcd_save_path,float voxelsize, std::string sequence, std::string init_time_stamp, std::string final_time_stamp, int frame_interval, bool is_map_large_scale) {
        save_path   = pcd_save_path;
        leafsize    = voxelsize;
        seq         = sequence;
        init_stamp  = init_time_stamp;
        final_stamp = final_time_stamp;
        interval    = frame_interval;

        last_ts     = std::stoi(final_stamp);
        init_ts     = std::stoi(init_stamp);

        is_large_scale = is_map_large_scale;

        std::cout << "\033[1;32m";
        std::cout << "[MAPGEN]: Voxelization size - " << leafsize << std::endl;
        std::cout << "[MAPGEN]: Target seq -  " << seq << std::endl;
        std::cout << "[MAPGEN]: From " << init_stamp << ", " << final_stamp << std::endl;
        std::cout << "[MAPGEN]: Is the map large-scale? " << is_large_scale << "\033[0m" << std::endl;
    }

    void accumPointCloud(
            const erasor::node &data, nav_msgs::Path &path) {
        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.header          = data.header;
        pose_stamped.header.frame_id = "map";
        pose_stamped.pose            = data.odom;

        odom_path.header = pose_stamped.header;
        odom_path.poses.push_back(pose_stamped);
//    std::cout<<erasor_utilsgeoPose2eigen(data.odom)<<std::endl;
        path = odom_path;
        Eigen::Matrix4f tf_lidar2origin;

        tf_lidar2origin << 1, 0, 0, 0,
                0, 1, 0, 0,
                0, 0, 1, 1.73,
                0, 0, 0, 1;

        pcl::PointCloud<pcl::PointXYZI> cloud = erasor_utils::cloudmsg2cloud<pcl::PointXYZI>(data.lidar);

        // To remove some noisy points in the vicinity of the vehicles
        pcl::PointCloud<pcl::PointXYZI> inliers, outliers; // w.r.t origin
        float                           max_dist_square = pow(CAR_BODY_SIZE, 2);
        for (auto const                 &pt : cloud.points) {
            double dist_square = pow(pt.x, 2) + pow(pt.y, 2);
            if (dist_square < max_dist_square) {
                inliers.push_back(pt);
            } else {
                outliers.push_back(pt);
            }
        }
        cloud = outliers;

        pcl::PointCloud<pcl::PointXYZI>::Ptr ptr_transformed(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::transformPointCloud(cloud, *ptr_transformed, tf_lidar2origin);

        Eigen::Matrix4f pose = erasor_utils::geoPose2eigen(data.odom);
        std::cout << std::setprecision(3) << std::left << setw(nameWidth) << setfill(separator) << "=> [Pose] " << pose(0, 3) << ", " << pose(1, 3) << ", " << pose(2, 3) << std::endl;
        pcl::PointCloud<pcl::PointXYZI>::Ptr world_transformed(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::transformPointCloud(*ptr_transformed, *world_transformed, pose);

        erasor_utils::voxelize_preserving_labels(world_transformed, cloud_curr, 0.2);

        if (is_initial) {
            cloud_map  = cloud_curr;
            is_initial = false;
        } else {
            cloud_map += cloud_curr;

            if (is_large_scale) {
                static int cnt_voxel = 0;
                if (cnt_voxel++ % 500 == 0){
                    pcl::PointCloud<pcl::PointXYZI>::Ptr ptr_map(new pcl::PointCloud<pcl::PointXYZI>);
                    *ptr_map = cloud_map;
                    std::cout << "\033[1;32m Voxelizing submap...\033[0m" << std::endl;
                    erasor_utils::voxelize_preserving_labels(ptr_map, cloud_map, leafsize);

                    cloud_maps.push_back(cloud_map);
                    cloud_map.clear();
                }

            }
            ++accum_count;
        }

    }
    void getPointClouds(pcl::PointCloud<pcl::PointXYZI>::Ptr map_out,
                        pcl::PointCloud<pcl::PointXYZI>::Ptr curr_out){
        *map_out  = cloud_map;
        *curr_out = cloud_curr;
    }

    void saveNaiveMap(const std::string& original_dir, const std::string& map_dir){
        pcl::PointCloud<pcl::PointXYZI> cloud_src;

        std::cout << "\033[1;32m On saving map cloud...it may take few seconds...\033[0m" << std::endl;
        if (is_large_scale){
            // Prvious submaps
            for (const auto & submap: cloud_maps){
                cloud_src += submap;
            }
            // Remain map
            cloud_src += cloud_map;
        }else{
            cloud_src = cloud_map;
        }

        pcl::io::savePCDFileASCII(original_dir, cloud_src);

        pcl::PointCloud<pcl::PointXYZI>::Ptr ptr_map(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PointCloud<pcl::PointXYZI> cloud_out;
        ptr_map->points.reserve(cloud_src.points.size());

        std::cout << "\033[1;32m Start to copy pts...\033[0m" << std::endl;
        *ptr_map = cloud_src;
        std::cout << "[Debug]: " << cloud_src.width << ", " << cloud_src.height << ", " << cloud_src.points.size() << std::endl;
        std::cout << "\033[1;32m On voxelizing...\033[0m" << std::endl;
        erasor_utils::voxelize_preserving_labels(ptr_map, cloud_out, leafsize);

        cloud_out.width  = cloud_out.points.size();
        cloud_out.height = 1;
        std::cout << "[Debug]: " << cloud_out.width << ", " << cloud_out.height << ", " << cloud_out.points.size() << std::endl;
        std::cout << "\033[1;32m Saving the map to pcd...\033[0m" << std::endl;
        pcl::io::savePCDFileASCII(map_dir, cloud_out);
        std::cout << "\033[1;32m Complete to save the map!:";
        std::cout << map_dir << "\033[0m" << std::endl;

    }
};

#endif

