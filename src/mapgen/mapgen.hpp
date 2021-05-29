#ifndef MAPGEN_H
#define MAPGEN_H

#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/extract_indices.h>
#include <unavlib/convt.h>
#include <erasor/node.h>
#include <string>
#include <iostream>
#include <fstream>
#include <vector>
#include <map>
#include <math.h>
using namespace unavlib;

struct Cluster{
  double x;
  double y;
  double z;
  int num_pt=0;
  uint32_t class_num;
};

double calc_dist(Cluster d0, Cluster d1){
  return sqrt(pow(d0.x-d1.x, 2) + pow(d0.y-d1.y, 2) + pow(d0.z-d1.z, 2));
}

class mapgen
{
private:
  bool is_initial = true;
  int count = 0;
  int accum_count = 0;
  int interval, last_ts, init_ts;
  pcl::PointCloud<pcl::PointXYZI> cloud_curr;
  pcl::PointCloud<pcl::PointXYZI> cloud_map;

  pcl::PointCloud<pcl::PointXYZ> dynamic_objects_but_not_moving;

  nav_msgs::Path odom_path;
  float leafsize;
  std::string seq, init_stamp, final_stamp;

  // --- check dynamic objects ---
  double movement_thr = 1.0;
  std::map<int, std::vector<Cluster> > not_moving_object_candidates; // <id, Cluster>
  std::vector<int> dynamic_classes = {252, 253, 254, 255, 256, 257, 258, 259};
  std::map<int, int> dynamic_objects; // <id, class_num> These indicate the id of moving objects
  std::vector<int> all_dynamic_ids;
  std::vector<int> moving_dynamic_ids;
  // In summary, static: {not_moving_object_candidates} - {dynamic_objects}
  void check_movement(const pcl::PointCloud<pcl::PointXYZI>& cloud){
    // Deprecated.
    std::map<int, Cluster> clusters_tmp; // <id, Cluster>
    // 1. points to <id, centroid>
    for (const auto &pt: cloud.points){
      uint32_t float2int = static_cast<uint32_t>(pt.intensity);
      uint32_t semantic_label = float2int & 0xFFFF;
      uint32_t inst_label = float2int >> 16;

      for (int class_num: dynamic_classes){
        if (semantic_label == class_num){ // 1. check it is in the moving object classes
          if (!clusters_tmp.empty()){
            for(auto it = clusters_tmp.begin(); it != clusters_tmp.end(); it++){
              if (it->first == inst_label){ // already exist
                it->second.x = it->second.x + pt.x;
                it->second.y = it->second.y + pt.y;
                it->second.z = it->second.z + pt.z;
                it->second.num_pt = it->second.num_pt + 1;
              }else{
                Cluster instance = {pt.x, pt.y, pt.z, 1, semantic_label};
                clusters_tmp[inst_label] = instance;
              }
            }
          }else{ // Initialization
            Cluster instance = {pt.x, pt.y, pt.z, 1, semantic_label};
            clusters_tmp[inst_label] = instance;
          }
        }
      }
    }
    // 2. Set points to centroid!
    for (auto it = clusters_tmp.begin(); it != clusters_tmp.end(); it++){
        it->second.x = it->second.x / static_cast<double>(it->second.num_pt);
        it->second.y = it->second.y / static_cast<double>(it->second.num_pt);
        it->second.z = it->second.z / static_cast<double>(it->second.num_pt);

        all_dynamic_ids.push_back(it->first);
        for (auto it_id = all_dynamic_ids.begin(); it_id != (all_dynamic_ids.end()-1); it_id++){
          if (it->first == *it_id){
            all_dynamic_ids.pop_back();
            break;
          }
        }
    }
    // Debug session
    std::cout<<"\033[1;32m=========[Dynamic Ids]==========="<<std::endl;
    for (auto it = all_dynamic_ids.begin(); it != all_dynamic_ids.end(); it++){
      std::cout<<*it<<", ";
    }
    std::cout<<"are dynamic objects!"<<std::endl;
    std::cout<<"=================================\033[0m"<<std::endl;
    std::cout<<"[Cluster size]: "<<clusters_tmp.size()<<std::endl;

    for (auto it = clusters_tmp.begin(); it != clusters_tmp.end(); it++){
      std::cout<<it->first<<", "<<it->second.x<<", "<<it->second.y<<", "<<it->second.z<<std::endl;
    }

    // 3. Update to not moving_object_candidates;
    if (not_moving_object_candidates.empty()){ // init
      for (auto it = clusters_tmp.begin(); it != clusters_tmp.end(); it++){
        std::vector<Cluster> obj_trajectories = {it->second};
        not_moving_object_candidates.insert(std::make_pair(it->first, obj_trajectories));
      }
      return;
    }

    for (auto it_not_mv = not_moving_object_candidates.begin(); it_not_mv != not_moving_object_candidates.end(); it_not_mv++){
      std::cout<<it_not_mv->first<<", "<<it_not_mv->second.size()<<std::endl;
    }

    // In General cases
    for (auto it = clusters_tmp.begin(); it != clusters_tmp.end(); it++){
      bool is_in = false;
      for (auto it_not_mv = not_moving_object_candidates.begin(); it_not_mv != not_moving_object_candidates.end(); it_not_mv++){
        if (it->first == it_not_mv->first){ // push back is needed
          it_not_mv->second.push_back(it->second);
          is_in = true;
        }
      }
      if (!is_in){ // new object
        std::vector<Cluster> obj_trajectories = {it->second};
        not_moving_object_candidates.insert(std::make_pair(it->first, obj_trajectories));
      }
    }
    // 4. check movement
    for (auto it_not_mv = not_moving_object_candidates.begin(); it_not_mv != not_moving_object_candidates.end(); it_not_mv++){
      if (it_not_mv->second.size() > 1){
        Cluster first_obj = it_not_mv->second.front();
        Cluster last_obj = it_not_mv->second.back();
        double movement = calc_dist(first_obj, last_obj);
        if (movement_thr < movement){ // It's dynamic object!
          dynamic_objects.insert(std::make_pair(it_not_mv->first, it_not_mv->second.front().class_num));
          moving_dynamic_ids.push_back(it_not_mv->first);
        }
      }
    }
    std::vector<int> static_objects;
    for (auto it_all = all_dynamic_ids.begin(); it_all != all_dynamic_ids.end(); it_all++){
      bool moved = false;
      for (auto it = moving_dynamic_ids.begin(); it != moving_dynamic_ids.end(); it++){
        if (*it_all == *it){
          moved = true;
        }
      }
      if (!moved){
        static_objects.push_back(*it_all);
      }

    }
    std::cout<<"\033[2;32m========================"<<std::endl;
    if (!static_objects.empty()){
      for (auto id:static_objects){
        std::cout<<id<<", ";
      }
      std::cout<<"are static!"<<std::endl;
    }
    std::cout<<"========================\033[0m"<<std::endl;

  }

void erasor_utils::voxelize_preserving_labels(pcl::PointCloud<pcl::PointXYZI>::Ptr src,
                          pcl::PointCloud<pcl::PointXYZI>& dst,
                          double leaf_size){
  pcl::PointCloud<pcl::PointXYZI> tmp, output;

  // 1. Voxelization
  static pcl::VoxelGrid<pcl::PointXYZI> voxel_filter;
  pcl::PointCloud<pcl::PointXYZI>::Ptr ptr_voxelized(new pcl::PointCloud<pcl::PointXYZI>);
  voxel_filter.setInputCloud(src);
  voxel_filter.setLeafSize(leaf_size, leaf_size, leaf_size);
  voxel_filter.filter(*ptr_voxelized);
  tmp = *ptr_voxelized;

  // 2. Find nearest point to update intensity(index and id)
  pcl::KdTreeFLANN<pcl::PointXYZI> kdtree;
  kdtree.setInputCloud (src);

  int K = 1;
  std::vector<int> pointIdxNKNSearch(K);
  std::vector<float> pointNKNSquaredDistance(K);
  // Set dst <- output
  for (const auto &pt: tmp.points){
    if ( kdtree.nearestKSearch (pt, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 )
    {
          auto updated = pt;
          // Update meaned intensity to original intensity
          updated.intensity = (*src)[pointIdxNKNSearch[0]].intensity;
          output.points.push_back(updated);
     }
  }
  dst = output;
}
  // -----------------------------

  void  filterPtcl(pcl::PointCloud<pcl::PointXYZ>::Ptr ptcl,float min,float max){
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    pcl::PointXYZ searchPoint;
    searchPoint.x = searchPoint.y = searchPoint.z = 0;
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
    std::vector<int> idxes;
    std::vector<float> dists;

    kdtree.setInputCloud (ptcl);
    kdtree.radiusSearch(searchPoint, max, idxes, dists);
    extract.setInputCloud (ptcl);
    inliers->indices = idxes;
    extract.setIndices (inliers);
    extract.filter (*ptcl);

    idxes.clear();
    dists.clear();

    kdtree.setInputCloud (ptcl);
    kdtree.radiusSearch(searchPoint, min, idxes, dists);
    extract.setInputCloud (ptcl);
    inliers->indices = idxes;
    extract.setIndices (inliers);
    extract.setNegative (true);
    extract.filter (*ptcl);
  }

public:
  mapgen(){
    cloud_map.reserve(500000000);
  }
  ~mapgen(){}
  void setValue(float voxelsize, std::string sequence, std::string init_time_stamp, std::string final_time_stamp, int frame_interval)
  {
    leafsize = voxelsize;
    seq = sequence;
    init_stamp = init_time_stamp;
    final_stamp = final_time_stamp;
    interval = frame_interval;
    last_ts = std::stoi(final_stamp);
    init_ts = std::stoi(init_stamp);

    std::cout<<leafsize<<std::endl;
    std::cout<<seq<< ", "<<init_stamp<<", "<<final_stamp<<std::endl;
  }

  void check_dynamicity(const pcl::PointCloud<pcl::PointXYZI>& cloud){
    std::cout<<"hey"<<std::endl;
  }



  void putData(const erasor::node & data, pcl::PointCloud<pcl::PointXYZI>::Ptr mapOut,
               pcl::PointCloud<pcl::PointXYZI>::Ptr currOut, nav_msgs::Path & path)
  {
    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header = data.header;
    pose_stamped.header.frame_id = "/map";
    pose_stamped.pose = data.odom;

    odom_path.header = pose_stamped.header;
    odom_path.poses.push_back(pose_stamped);
//    std::cout<<erasor_utilsgeoPose2eigen(data.odom)<<std::endl;
    path = odom_path;
    Eigen::Matrix4f tf_lidar2origin;

    tf_lidar2origin << 1, 0, 0,  0,
                       0, 1, 0,  0,
                       0, 0, 1, 1.73,
                       0, 0, 0,  1;

    pcl::PointCloud<pcl::PointXYZI> cloud = erasor_utilscloudmsg2cloud<pcl::PointXYZI>(data.lidar);
    // ------------ Debugging process -------------
//    std::string filename = "/home/shapelim/hdd2/kitti_semantic/dataset/debug_cpp/" + std::to_string(count) + ".csv";
//    std::ofstream fs(filename);
//    count++;
//    std::cout<<data.lidar.data.size()<<" vs "<<cloud.points.size()<<std::endl;
//    for (auto const &pt_tmp: cloud.points){
//      uint32_t float2int = static_cast<uint32_t>(pt_tmp.intensity);

//      uint32_t semantic_label = float2int & 0xFFFF;
//      uint32_t inst_label = float2int >> 16;
//      fs << semantic_label << ","<<inst_label <<std::endl;
//    }
//    fs.close();
    // --------------------------------------------

    pcl::PointCloud<pcl::PointXYZI> inliers, outliers; // w.r.t origin
    float CAR_BODY_SIZE = 2.7;
    float max_dist_square = pow(CAR_BODY_SIZE, 2);
    for (auto const &pt : cloud.points){
      double dist_square = pow(pt.x, 2) + pow(pt.y, 2);
      if (dist_square < max_dist_square){
          inliers.push_back(pt);
      }else{
          outliers.push_back(pt);
      }
    }
    cloud = outliers;

    pcl::PointCloud<pcl::PointXYZI>::Ptr ptr_transformed(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::transformPointCloud(cloud, *ptr_transformed, tf_lidar2origin);

    Eigen::Matrix4f pose = erasor_utilsgeoPose2eigen(data.odom);
    std::cout<<"[Current Pose]: "<<pose(0, 3)<< ", "<< ", "<<pose(1, 3)<<", "<<pose(2, 3)<<std::endl;
    pcl::PointCloud<pcl::PointXYZI>::Ptr world_transformed(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::transformPointCloud(*ptr_transformed, *world_transformed, pose);

    erasor_utils::voxelize_preserving_labels(world_transformed, cloud_curr, 0.2);

//    check_movement(cloud_curr);

    if (is_initial){
      cloud_map = cloud_curr;
       is_initial = false;
    }else{
      cloud_map += cloud_curr;

      if (data.header.seq == last_ts){
        // Original
        std::string original_dir = "/media/shapelim/UX960NVMe1/kitti_semantic/semantic_KITTI_map/src_0_2/seq05/" + seq + "_" +init_stamp+ "_to_" + final_stamp + "_w_interval" + std::to_string(interval) + "_voxel_"+ std::to_string(leafsize)+"_original.pcd";
        pcl::io::savePCDFileASCII(original_dir, cloud_map);

        pcl::PointCloud<pcl::PointXYZI>::Ptr ptr_map(new pcl::PointCloud<pcl::PointXYZI>);
        *ptr_map = cloud_map;
        std::cout<<"\033[1;32m On voxelizing...\033[0m"<<std::endl;
        erasor_utils::voxelize_preserving_labels(ptr_map, cloud_map, leafsize);
        std::string map_dir = "/media/shapelim/UX960NVMe1/kitti_semantic/semantic_KITTI_map/src_0_2/seq05/" + seq + "_" +init_stamp+ "_to_" + final_stamp + "_w_interval" + std::to_string(interval) + "_voxel_"+ std::to_string(leafsize)+".pcd";
        cloud_map.width = cloud_map.points.size();
        cloud_map.height = 1;
        std::cout<<"[Debug]: "<<cloud_map.width<<", "<<cloud_map.height <<", "<<cloud_map.points.size()<<std::endl;
        std::cout<<"\033[1;32m Saving the map to pcd...\033[0m"<<std::endl;
        pcl::io::savePCDFileASCII(map_dir, cloud_map);
        std::cout<<" = = = = = = = = = = = = ="<<std::endl;
        std::cout<<"\033[1;32m Complete to save the map!\033[0m"<<std::endl;
        std::cout<<data.header.seq<<std::endl;
        std::cout<<" = = = = = = = = = = = = ="<<std::endl;
      }
      ++accum_count;
    }

    *mapOut = cloud_map;
    *currOut = cloud_curr;


  }
};

#endif

