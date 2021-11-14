#include <ros/ros.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/io/pcd_io.h>
#include <unavlib/convt.h>
#include <unavlib/others.h>
#include <string>
#include <map>
#include <vector>

using namespace unavlib;

std::vector<int> dynamic_classes = {252, 253, 254, 255, 256, 257, 258, 259};


void label_map(pcl::PointCloud<pcl::PointXYZI>::Ptr src,
               const pcl::PointCloud<pcl::PointXYZI>::Ptr medium,
                          pcl::PointCloud<pcl::PointXYZI>& dst,
                          double leaf_size){
  pcl::PointCloud<pcl::PointXYZI> tmp, output;
  int origin_size = (*src).points.size();
  // 1. Voxelization
  static pcl::VoxelGrid<pcl::PointXYZI> voxel_filter;
  pcl::PointCloud<pcl::PointXYZI>::Ptr ptr_voxelized(new pcl::PointCloud<pcl::PointXYZI>);
  voxel_filter.setInputCloud(src);
  voxel_filter.setLeafSize(leaf_size, leaf_size, leaf_size);
  voxel_filter.filter(*ptr_voxelized);
  tmp = *ptr_voxelized;

  std::cout<<"\033[1;32m"<<origin_size<<" - > "<<tmp.points.size()<<"\033[0m"<<std::endl;

  // 2. Find nearest point to update intensity(index and id)

  pcl::KdTreeFLANN<pcl::PointXYZI> kdtree;
  kdtree.setInputCloud (medium);

  int K = 1;
  std::vector<int> pointIdxNKNSearch(K);
  std::vector<float> pointNKNSquaredDistance(K);
  // Set dst <- output
  for (const auto &pt: tmp.points){
    if ( kdtree.nearestKSearch (pt, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 )
    {
          auto updated = pt;
          // Update meaned intensity to original intensity
          updated.intensity = (*medium)[pointIdxNKNSearch[0]].intensity;
          output.points.push_back(updated);
     }
  }
  dst = output;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "removert_update");
    std::cout<<"KiTTI MAPVIZ STARTED"<<std::endl;
    ros::NodeHandle nodeHandler;

    std::string mode, removertName, srcName, denseSrcName, seq;


    nodeHandler.param<std::string>("/removert", removertName, "/media/shapelim");
    nodeHandler.param<std::string>("/DenseOriginal", denseSrcName, "/media/shapelim");
    nodeHandler.param<std::string>("/seq", seq, "00");
    nodeHandler.param<std::string>("/mode", mode, "rm3");

    ////////////////////////////////////////////////////////////////////

    std::cout<<"Loading map..."<<std::endl;
    std::cout<<removertName<<std::endl;
    std::cout<<denseSrcName<<std::endl;
    // load dense map
    std::cout<<"\033[1;32mLoading dense map...\033[0m"<<std::endl;
    pcl::PointCloud<pcl::PointXYZI>::Ptr ptr_densemap(new pcl::PointCloud<pcl::PointXYZI>);
    datahandle3d::load_pcd(denseSrcName, ptr_densemap);
    pcl::PointCloud<pcl::PointXYZI> srcDense;

    std::cout<<"Setting removert pcd..."<<std::endl;
    // load removert
    pcl::PointCloud<pcl::PointXYZI>::Ptr ptr_removert(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr ptr_transformed(new pcl::PointCloud<pcl::PointXYZI>);
    datahandle3d::load_pcd(removertName, ptr_removert);
    Eigen::Matrix4f tf;
    // Only needed for removert!!!
//    tf << 0,  0, 1, 0,
//         -1,  0, 0, 0,
//          0, -1, 0, 1.73,
//          0,  0, 0, 1;
//    pcl::transformPointCloud(*ptr_removert, *ptr_transformed, tf);

    pcl::PointCloud<pcl::PointXYZI> removert_output;
    std::cout<<"Labeling removert pcd..."<<std::endl;
    label_map(ptr_removert, ptr_densemap, removert_output, 0.2);
    std::cout<<"Saving removert pcd..."<<std::endl;
    removert_output.width = removert_output.points.size();
    removert_output.height = 1;
    std::string tmp = removertName;
    tmp.erase(tmp.end()-4, tmp.end());
    std::string save_name = tmp + "_w_label.pcd";
    pcl::io::savePCDFileASCII(save_name, removert_output);
    std::cout<<"\033[1;32mComplete to save!!!\033[0m"<<std::endl;

    return 0;
}
