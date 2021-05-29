#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <iostream>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/ndt.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/common/transforms.h>
#include <vector>
#include <ctime>
#include <time.h>
using namespace std;

string root = "/media/shapelim/SAMSUNG/A4_XYZI";
string mapname = "/map/";
string pc_currname = "/curr/";
string posename = "/pose/";
string type_pcd = ".pcd";
string type_bin = ".bin";

Eigen::Matrix4f load_pose(string bin_name){
    std::ifstream in(bin_name,std::ios_base::binary);
    Eigen::Matrix4f pose_se3;
    std::cout << "Loaded pose from " <<bin_name<< std::endl;
    float f = 0;
    if(in.good())
    {
      for (int i = 0; i < 4; ++i){
        for (int j = 0; j < 3; ++j){
            in.read((char *)&f,sizeof(float));
            pose_se3(j, i) = f;
        }
      }
    }
    pose_se3(3, 0) = 0;
    pose_se3(3, 1) = 0;
    pose_se3(3, 2) = 0;
    pose_se3(3, 3) = 1;
    return pose_se3;
}

int load_pc(string pcd_name, pcl::PointCloud<pcl::PointXYZI>& dst){
  pcl::PointCloud<pcl::PointXYZI>::Ptr ptr_pc(new pcl::PointCloud<pcl::PointXYZI>);
  if (pcl::io::loadPCDFile<pcl::PointXYZI> (pcd_name, *ptr_pc) == -1)
  {
    PCL_ERROR ("Couldn't read file!!! \n");
    return (-1);
  }
  std::cout << "Loaded " << ptr_pc->size () << " data points from " <<pcd_name<< std::endl;
  dst = *ptr_pc;
  return 0;
}

main (int argc, char** argv)
{
  srand (time (NULL));

  pcl::PointCloud<pcl::PointXYZI> src;
  clock_t start, start2, end, end2;
  string map_dir, pose_dir;
  string filename = std::to_string(450);
  map_dir = root + mapname + filename + type_pcd;
  pose_dir = root + posename + filename + type_bin;

  Eigen::Matrix4f pose_curr = load_pose(pose_dir);
  load_pc(map_dir, src);

  cout<<"Num: "<<src.size()<<endl;

  // Generate pointcloud data
  start = clock();
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>);
  *cloud = src;
  end = clock();
  cout<<"Setting? "<<(static_cast<double>(end - start))/CLOCKS_PER_SEC <<std::endl;

  // Setting criteria point
  pcl::PointXYZI searchPoint;
  searchPoint.x = pose_curr(0, 3);
  searchPoint.y = pose_curr(1, 3);
  searchPoint.z = pose_curr(2, 3);

  float radius = 80.0;

  std::cout << "Neighbors within radius search at (" << searchPoint.x
            << " " << searchPoint.y
            << " " << searchPoint.z
            << ") with radius=" << radius << std::endl;

  start2 = clock();


  // Neighbors within radius search

  end2 = clock();
  cout<<"Takes "<<(static_cast<double>(end2 - start2))/CLOCKS_PER_SEC <<std::endl;

  return 0;
}
