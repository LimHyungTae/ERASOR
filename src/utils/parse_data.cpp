#include <iostream>
#include <vector>
#include <cstdlib>
#include <ctime>
#include <algorithm>
#include <string>
#include <sstream>
#include <fstream>
#include <unavlib/convt.h>
#include <unavlib/others.h>
#include <sensor_msgs/PointCloud2.h>
#include <time.h>

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
#include <map_updater/src/scdr.h>
#include <termios.h>
//#include <src/scdr.h>
#define LINE_SPACE 10

using namespace std;
using namespace unavlib;

string root = "/media/shapelim/SAMSUNG/A4_XYZI";
string mapname = "/map/";
string pc_currname = "/curr/";
string posename = "/pose/";
string type_pcd = ".pcd";
string type_bin = ".bin";

double max_d;

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

void pass_through(const pcl::PointCloud<pcl::PointXYZI>& src,
                  pcl::PointCloud<pcl::PointXYZI>& dst){
    pcl::PointCloud<pcl::PointXYZI>::Ptr ptr_filtered(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PassThrough<pcl::PointXYZI> ptfilter;

    *ptr_filtered = src;

    ptfilter.setInputCloud(ptr_filtered);
    ptfilter.setFilterFieldName("x");
    ptfilter.setFilterLimits(-max_d, max_d);
    ptfilter.filter(*ptr_filtered);

    ptfilter.setInputCloud(ptr_filtered);
    ptfilter.setFilterFieldName("y");
    ptfilter.setFilterLimits(-max_d, max_d);
    ptfilter.filter(*ptr_filtered);

    ptfilter.setInputCloud(ptr_filtered);
    ptfilter.setFilterFieldName("z");
    ptfilter.setFilterLimits(-0.2, 2.0);
    ptfilter.filter(*ptr_filtered);

    dst = *ptr_filtered;
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

void transform(Eigen::Matrix4f& pose, const pcl::PointCloud<pcl::PointXYZI>& src, pcl::PointCloud<pcl::PointXYZI>& dst){
  pcl::PointCloud<pcl::PointXYZI>::Ptr ptr_transformed(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::transformPointCloud(src, *ptr_transformed, pose);
  dst = *ptr_transformed;
}

static void Display_Init_Menu()
{
  std::cout<<"----------- < Init menu > ----------"<<std::endl;
  std::cout<<"[r] Re-publish"<<std::endl;
  std::cout<<"[n] Next"<<std::endl;
  std::cout<<"[c] Kill the process"<<std::endl;
  std::cout<<"----------------------------------------"<<std::endl;

}

void set_dirs(const string& filename, string& map_dir, string& pc_dir, string& pose_dir){
  map_dir = root + mapname + filename + type_pcd;
  pc_dir = root + pc_currname + filename + type_pcd;
  pose_dir = root + posename + filename + type_bin;

}

int main(int argc, char **argv){

//  ifstream fin;

//  fin.open("/media/shapelim/UX960 NVMe/kitti_semantic/dataset/sequences/00/labels/000000.label", ios::binary);

//  char buf[100000000];
//  string s;

//  //seekg()를 이용하여 파일의 마지막으로 포인터를 옮긴다.
//  fin.seekg(0, ios::end);

//  //tellg()를 이용하여 파일의 사이즈를 구한다.
//  int sz = fin.tellg();
//  std::cout<<sz<<std::endl;

//  //seekg()를 이용하여 다시 파일의 처음으로 포인터를 옮긴다.
//  fin.seekg(0, ios::beg);

//  //binary로 파일을 읽을 때는 read함수로 읽는다.

//  unsigned int f = 0;
//  fin.read((char *)&f, sizeof(int));
//  cout<<f<<endl;

  /* Test2 */
  std::ifstream input("/media/shapelim/UX960 NVMe/kitti_semantic/dataset/sequences/00/labels/000000.label", std::ios::binary );
  // copies all data into buffer
  std::vector<uint32_t> buffer(std::istreambuf_iterator<char>(input), {});
  std::cout<<buffer.size()<<std::endl;

    std::ifstream input2("/media/shapelim/UX960 NVMe/kitti_semantic/dataset/sequences/00/veoldyne/000000.bin", std::ios::binary );
  // copies all data into buffer
  std::vector<float> buffer2(std::istreambuf_iterator<char32_t>(input2), {});
  std::cout<<buffer2.size()<<std::endl;

  return 0;
}


