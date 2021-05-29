#include <iostream>
#include <vector>
#include <cstdlib>
#include <ctime>
#include <algorithm>
#include <string>
#include <fstream>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/ndt.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/crop_box.h>

using namespace std;

float leaf_size = 2.0;

template<typename T>
void print_cloud(pcl::PointCloud<T> src){
  if (src.size() > 0){
    for (const auto &p: src.points){
      cout<< p.x <<", "<< p.y<<", "<<p.z<<endl;
      cout<< static_cast<int>(p.x / leaf_size) << ", " << static_cast<int>(p.y / leaf_size) << endl;
    }
  }
}
bool point_cmp(double a, double b){
    return a < b;
}

int main(){
  pcl::PointCloud<pcl::PointXYZI> cloud, output;

  static pcl::VoxelGrid<pcl::PointXYZI> voxel_filter;
  pcl::PointCloud<pcl::PointXYZI>::Ptr ptr_voxelized(new pcl::PointCloud<pcl::PointXYZI>);

  pcl::PointXYZI point;
  point.x = 1.1;
  point.y = 0.4;
  point.z = 1;
  point.intensity = 0;
  cloud.push_back(point);

  point.x = 1.5;
  point.y = 2.1;
  point.z = 1;
  point.intensity = 1000;
  cloud.push_back(point);

  point.x = 1.9;
  point.y = 1.9;
  point.z = 1;
  point.intensity = 100;
  cloud.push_back(point);

  *ptr_voxelized = cloud;
  voxel_filter.setInputCloud(ptr_voxelized);
  voxel_filter.setLeafSize(leaf_size, leaf_size, leaf_size);
  voxel_filter.filter(*ptr_voxelized);
  output = *ptr_voxelized;
  print_cloud(output);

  cloud.points.clear();

  point.x = 1.2;
  point.y = 0.75;
  point.z = 1;
  point.intensity = 0;
  cloud.push_back(point);

  point.x = 1.5;
  point.y = 2.5;
  point.z = 1;
  point.intensity = 1000;
  cloud.push_back(point);

  point.x = 1.1;
  point.y = 0.9;
  point.z = 1;
  point.intensity = 100;
  cloud.push_back(point);

  *ptr_voxelized = cloud;
  voxel_filter.setInputCloud(ptr_voxelized);
  voxel_filter.setLeafSize(leaf_size, leaf_size, leaf_size);
  voxel_filter.filter(*ptr_voxelized);
  output = *ptr_voxelized;
  print_cloud(output);

  std::vector<float> h = {0.3, 0.5, -0.1};
  std::sort(h.begin(), h.end(), point_cmp);
  std::cout<<h.at(0) <<" , "<< h.at(2)<<std::endl;

}
//  pcl::PointCloud<pcl::PointXYZ>::Ptr ptr_filtered(new pcl::PointCloud<pcl::PointXYZ>);
//  pcl::PassThrough<pcl::PointXYZ> ptfilter(true);
//  pcl::PassThrough<pcl::PointXYZ> ptfilter2(true);

//  double a = 10.20;
//  int div = 3;
//  auto c = (a / div);
//  cout<<c<<endl;
//  vector<int> test;
//  test.push_back(2);
//  test.push_back(2);
//  test[1] = 10;
//  cout<<test[1]<<endl;
//  auto d = static_cast<int>(a / div);
//  cout<<d<<endl;

//    *ptr_filtered = cloud2;
//    ptfilter.setInputCloud(ptr_filtered);
//    ptfilter.setFilterFieldName("x");
//    ptfilter.setFilterLimits(-0.2, 0.8);
//    ptfilter.filter(*ptr_filtered);
//    auto indices_x = ptfilter.getRemovedIndices();

//    cloud_middle = *ptr_filtered;
//    cout<<ptr_filtered->size()<<endl;
//    cout<<"====cloud_middle===="<<endl;
//    print_cloud(cloud_middle);
//    cout<<"====cloud_middle===="<<endl;
//    cout<<"Idx x: "<<indices_x->size()<<endl;
//    cout<<indices_x->at(0)<<endl;
//    cout<<indices_x->at(1)<<endl;


//    ptfilter2.setInputCloud(ptr_filtered);
//    ptfilter2.setFilterFieldName("y");
//    ptfilter2.setFilterLimits(-0.2, 0.8);
//    ptfilter2.filter(*ptr_filtered);
//    pcl::IndicesConstPtr indices_y = ptfilter2.getRemovedIndices();

//    cloud_out = *ptr_filtered;

//    cout<<"====cloud_out===="<<endl;
//    print_cloud(cloud_out);
//    cout<<"====cloud_out===="<<endl;
//    cout<<"Idx y: "<<indices_y->size()<<endl;
//    cout<<indices_y->at(0)<<endl;


//    static pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
//    pcl::PointCloud<pcl::PointXYZ>::Ptr ptr_voxelized(new pcl::PointCloud<pcl::PointXYZ>);
//    *ptr_filtered = cloud2;
//    voxel_filter.setInputCloud(ptr_filtered);
//    voxel_filter.setLeafSize(2, 2, 1);
//    voxel_filter.filter(*ptr_voxelized);
//    cloud2 = *ptr_voxelized;
//    print_cloud(cloud2);





//  pcl::PointCloud<pcl::PointXYZ>::Ptr ptr_filtered(new pcl::PointCloud<pcl::PointXYZ>);
//  ptr_filtered->points.push_back(cloud2.at(1));
//  ptr_filtered->points.at(0).x = 10;

//  cout<< ptr_filtered->points.at(0).x <<endl;
//  cout<< ptr_filtered->points.at(0).y <<endl;
//  cout<< ptr_filtered->points.at(0).z <<endl;

//  cout<<cloud2.at(1).x <<endl;
//  cout<<cloud2.at(1).y <<endl;
//  cout<<cloud2.at(1).z <<endl;

//  pcl::PointXYZ test;
//  pcl::PointXYZ numerator(3, 3, 3);
//  test.getArray3fMap() = cloud2.at(0).getArray3fMap() + cloud2.at(1).getArray3fMap();
//  test.getArray3fMap() = test.getArray3fMap() / numerator.getArray3fMap();

//  cout<<test.x <<endl;
//  cout<<test.y <<endl;
//  cout<<test.z <<endl;





