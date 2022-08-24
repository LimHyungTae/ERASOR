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
  ros::init(argc, argv, "main");
  ros::NodeHandle nh = ros::NodeHandle("~");
  int init_idx, num_rings, num_sectors;
  double max_range, min_h, max_h;
  nh.param("/idx", init_idx, 300);
  nh.param("/scdr/max_range", max_range, 78.0);
  nh.param("/scdr/num_rings", num_rings, 20);
  nh.param("/scdr/num_sectors", num_sectors, 100);
  nh.param("/scdr/max_h", max_h, 3.0);
  nh.param("/scdr/min_h", min_h, 0.0);
  cout<<"===========Setting params...============"<<endl;
  cout<<"target idx: "<< init_idx<<endl;
  cout<<"max_range "<< max_range<<endl;
  cout<<"num_rings "<< num_rings<<endl;
  cout<<"num_sectors "<< num_sectors<<endl;
  cout<<"max_h "<< max_h<<endl;
  cout<<"max_h "<< min_h<<endl;
  cout<<"========================================"<<endl;
  max_d = max_range;

  ros::Publisher pub_map = nh.advertise<sensor_msgs::PointCloud2>("/SCDR/map",100);
  ros::Publisher pub_pc_curr = nh.advertise<sensor_msgs::PointCloud2>("/SCDR/pc2_curr",100);

  ros::Publisher pub_map_f = nh.advertise<sensor_msgs::PointCloud2>("/SCDR/map_cut",100);
  ros::Publisher pub_pc_curr_f = nh.advertise<sensor_msgs::PointCloud2>("/SCDR/pc2_curr_cut",100);

  ros::Publisher pub_arranged = nh.advertise<sensor_msgs::PointCloud2>("/SCDR/arranged",100);
  ros::Publisher pub_complement = nh.advertise<sensor_msgs::PointCloud2>("/SCDR/complement",100);
  ros::Publisher pub_final = nh.advertise<sensor_msgs::PointCloud2>("/SCDR/final",100);
  ros::Publisher pub_original = nh.advertise<sensor_msgs::PointCloud2>("/SCDR/original",100);
  ros::Rate loop_rate(5);

  pcl::PointCloud<pcl::PointXYZI> pc_map, pc_curr;
  pcl::PointCloud<pcl::PointXYZI> map_transformed, curr_transformed;
  pcl::PointCloud<pcl::PointXYZI> map_f, curr_f; // filtered -> f
  pcl::PointCloud<pcl::PointXYZI> map_arranged, map_complement; // filtered
  pcl::PointCloud<pcl::PointXYZI> map_final, map_original, map_original_transformed;

  // Declaration
  SCDR scdr = SCDR(max_range, num_rings, num_sectors, min_h, max_h);

  Eigen::Matrix4f pose_curr, pose_prev, origin2body_curr, origin2body_prev, prev2curr;
  sensor_msgs::PointCloud2 pc2_map, pc2_map_original, pc2_curr, pc2_map_f, pc2_curr_f;
  string map_dir, pc_dir, pose_dir;

  string filename = std::to_string(init_idx);
  set_dirs(filename, map_dir, pc_dir, pose_dir);
  // Load data
  pose_curr = load_pose(pose_dir);
  origin2body_curr = pose_curr.inverse();

  load_pc(map_dir, pc_map);
  load_pc(pc_dir, pc_curr);
  map_original = pc_map;
  transform(origin2body_curr, pc_map, map_transformed);
  transform(origin2body_curr, pc_curr, curr_transformed);
  transform(origin2body_curr, map_original, map_original_transformed);

  pass_through(map_transformed, map_f);
  pass_through(curr_transformed, curr_f);

  pc2_map = erasor_utilscloud2msg(map_transformed);
  pc2_curr = erasor_utilscloud2msg(curr_transformed);
  pc2_map_f = erasor_utilscloud2msg(map_f);
  pc2_curr_f = erasor_utilscloud2msg(curr_f);

  int count = 0;

  clock_t start, mid1, mid2, end;
  clock_t start_original, end_original;
  clock_t start_kd, end_kd;

  scdr.set_inputs(map_transformed, curr_transformed);
  mid1 = clock();
  scdr.compare_then_select();
  mid2 = clock();
  scdr.get_pcs(map_arranged, map_complement);
  end = clock();
  map_final = map_arranged + map_complement;
  pose_prev = pose_curr;
  int tmp;
  while (true){
    if (tmp != LINE_SPACE){
      Display_Init_Menu();
    }
    std::cout<<"Press Key..."<<std::endl;

    tmp = getchar();

    if (tmp == LINE_SPACE) continue;
    cout<<"Debug: "<<tmp<<endl;
    switch (tmp){
      case 'n':
        count++;
        cout<<"Next"<<endl;
        init_idx += 5;
        filename = std::to_string(init_idx);
        set_dirs(filename, map_dir, pc_dir, pose_dir);
        pose_curr = load_pose(pose_dir);
        origin2body_curr = pose_curr.inverse();
        prev2curr = origin2body_curr * pose_prev;
        pc_map = map_final;

        load_pc(pc_dir, pc_curr);
        load_pc(map_dir, map_original);
        ROS_WARN_STREAM("On transforming...");

        transform(prev2curr, pc_map, map_transformed);
        transform(origin2body_curr, pc_curr, curr_transformed);
        transform(origin2body_curr, map_original, map_original_transformed);
        end_original = clock();

        pass_through(map_transformed, map_f);
        pass_through(curr_transformed, curr_f);

        pc2_map = erasor_utilscloud2msg(map_transformed);
        pc2_curr = erasor_utilscloud2msg(curr_transformed);
        pc2_map_f = erasor_utilscloud2msg(map_f);
        pc2_curr_f = erasor_utilscloud2msg(curr_f);

        scdr.set_inputs(map_transformed, curr_transformed);
        scdr.compare_then_select();
        scdr.get_pcs(map_arranged, map_complement);


        map_final = map_arranged + map_complement;
        if ((count + 1) % 7 == 0){
          double leaf_size = 0.25;
          static pcl::VoxelGrid<pcl::PointXYZI> voxel_filter;
          pcl::PointCloud<pcl::PointXYZI>::Ptr src(new pcl::PointCloud<pcl::PointXYZI>);
          pcl::PointCloud<pcl::PointXYZI>::Ptr ptr_voxelized(new pcl::PointCloud<pcl::PointXYZI>);
          *src = map_final;
          voxel_filter.setInputCloud(src);
          voxel_filter.setLeafSize(leaf_size, leaf_size, leaf_size);
          voxel_filter.filter(*ptr_voxelized);
          map_final = *ptr_voxelized;

        }
        ROS_WARN_STREAM("Removal done.");
        pose_prev = pose_curr;
        break;
      case 'p':
        cout<<"Republish"<<endl;

        break;
     case 'c':
       cout<<"Exit"<<endl;
       return 0;

     }

     for (int i=0; i < 3; ++i){
         if ( (i+1) % 2 == 0) cout<<i<<"th publish..."<<endl;

//         cout<<i<<"th |";
//         cout<<"Takes "<<(static_cast<double>(end - start))/CLOCKS_PER_SEC <<std::endl;
//         cout<<(static_cast<double>(mid1 - start))/CLOCKS_PER_SEC<<", ";
//         cout<<(static_cast<double>(mid2 - mid1))/CLOCKS_PER_SEC<<", ";
//         cout<<(static_cast<double>(end - mid2))/CLOCKS_PER_SEC<<endl;

         sensor_msgs::PointCloud2 pc2_arranged = erasor_utilscloud2msg(map_arranged);
         sensor_msgs::PointCloud2 pc2_complement = erasor_utilscloud2msg(map_complement);
         sensor_msgs::PointCloud2 pc2_map_final = erasor_utilscloud2msg(map_final);
         sensor_msgs::PointCloud2 pc2_map_original = erasor_utilscloud2msg(map_original_transformed);

         pub_final.publish(pc2_map_final);
         pub_original.publish(pc2_map_original);
         pub_map.publish(pc2_map);
         pub_pc_curr.publish(pc2_curr);
         pub_map_f.publish(pc2_map_f); // for debugging
         pub_pc_curr_f.publish(pc2_curr_f);
         pub_arranged.publish(pc2_arranged);
         pub_complement.publish(pc2_complement);

         ros::spinOnce();
         loop_rate.sleep();
     }
  }
  return 0;
}


