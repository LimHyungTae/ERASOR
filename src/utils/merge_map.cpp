#include <ros/ros.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <unavlib/convt.h>
#include <unavlib/others.h>
#include <string>
#include <map>
#include <vector>

using namespace unavlib;
using namespace std;

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

int main(int argc, char **argv)
{
    ros::init(argc, argv, "merge_map");
    std::cout<<"KiTTI MAP-Merge STARTED"<<std::endl;
    ros::NodeHandle nodeHandler;
    // For 475~625 + 824~1450
//    std::string s0 = "/media/shapelim/UX960NVMe1/kitti_semantic/semantic_KITTI_map/src_0_2/seq05/05_475_to_625_w_interval1_voxel_0.200000.pcd";
//    std::string s1 = "/media/shapelim/UX960NVMe1/kitti_semantic/semantic_KITTI_map/src_0_2/seq05/05_824_to_1450_w_interval1_voxel_0.200000.pcd";
//    pcl::PointCloud<pcl::PointXYZI>::Ptr ptr0(new pcl::PointCloud<pcl::PointXYZI>);
//    pcl::PointCloud<pcl::PointXYZI>::Ptr ptr1(new pcl::PointCloud<pcl::PointXYZI>);

//    datahandle3d::load_pcd(s0, ptr0);
//    datahandle3d::load_pcd(s1, ptr1);
//    pcl::PointCloud<pcl::PointXYZI> mapOut;
//    cout<<"src0 : "<<ptr0->points.size()<<endl;
//    cout<<"src1 : "<<ptr1->points.size()<<endl;

    // case 1. prediction merge
//    std::string s0 = "/media/shapelim/UX960NVMe1/kitti_semantic/semantic_KITTI_map/src_0_2/seq05/total_map_src/05_25_to_475_w_interval1_voxel_0.200000.pcd";
//    std::string s1 = "/media/shapelim/UX960NVMe1/kitti_semantic/semantic_KITTI_map/src_0_2/seq05/total_map_src/05_625_to_825_w_interval1_voxel_0.200000.pcd";
//    std::string s2 = "/media/shapelim/UX960NVMe1/kitti_semantic/semantic_KITTI_map/src_0_2/seq05/total_map_src/05_1450_to_1900_w_interval1_voxel_0.200000.pcd";
//    std::string s3 = "/media/shapelim/UX960NVMe1/kitti_semantic/semantic_KITTI_map/src_0_2/seq05/total_map_src/partial_05_475_625_824_1450_result.pcd";
//    std::string s4 = "/media/shapelim/UX960NVMe1/kitti_semantic/semantic_KITTI_map/src_0_2/seq05/total_map_src/partial_05_1900_to_2350.pcd";
//    std::string s5 = "/media/shapelim/UX960NVMe1/kitti_semantic/semantic_KITTI_map/src_0_2/seq05/total_map_src/partial_05_2350_2670.pcd";

//    pcl::PointCloud<pcl::PointXYZI>::Ptr ptr0(new pcl::PointCloud<pcl::PointXYZI>);
//    pcl::PointCloud<pcl::PointXYZI>::Ptr ptr1(new pcl::PointCloud<pcl::PointXYZI>);
//    pcl::PointCloud<pcl::PointXYZI>::Ptr ptr2(new pcl::PointCloud<pcl::PointXYZI>);
//    pcl::PointCloud<pcl::PointXYZI>::Ptr ptr3(new pcl::PointCloud<pcl::PointXYZI>);
//    pcl::PointCloud<pcl::PointXYZI>::Ptr ptr4(new pcl::PointCloud<pcl::PointXYZI>);
//    pcl::PointCloud<pcl::PointXYZI>::Ptr ptr5(new pcl::PointCloud<pcl::PointXYZI>);


//    datahandle3d::load_pcd(s0, ptr0);
//    datahandle3d::load_pcd(s1, ptr1);
//    datahandle3d::load_pcd(s2, ptr2);
//    datahandle3d::load_pcd(s3, ptr3);
//    datahandle3d::load_pcd(s4, ptr4);
//    datahandle3d::load_pcd(s5, ptr5);
//    pcl::PointCloud<pcl::PointXYZI> tmp0, tmp1, tmp2, tmpOut, mapOut;

//    cout<<"src0 : "<<ptr0->points.size()<<endl;
//    cout<<"src1 : "<<ptr1->points.size()<<endl;
//    cout<<"src2 : "<<ptr2->points.size()<<endl;
//    cout<<"src3 : "<<ptr3->points.size()<<endl;
//    cout<<"src4 : "<<ptr4->points.size()<<endl;
//    cout<<"src5 : "<<ptr5->points.size()<<endl;
//    tmp0 = *ptr0 + *ptr1;
//    tmp1 = *ptr2 + *ptr3;
//    tmp2 = *ptr4 + *ptr5;

//    tmpOut = tmp0 + tmp1;
//    tmpOut = tmpOut + tmp2;
//    cout<<"tmpOut : "<<tmpOut.points.size()<<endl;

    // Case 2. raw material
//    std::string s0 = "/media/shapelim/UX960NVMe1/kitti_semantic/semantic_KITTI_map/src_0_2/seq05/total_map_raw/05_25_to_475_w_interval1_voxel_0.200000.pcd";
//    std::string s1 = "/media/shapelim/UX960NVMe1/kitti_semantic/semantic_KITTI_map/src_0_2/seq05/total_map_raw/05_475_to_625_w_interval1_voxel_0.200000.pcd";
//    std::string s2 = "/media/shapelim/UX960NVMe1/kitti_semantic/semantic_KITTI_map/src_0_2/seq05/total_map_raw/05_625_to_825_w_interval1_voxel_0.200000.pcd";
//    std::string s3 = "/media/shapelim/UX960NVMe1/kitti_semantic/semantic_KITTI_map/src_0_2/seq05/total_map_raw/05_824_to_1450_w_interval1_voxel_0.200000.pcd";
//    std::string s4 = "/media/shapelim/UX960NVMe1/kitti_semantic/semantic_KITTI_map/src_0_2/seq05/total_map_raw/05_1450_to_1900_w_interval1_voxel_0.200000.pcd";
//    std::string s5 = "/media/shapelim/UX960NVMe1/kitti_semantic/semantic_KITTI_map/src_0_2/seq05/total_map_raw/05_1900_to_2350_w_interval1_voxel_0.200000.pcd";
//    std::string s6 = "/media/shapelim/UX960NVMe1/kitti_semantic/semantic_KITTI_map/src_0_2/seq05/total_map_raw/05_2350_to_2670_w_interval2_voxel_0.200000.pcd";

//    pcl::PointCloud<pcl::PointXYZI>::Ptr ptr0(new pcl::PointCloud<pcl::PointXYZI>);
//    pcl::PointCloud<pcl::PointXYZI>::Ptr ptr1(new pcl::PointCloud<pcl::PointXYZI>);
//    pcl::PointCloud<pcl::PointXYZI>::Ptr ptr2(new pcl::PointCloud<pcl::PointXYZI>);
//    pcl::PointCloud<pcl::PointXYZI>::Ptr ptr3(new pcl::PointCloud<pcl::PointXYZI>);
//    pcl::PointCloud<pcl::PointXYZI>::Ptr ptr4(new pcl::PointCloud<pcl::PointXYZI>);
//    pcl::PointCloud<pcl::PointXYZI>::Ptr ptr5(new pcl::PointCloud<pcl::PointXYZI>);
//    pcl::PointCloud<pcl::PointXYZI>::Ptr ptr6(new pcl::PointCloud<pcl::PointXYZI>);


//    datahandle3d::load_pcd(s0, ptr0);
//    datahandle3d::load_pcd(s1, ptr1);
//    datahandle3d::load_pcd(s2, ptr2);
//    datahandle3d::load_pcd(s3, ptr3);
//    datahandle3d::load_pcd(s4, ptr4);
//    datahandle3d::load_pcd(s5, ptr5);
//    datahandle3d::load_pcd(s6, ptr6);
//    pcl::PointCloud<pcl::PointXYZI> tmp0, tmp1, tmp2, tmpOut, mapOut;

//    cout<<"src0 : "<<ptr0->points.size()<<endl;
//    cout<<"src1 : "<<ptr1->points.size()<<endl;
//    cout<<"src2 : "<<ptr2->points.size()<<endl;
//    cout<<"src3 : "<<ptr3->points.size()<<endl;
//    cout<<"src4 : "<<ptr4->points.size()<<endl;
//    cout<<"src5 : "<<ptr5->points.size()<<endl;
//    cout<<"src6 : "<<ptr6->points.size()<<endl;
//    tmp0 = *ptr0 + *ptr1;
//    tmp1 = *ptr2 + *ptr3;
//    tmp2 = *ptr4 + *ptr5;
//    tmp2 = tmp2 + *ptr6;

//    tmpOut = tmp0 + tmp1;
//    tmpOut = tmpOut + tmp2;
//    cout<<"tmpOut : "<<tmpOut.points.size()<<endl;
    // case 3
    std::string s0 = "/media/shapelim/UX960NVMe1/kitti_semantic/semantic_KITTI_map/src_0_2/seq05/merge_final_05_prediction.pcd";
    std::string s1 = "/media/shapelim/UX960NVMe1/kitti_semantic/semantic_KITTI_map/src_0_2/seq05/total_map_src/partial_05_475_625_824_1450_result_v2.pcd";

    pcl::PointCloud<pcl::PointXYZI>::Ptr ptr0(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr ptr1(new pcl::PointCloud<pcl::PointXYZI>);


    datahandle3d::load_pcd(s0, ptr0);
    datahandle3d::load_pcd(s1, ptr1);
    pcl::PointCloud<pcl::PointXYZI> tmp0, tmp1, tmp2, tmpOut, mapOut;

    cout<<"src0 : "<<ptr0->points.size()<<endl;
    cout<<"src1 : "<<ptr1->points.size()<<endl;
    tmpOut = *ptr0 + *ptr1;
    cout<<"tmpOut : "<<tmpOut.points.size()<<endl;

    // voxelization
    pcl::PointCloud<pcl::PointXYZI>::Ptr ptr_src(new pcl::PointCloud<pcl::PointXYZI>);
    *ptr_src = tmpOut;
    cout<<"Voxelizing..."<<endl;
    erasor_utils::voxelize_preserving_labels(ptr_src, mapOut, 0.2); // 0.05m is the criteria!

    mapOut.width = mapOut.points.size();
    mapOut.height = 1;

    pcl::io::savePCDFileASCII("/media/shapelim/UX960NVMe1/kitti_semantic/semantic_KITTI_map/src_0_2/seq05/merge_final_05_prediction_v2.pcd", mapOut);
//    pcl::io::savePCDFileASCII("/media/shapelim/UX960NVMe1/kitti_semantic/semantic_KITTI_map/src_0_2/seq05/merge_final_raw.pcd", mapOut);
    std::cout<<"Complete to save"<<std::endl;

    return 0;
}
