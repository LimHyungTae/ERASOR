#include <ros/ros.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <unavlib/convt.h>
#include <unavlib/others.h>
#include <string>
#include <map>
#include <vector>

ros::Publisher  mapPublisher;
ros::Publisher  dynamicObjPublisher;
ros::Publisher  targetPublisher;
using namespace unavlib;

std::vector<int> dynamic_classes = {252, 253, 254, 255, 256, 257, 258, 259};
std::map<int, int> dynamic_objects; // <id, class_num> These indicate the id of moving objects
std::vector<int> all_dynamic_ids;
std::vector<int> moving_dynamic_ids;

void parse_dynamic_obj(const pcl::PointCloud<pcl::PointXYZI>& cloudIn, pcl::PointCloud<pcl::PointXYZI>& dynamicOut, pcl::PointCloud<pcl::PointXYZI>& staticOut){
  dynamicOut.points.clear();
  staticOut.points.clear();

  for (const auto &pt: cloudIn.points){
    uint32_t float2int = static_cast<uint32_t>(pt.intensity);
    uint32_t semantic_label = float2int & 0xFFFF;
    uint32_t inst_label = float2int >> 16;
    bool is_static = true;
    for (int class_num: dynamic_classes){
      if (semantic_label == class_num){ // 1. check it is in the moving object classes
        dynamicOut.points.push_back(pt);
        is_static = false;
      }
    }
    if (is_static){
      staticOut.points.push_back(pt);

    }
  }
}

void fetch_specific_object(const pcl::PointCloud<pcl::PointXYZI>& dynamicObjs, uint32_t target_class_num, uint32_t target_id, pcl::PointCloud<pcl::PointXYZI>& targetObj){
  targetObj.points.clear();

  for (const auto &pt: dynamicObjs.points){
    uint32_t float2int = static_cast<uint32_t>(pt.intensity);
    uint32_t semantic_label = float2int & 0xFFFF;
    uint32_t inst_label = float2int >> 16;
    if ((semantic_label == target_class_num) && (inst_label == target_id)){
      std::cout<<"Find target cloud"<<std::endl;
      targetObj.points.push_back(pt);
    }
  }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "mapviz");
    std::cout<<"KiTTI MAPVIZ STARTED"<<std::endl;
    ros::NodeHandle nodeHandler;

    std::string map_name, seq;
    int class_num, id;
    uint32_t target_class_num, target_id;
    nodeHandler.param<std::string>("/filename", map_name, "/media/shapelim/110d08b2-7282-4ca3-abe3-6ac5b7a011de/이동지능_factory_optimized/Optimized/DRB1_ALL.pcd");
    nodeHandler.param("/class_num", class_num, 260);
    nodeHandler.param("/id", id, 0);

    target_class_num = static_cast<uint32_t>(class_num);
    target_id = static_cast<uint32_t>(id);


    mapPublisher = nodeHandler.advertise<sensor_msgs::PointCloud2>("/mapviz", 100);
    dynamicObjPublisher = nodeHandler.advertise<sensor_msgs::PointCloud2>("/dynamicObj", 100);
    targetPublisher = nodeHandler.advertise<sensor_msgs::PointCloud2>("/target", 100);

    pcl::PointCloud<pcl::PointXYZI>::Ptr ptr_map(new pcl::PointCloud<pcl::PointXYZI>);

    datahandle3d::load_pcd(map_name, ptr_map);
    pcl::PointCloud<pcl::PointXYZI> dynamicObjs, staticObjs, targetObj;
    parse_dynamic_obj(*ptr_map, dynamicObjs, staticObjs);
    fetch_specific_object(dynamicObjs, target_class_num, target_id, targetObj);

    auto msg = erasor_utilscloud2msg(staticObjs);
    auto dynamicObjsmsg = erasor_utilscloud2msg(dynamicObjs);
    auto targetmsg = erasor_utilscloud2msg(targetObj);

    ros::Rate loop_rate(2);
    static int count_ = 0;
    while (ros::ok())
    {
      mapPublisher.publish(msg);
      dynamicObjPublisher.publish(dynamicObjsmsg);
      targetPublisher.publish(targetmsg);
      if (++count_ %2 == 0) std::cout<<"On "<<count_<<"th publish!"<<std::endl;

      ros::spinOnce();
      loop_rate.sleep();
    }
    return 0;
}
