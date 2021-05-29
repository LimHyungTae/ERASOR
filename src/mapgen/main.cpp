#include "mapgen.hpp"

mapgen     mapgenerator;

ros::Publisher  cloudPublisher;
ros::Publisher  mapPublisher;
ros::Publisher  pathPublisher;
nav_msgs::Path path;
using namespace erasor;

void callbackData(const node msg)
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloudCurr(new pcl::PointCloud<pcl::PointXYZI>());
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloudMap(new pcl::PointCloud<pcl::PointXYZI>());

  std::cout<<"- - - - - - - - - - - - - - -  - - - - - - - - - -"<<std::endl;
  std::cout<<"[MAPGEN] " <<msg.header.seq<< "th sequence come"<<std::endl;
  std::cout<<"- - - - - - - - - - - - - - -  - - - - - - - - - -"<<std::endl;
  mapgenerator.putData(msg, cloudMap, cloudCurr, path);

  std::cout<<"map: "<<(*cloudMap).size()<<" curr: "<< (*cloudCurr).size()<<std::endl;

  cloudPublisher.publish(erasor_utilscloud2msg(*cloudCurr));
  mapPublisher.publish(erasor_utilscloud2msg(*cloudMap));
  pathPublisher.publish(path);
}
#include <std_msgs/Int32.h>

void saveMap(std_msgs::Int32& req){
  std::cout<<"Hello!"<<std::endl;
  if (req.data == 1){
    std::cout<<"Debug!"<<std::endl;
  }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "merger");
    std::cout<<"KiTTI MAPGEN STARTED"<<std::endl;
    ros::NodeHandle   nodeHandler;
    float voxelsize;
    std::string sequence, init_stamp, final_stamp;
    int interval;
    nodeHandler.param("/map/voxelsize", voxelsize,(float)0.05);
    nodeHandler.param<std::string>("/map/sequence", sequence, "00");
    nodeHandler.param<std::string>("/map/init_stamp", init_stamp, "150");
    nodeHandler.param<std::string>("/map/final_stamp", final_stamp, "200");
    nodeHandler.param<int>("/map/interval", interval, 3);

    mapgenerator.setValue(voxelsize, sequence, init_stamp, final_stamp, interval);
    mapPublisher = nodeHandler.advertise<sensor_msgs::PointCloud2>("/mapgen/map", 100);
    cloudPublisher = nodeHandler.advertise<sensor_msgs::PointCloud2>("/mapgen/curr", 100);
    pathPublisher = nodeHandler.advertise<nav_msgs::Path>("/path", 100);

    ros::Subscriber subData =
        nodeHandler.subscribe<node>("/node/combined/optimized",1000,callbackData);
    ros::spin();

    return 0;
}
