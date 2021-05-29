#include "mapgen.hpp"

mapgen mapgenerator;

ros::Publisher cloudPublisher;
ros::Publisher mapPublisher;
ros::Publisher pathPublisher;
nav_msgs::Path path;
using namespace erasor;

void callbackData(const node msg) {
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloudCurr(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloudMap(new pcl::PointCloud<pcl::PointXYZI>());

    std::cout << "[MAPGEN] " << msg.header.seq << "th sequence come" << std::endl;
    mapgenerator.accumPointCloud(msg, cloudMap, cloudCurr, path);
    std::cout << "# of map: " << (*cloudMap).size() << " # of curr: " << (*cloudCurr).size() << std::endl;

    cloudPublisher.publish(erasor_utils::cloud2msg(*cloudCurr));
    mapPublisher.publish(erasor_utils::cloud2msg(*cloudMap));
    pathPublisher.publish(path);
}

std::vector<std::string> parse_rosbag_name(std::string& rosbag_name){
    std::string delimiter = "_";
    size_t pos = 0;
    std::vector<std::string> string_parsed;
    std::string token;
    while ((pos = rosbag_name.find(delimiter)) != std::string::npos) {
        token = rosbag_name.substr(0, pos);
        string_parsed.push_back(token);
        rosbag_name.erase(0, pos + delimiter.length());
    }
    return string_parsed;

}
int main(int argc, char **argv) {
    ros::init(argc, argv, "merger");
    ros::NodeHandle nodeHandler;
    std::cout << "KiTTI MAPGEN STARTED" << std::endl;

    std::string target_rosbag;
    std::string sequence;
    std::string init_stamp;
    std::string final_stamp;
    std::string save_path;

    float voxelsize;
    int   interval;



    nodeHandler.param("/map/voxelsize", voxelsize, (float) 0.05);
    nodeHandler.param<std::string>("/map/target_rosbag", target_rosbag, "/");
    nodeHandler.param<std::string>("/map/save_path", save_path, "/");

    auto name_parsed = parse_rosbag_name(target_rosbag);
    sequence = name_parsed[0];
    init_stamp = name_parsed[1];
    final_stamp = name_parsed[3];
    interval = std::stoi(name_parsed[6]);

    mapgenerator.setValue(save_path, voxelsize, sequence, init_stamp, final_stamp, interval);
    mapPublisher   = nodeHandler.advertise<sensor_msgs::PointCloud2>("/mapgen/map", 100);
    cloudPublisher = nodeHandler.advertise<sensor_msgs::PointCloud2>("/mapgen/curr", 100);
    pathPublisher  = nodeHandler.advertise<nav_msgs::Path>("/path", 100);

    ros::Subscriber subData = nodeHandler.subscribe<node>("/node/combined/optimized", 1000, callbackData);
    ros::spin();

    return 0;
}
