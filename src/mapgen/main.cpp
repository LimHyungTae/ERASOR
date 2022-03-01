#include "mapgen.hpp"

mapgen mapgenerator;

ros::Publisher cloudPublisher;
ros::Publisher mapPublisher;
ros::Publisher pathPublisher;
nav_msgs::Path path;

using namespace erasor;

std::string sequence;
std::string init_stamp;
std::string final_stamp;
std::string save_path;

int   interval;
int   viz_interval;

bool is_large_scale;

float voxelsize;

void saveGlobalMap(){
    std::string original_dir =
                        save_path + "/" + sequence + "_" + init_stamp +
                        "_to_" + final_stamp + "_w_interval" + std::to_string(interval) + "_voxel_" + std::to_string(voxelsize) +
                        "_original.pcd";

    std::string map_dir = save_path + "/" + sequence + "_" + init_stamp +
                          "_to_" + final_stamp + "_w_interval" + std::to_string(interval) + "_voxel_" + std::to_string(voxelsize) +
                          ".pcd";

    mapgenerator.saveNaiveMap(original_dir, map_dir);
}
void callbackSaveFlag(const std_msgs::Float32::ConstPtr &msg) {
    std::cout << "Flag comes!" << std::endl;
    saveGlobalMap();
}

void callbackData(const node msg) {
    signal(SIGINT, erasor_utils::signal_callback_handler);
    static int cnt = 0;
    if ((cnt % viz_interval) == 0){
        std::cout << std::left << setw(nameWidth) << setfill(separator) << "[MAPGEN] " << msg.header.seq << "th frame comes!" << std::endl;
    }

    mapgenerator.accumPointCloud(msg, path);
    if (msg.header.seq >= std::stoi(final_stamp)){
        saveGlobalMap();
    }

    // Visualization
    if ((cnt % viz_interval) == 0){
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloudCurr(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloudMap(new pcl::PointCloud<pcl::PointXYZI>());

        mapgenerator.getPointClouds(cloudMap, cloudCurr);
        cloudPublisher.publish(erasor_utils::cloud2msg(*cloudCurr));
        mapPublisher.publish(erasor_utils::cloud2msg(*cloudMap));
        pathPublisher.publish(path);
    }
    cnt++;
}

std::vector<std::string> parseRosbagName(std::string& rosbag_name){
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

    nodeHandler.param("/map/voxelsize", voxelsize, (float) 0.05);
    nodeHandler.param<std::string>("/map/target_rosbag", target_rosbag, "/");
    nodeHandler.param<std::string>("/map/save_path", save_path, "/");
    nodeHandler.param<int>("/map/viz_interval", viz_interval, 10);

    /*** Below params are for large-scale map building
    /* Because ROS can publish point cloud whose volume is under the 1 GB
    /* And it is to reduce computational burden */
    nodeHandler.param<bool>("/large_scale/is_large_scale", is_large_scale, false);

    auto name_parsed = parseRosbagName(target_rosbag);
    sequence = name_parsed[0];
    init_stamp = name_parsed[1];
    final_stamp = name_parsed[3];
    interval = std::stoi(name_parsed[6]);

    mapgenerator.setValue(save_path, voxelsize, sequence, init_stamp, final_stamp, interval, is_large_scale);
    mapPublisher   = nodeHandler.advertise<sensor_msgs::PointCloud2>("/mapgen/map", 100);
    cloudPublisher = nodeHandler.advertise<sensor_msgs::PointCloud2>("/mapgen/curr", 100);
    pathPublisher  = nodeHandler.advertise<nav_msgs::Path>("/path", 100);

    ros::Subscriber subData = nodeHandler.subscribe<node>("/node/combined/optimized", 3000, callbackData);
    ros::Subscriber subSaveFlag = nodeHandler.subscribe<std_msgs::Float32>("/saveflag", 1000, callbackSaveFlag);
    ros::spin();

    return 0;
}
