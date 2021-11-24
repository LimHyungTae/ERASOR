#include <ros/ros.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <tools/erasor_utils.hpp>
#include <string>
#include <map>
#include <vector>

ros::Publisher mapPublisher;
ros::Publisher dynamicObjPublisher;
ros::Publisher targetPublisher;

std::vector<int>   dynamic_classes = {252, 253, 254, 255, 256, 257, 258, 259};
std::map<int, int> dynamic_objects; // <id, class_num> These indicate the id of moving objects
std::vector<int>   all_dynamic_ids;
std::vector<int>   moving_dynamic_ids;

sensor_msgs::PointCloud2 map_msg;
sensor_msgs::PointCloud2 dyn_obj_msg;
sensor_msgs::PointCloud2 target_msg;

void parse_dynamic_obj(
        const pcl::PointCloud<pcl::PointXYZI> &cloudIn, pcl::PointCloud<pcl::PointXYZI> &dynamicOut,
        pcl::PointCloud<pcl::PointXYZI> &staticOut) {
    dynamicOut.points.clear();
    staticOut.points.clear();

    for (const auto &pt: cloudIn.points) {
        uint32_t float2int      = static_cast<uint32_t>(pt.intensity);
        uint32_t semantic_label = float2int & 0xFFFF;
        uint32_t inst_label     = float2int >> 16;
        bool     is_static      = true;
        for (int class_num: dynamic_classes) {
            if (semantic_label == class_num) { // 1. check it is in the moving object classes
                dynamicOut.points.push_back(pt);
                is_static = false;
            }
        }
        if (is_static) {
            staticOut.points.push_back(pt);

        }
    }
}

void fetch_specific_object(
        const pcl::PointCloud<pcl::PointXYZI> &dynamicObjs, uint32_t target_class_num, uint32_t target_id,
        pcl::PointCloud<pcl::PointXYZI> &targetObj) {
    targetObj.points.clear();

    for (const auto &pt: dynamicObjs.points) {
        uint32_t float2int      = static_cast<uint32_t>(pt.intensity);
        uint32_t semantic_label = float2int & 0xFFFF;
        uint32_t inst_label     = float2int >> 16;
        if ((semantic_label == target_class_num) && (inst_label == target_id)) {
            std::cout << "Find target cloud" << std::endl;
            targetObj.points.push_back(pt);
        }
    }
}


void fetch_specific_class(
        const pcl::PointCloud<pcl::PointXYZI> &dynamicObjs, uint32_t target_class_num, pcl::PointCloud<pcl::PointXYZI> &targetObj) {

    targetObj.points.clear();

    for (const auto &pt: dynamicObjs.points) {
        uint32_t float2int      = static_cast<uint32_t>(pt.intensity);
        uint32_t semantic_label = float2int & 0xFFFF;
        uint32_t inst_label     = float2int >> 16;
        if (semantic_label == target_class_num) {
            std::cout << "Find target cloud" << std::endl;
            targetObj.points.push_back(pt);
        }
    }
}

void callback_flag(const std_msgs::Float32::ConstPtr &msg) {
    std::cout << "Trying to publish clouds!" << std::endl;
    mapPublisher.publish(map_msg);
    dynamicObjPublisher.publish(dyn_obj_msg);
    targetPublisher.publish(target_msg);
    std::cout << "Publish complete!" << std::endl;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "mapviz");
    std::cout << "KiTTI MAPVIZ STARTED" << std::endl;
    ros::NodeHandle nodeHandler;

    std::string map_name, seq;
    int         class_num, id;
    uint32_t    target_class_num, target_id;
    nodeHandler.param<std::string>("/filename", map_name, "/");
    nodeHandler.param("/class_num", class_num, 260);
    nodeHandler.param("/id", id, 0);

    target_class_num = static_cast<uint32_t>(class_num);
    target_id        = static_cast<uint32_t>(id);

    ros::Subscriber sub = nodeHandler.subscribe<std_msgs::Float32>("/pub_flag", 10, &callback_flag);

    mapPublisher        = nodeHandler.advertise<sensor_msgs::PointCloud2>("/mapviz", 100);
    dynamicObjPublisher = nodeHandler.advertise<sensor_msgs::PointCloud2>("/dynamicObj", 100);
    targetPublisher     = nodeHandler.advertise<sensor_msgs::PointCloud2>("/target", 100);

    pcl::PointCloud<pcl::PointXYZI>::Ptr ptr_map(new pcl::PointCloud<pcl::PointXYZI>);

    cout << "On loading target file...it takes few seconds..." << endl;
    erasor_utils::load_pcd(map_name, ptr_map);
    cout << "Load map complete" << endl;
    pcl::PointCloud<pcl::PointXYZI> dynamic_objs, static_objs, target_obj;
    parse_dynamic_obj(*ptr_map, dynamic_objs, static_objs);
    fetch_specific_class(dynamic_objs, target_class_num, target_obj);
    cout<<"Total "<< target_obj.points.size()<<" Points exist"<<endl;

    map_msg           = erasor_utils::cloud2msg(static_objs);
    dyn_obj_msg       = erasor_utils::cloud2msg(dynamic_objs);
    target_msg        = erasor_utils::cloud2msg(target_obj);

    ros::Rate  loop_rate(1);
    static int count_ = 0;

    bool is_once = true;
    if (is_once) {
        mapPublisher.publish(map_msg);
        dynamicObjPublisher.publish(dyn_obj_msg);
        targetPublisher.publish(target_msg);
    } else {
        while (ros::ok()) {
            signal(SIGINT, erasor_utils::signal_callback_handler);

            mapPublisher.publish(map_msg);
            dynamicObjPublisher.publish(dyn_obj_msg);
            targetPublisher.publish(target_msg);
            if (++count_ % 30 == 0) std::cout << "On " << count_ << "th publish!" << std::endl;

            ros::spinOnce();
            loop_rate.sleep();
        }
    }
    ros::spin();

    return 0;
}
