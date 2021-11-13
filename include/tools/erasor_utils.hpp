#ifndef ERASOR_UTILS_H
#define ERASOR_UTILS_H

#include <cstdlib>
#include <ctime>
#include <algorithm>
#include <sstream>
#include <fstream>
#include <cmath>
#include <string>
#include <map>
#include <iostream>
#include <vector>
#include <memory>

#include <ros/ros.h>
#include <pcl/common/common.h>
#include <pcl/common/centroid.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/visualization/pcl_visualizer.h>
// ROS msgs
#include <erasor/node.h>
#include <visualization_msgs/Marker.h>
#include <jsk_recognition_msgs/PolygonArray.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/Polygon.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Point32.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>
#include "signal.h"


namespace erasor_utils {
    template<typename T>
    sensor_msgs::PointCloud2 cloud2msg(pcl::PointCloud<T> cloud, std::string frame_id = "map")
    {
        sensor_msgs::PointCloud2 cloud_ROS;
        pcl::toROSMsg(cloud, cloud_ROS);
        cloud_ROS.header.frame_id = frame_id;
        return cloud_ROS;
    }

    template<typename T>
    pcl::PointCloud<T> cloudmsg2cloud(sensor_msgs::PointCloud2 cloudmsg)
    {
        pcl::PointCloud<T> cloudresult;
        pcl::fromROSMsg(cloudmsg,cloudresult);
        return cloudresult;
    }

    template<typename T>
    void cloudmsg2cloudptr(sensor_msgs::PointCloud2 cloudmsg,boost::shared_ptr< pcl::PointCloud< T > > cloudPtr)
    {
        pcl::fromROSMsg(cloudmsg,*cloudPtr);
    }


    template<typename T>
    int load_pcd(std::string pcd_name, boost::shared_ptr<pcl::PointCloud<T> > dst) {
        if (pcl::io::loadPCDFile<T>(pcd_name, *dst) == -1) {
            PCL_ERROR ("Couldn't read file!!! \n");
            return (-1);
        }
//        std::cout << "Loaded " << dst->size() << " data points from " << pcd_name << std::endl;
        return 0;
    }

    template<typename T>
    int load_pcd(std::string pcd_name, pcl::PointCloud<T>& dst) {
        if (pcl::io::loadPCDFile<T>(pcd_name, dst) == -1) {
            PCL_ERROR ("Couldn't read file!!! \n");
            return (-1);
        }
        std::cout << "Loaded " << dst.size() << " data points from " << pcd_name << std::endl;
        return 0;
    }

    geometry_msgs::Pose eigen2geoPose(Eigen::Matrix4f pose);

    Eigen::Matrix4f geoPose2eigen(geometry_msgs::Pose geoPose);

    void parse_dynamic_obj(
            const pcl::PointCloud<pcl::PointXYZI> &cloudIn, pcl::PointCloud<pcl::PointXYZI> &dynamicOut,
            pcl::PointCloud<pcl::PointXYZI> &staticOut);

    void voxelize_preserving_labels(pcl::PointCloud<pcl::PointXYZI>::Ptr src, pcl::PointCloud<pcl::PointXYZI> &dst, double leaf_size);

    void count_stat_dyn(const pcl::PointCloud<pcl::PointXYZI> &cloudIn, int &num_static, int &num_dynamic);

    void signal_callback_handler(int signum);

}
#endif // ERASOR_UTILS_H
