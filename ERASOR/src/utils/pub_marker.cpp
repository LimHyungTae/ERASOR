#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <unavlib/convt.h>
#include <unavlib/others.h>
#include <string>
#include <map>
#include <vector>
#include <visualization_msgs/Marker.h>
#include <std_msgs/ColorRGBA.h>
ros::Publisher  vizPublisher;

using namespace unavlib;


int main(int argc, char **argv)
{
    ros::init(argc, argv, "vizmarker");

    ros::NodeHandle nodeHandler;

    vizPublisher = nodeHandler.advertise<visualization_msgs::Marker>("/marker", 100);

    visualization_msgs::Marker cube_list;
    cube_list.header.frame_id = "/map";
    cube_list.header.stamp = ros::Time::now();
    cube_list.ns = "cubes";
    cube_list.action = visualization_msgs::Marker::ADD;
    cube_list.pose.orientation.w = 1.0;
    cube_list.type = visualization_msgs::Marker::CUBE_LIST;
    cube_list.scale.x = 10.0;
    cube_list.scale.y = 10.0;
    cube_list.scale.z = 2.0;

    cube_list.color.r = 0.0;
    cube_list.color.g = 0.0;
    cube_list.color.b = 1.0;
    cube_list.color.a = 0.3;
    cube_list.id = 3;

    geometry_msgs::Point p;
    std_msgs::ColorRGBA test;

    p.x = 10;
    p.y = 10;
    p.z = 10;

    test.r = 1.0; test.g = 0.0; test.b = 1.0; test.a = 1.0;

    cube_list.points.push_back(p);
    cube_list.colors.push_back(test);


    p.x = 0;
    p.y = 0;
    p.z = 0;

    test.r = 0.0; test.g = 1.0; test.b = 1.0; test.a = 1.0;

    cube_list.points.push_back(p);
    cube_list.colors.push_back(test);

    ros::Rate loop_rate(2);
    while (ros::ok())
    {
      vizPublisher.publish(cube_list);
      ros::spinOnce();
      loop_rate.sleep();
    }
    return 0;
}
