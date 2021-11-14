#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <unavlib/convt.h>
#include <unavlib/others.h>
#include <string>
#include <map>
#include <vector>
#include <visualization_msgs/Marker.h>
#include <std_msgs/ColorRGBA.h>
#include <jsk_recognition_msgs/PolygonArray.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/Polygon.h>
#include <geometry_msgs/Point32.h>

ros::Publisher  vizPublisher;

using namespace unavlib;


int main(int argc, char **argv)
{
    ros::init(argc, argv, "vizmarker");

    ros::NodeHandle nodeHandler;

    vizPublisher = nodeHandler.advertise<jsk_recognition_msgs::PolygonArray>("/poly_marker", 100);

    jsk_recognition_msgs::PolygonArray poly_list;
    poly_list.header.frame_id = "/world";
    poly_list.header.stamp = ros::Time::now();

    // Elements
    geometry_msgs::PolygonStamped polygons;
    polygons.header = poly_list.header;

    geometry_msgs::Point32 point;

    point.x = 2.0; point.y = 0.0; point.z = 5.0;
    polygons.polygon.points.push_back(point);
    polygons.polygon.points.push_back(point);
    point.x = 4.0; point.y = 0.0; point.z = 5.0;
    polygons.polygon.points.push_back(point);
    point.x = 0.0; point.y = 4.0; point.z = 5.0;
    polygons.polygon.points.push_back(point);

    point.x = 0.0; point.y = 2.0; point.z = 5.0;
    polygons.polygon.points.push_back(point);
    poly_list.polygons.push_back(polygons);

    poly_list.likelihood.push_back(0.0);

    ros::Rate loop_rate(2);
    while (ros::ok())
    {
      static int cnt;
      vizPublisher.publish(poly_list);
      std::cout<<"pub! "<<++cnt<<std::endl;
      ros::spinOnce();
      loop_rate.sleep();
    }
    return 0;
}
