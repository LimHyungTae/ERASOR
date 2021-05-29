#include <ros/ros.h>
#include "erasor/OnlineMapUpdater.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ERASOR_STATIC_MAP_BUILDING");
    ros::NodeHandle nh;
    erasor::OnlineMapUpdater updater = erasor::OnlineMapUpdater();
    ros::spin();

    return 0;
}
