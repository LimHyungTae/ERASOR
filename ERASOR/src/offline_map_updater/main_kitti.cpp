#include <ros/ros.h>
#include "erasor/OfflineMapUpdater.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ERASOR_STATIC_MAP_BUILDING");
    ros::NodeHandle nh;
    erasor::OfflineMapUpdater updater = erasor::OfflineMapUpdater();
    ros::spin();

    return 0;
}
