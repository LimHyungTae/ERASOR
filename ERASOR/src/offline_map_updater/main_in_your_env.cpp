//
// Created by shapelim on 21. 10. 18..
//

#include "tools/erasor_utils.hpp"
#include <boost/format.hpp>
#include <cstdlib>
#include <erasor/OfflineMapUpdater.h>

using PointType = pcl::PointXYZI;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "erasor_in_your_env");
    ros::NodeHandle nh;
    erasor::OfflineMapUpdater updater = erasor::OfflineMapUpdater();
    ros::spin();
//    std::string staticmap_path = std::getenv("HOME") + filename;

    // Set ROS visualization publishers
//    ros::Rate loop_rate(10);

//     while (ros::ok() && time_goes_enough) {
//         cout<< "\033[1;36m[ERASOR]: On saving static map..." << endl;
//         updater.save_static_map(0.2);
//         time_goes_enough = false;
//         cout<< "Static map building complete!\033[0m" << endl;
//         ros::spinOnce();
//         loop_rate.sleep();
//     }

    return 0;
}