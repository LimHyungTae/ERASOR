//
// Created by shapelim on 21. 10. 18..
//

#include "tools/erasor_utils.hpp"
#include <boost/format.hpp>
#include <cstdlib>
#include <erasor/OfflineMapUpdater.h>

// Heuristic, yet boosting speed by vector.reserve()
#define MAP_CLOUD_LARGE_ENOUGH 2000000
#define QUERY_CLOUD_LARGE_ENOUGH 200000

unique_ptr<ERASOR> ErasorVel16;

string DATA_DIR;
int INTERVAL, INIT_IDX;
float VOXEL_SIZE, MAX_RANGE;
std::string filename = "/staticmap_via_erasor.pcd";

using PointType = pcl::PointXYZI;

void fetch_VoI(
        Eigen::Matrix4f& pose, double max_range,
        const pcl::PointCloud<PointType> &map,
        pcl::PointCloud<PointType> &inliers,
        pcl::PointCloud<PointType> &outskirts,
        pcl::PointCloud<PointType> &map_voi, std::string mode="naive") {
    // 1. Divide map_arranged into map_central and map_outskirts
    static double margin = 0;
    inliers.clear();
    outskirts.clear();
    map_voi.clear();

    if (mode == "naive") {
        double max_dist_square = pow(max_range + margin, 2);
        for (auto const &pt : map.points) {
            double dist_square = pow(pt.x - pose(0, 3), 2) + pow(pt.y - pose(1, 3), 2);
            if (dist_square < max_dist_square) {
                inliers.push_back(pt);
            } else {
                outskirts.push_back(pt);
            }
        }
    }
    pcl::PointCloud<PointType>::Ptr ptr_transformed(new pcl::PointCloud<PointType>);
    pcl::transformPointCloud(inliers, *ptr_transformed, pose.inverse());
    map_voi = *ptr_transformed;
}

vector<float> split_line(string input, char delimiter) {
    vector<float> answer;
    stringstream ss(input);
    string temp;

    while (getline(ss, temp, delimiter)) {
        answer.push_back(stof(temp));
    }
    return answer;
}

void load_all_poses(string txt, vector<Eigen::Matrix4f >& poses){
    // These poses are already w.r.t body frame!
    // Thus, tf4x4 by pose * corresponding cloud -> map cloud
    cout<<"Target path: "<< txt<<endl;
    poses.clear();
    poses.reserve(2000);
    std::ifstream in(txt);
    std::string line;

    int count = 0;
    while (std::getline(in, line)) {
        if (count == 0){
            count++;
            continue;
        }

        vector<float> pose = split_line(line, ',');

        Eigen::Translation3f ts(pose[2], pose[3], pose[4]);
        Eigen::Quaternionf q(pose[8], pose[5], pose[6], pose[7]);
        Eigen::Matrix4f tf4x4_cam = Eigen::Matrix4f::Identity(); // Crucial!
        tf4x4_cam.topLeftCorner<3, 3>(0, 0) = q.toRotationMatrix();
        tf4x4_cam.topRightCorner(3, 1) = ts.vector();

        Eigen::Matrix4f tf4x4_lidar = tf4x4_cam;
        poses.emplace_back(tf4x4_lidar);
        count++;
    }
    std::cout<<"Total "<<count<<" poses are loaded"<<std::endl;
}

template<typename T>
void voxelize(const boost::shared_ptr<pcl::PointCloud<T> > srcPtr, pcl::PointCloud<T> &dst, double voxelSize) {
    static pcl::VoxelGrid<T> voxel_filter;
    voxel_filter.setInputCloud(srcPtr);
    voxel_filter.setLeafSize(voxelSize, voxelSize, voxelSize);
    voxel_filter.filter(dst);
}

template<typename T>
void voxelize(const boost::shared_ptr<pcl::PointCloud<T> > srcPtr, boost::shared_ptr<pcl::PointCloud<T> > dstPtr,
              double voxelSize) {
    static pcl::VoxelGrid<T> voxel_filter;
    voxel_filter.setInputCloud(srcPtr);
    voxel_filter.setLeafSize(voxelSize, voxelSize, voxelSize);
    voxel_filter.filter(*dstPtr);
}


void set_nav_path(const Eigen::Matrix4f& pose, const int& idx, nav_msgs::Path& path){
    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header.stamp = ros::Time::now();
    pose_stamped.header.seq = idx;
    pose_stamped.header.frame_id = "/map";
    pose_stamped.pose = erasor_utils::eigen2geoPose(pose);

    path.header = pose_stamped.header;
    path.poses.push_back(pose_stamped);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "erasor_in_your_env");
    ros::NodeHandle nh;
    erasor::OfflineMapUpdater updater = erasor::OfflineMapUpdater();

    nh.param<string>("/data_dir", DATA_DIR, "/");
    nh.param<float>("/voxel_size", VOXEL_SIZE, 0.075);
    nh.param<int>("/init_idx", INIT_IDX, 0);
    nh.param<int>("/interval", INTERVAL, 2);

    std::string staticmap_path = std::getenv("HOME") + filename;

    // Set ROS visualization publishers
    ros::Publisher NodePublisher = nh.advertise<erasor::node>("/node/combined/optimized", 100);
    ros::Rate loop_rate(10);

    /***
     * Set target data
     * Note that pcd files are in `pcd_dir`
     * And check that each i-th pose corresponds to i-th pcd file
     *
     * In our example, transformation result (pose * point cloud) denotes
     * partial part of map cloud directly
     * (i.e. there are no additional transformation matrix, e.g. lidar2body)
     *
     * In your own case, be careful to set max_h and min_h correctly!
     */
    // Set target data
    cout << "\033[1;32mTarget directory:" << DATA_DIR << "\033[0m" << endl;
    string raw_map_path = DATA_DIR + "/dense_global_map.pcd";
    string pose_path = DATA_DIR + "/poses_lidar2body.csv";
    string pcd_dir = DATA_DIR + "/pcds"; //
    // Load raw pointcloud

    vector<Eigen::Matrix4f> poses;
    load_all_poses(pose_path, poses);

    int N = poses.size();

    for (int i = INIT_IDX; i < N; ++i) {
        signal(SIGINT, erasor_utils::signal_callback_handler);

        pcl::PointCloud<PointType>::Ptr srcCloud(new pcl::PointCloud<PointType>);
        string pcd_name = (boost::format("%s/%06d.pcd") % pcd_dir % i).str();
        erasor_utils::load_pcd(pcd_name, srcCloud);

        erasor::node node;
        node.header.seq = i;
        node.odom = erasor_utils::eigen2geoPose(poses[i]);
        node.lidar = erasor_utils::cloud2msg(*srcCloud);
        NodePublisher.publish(node);
        ros::spinOnce();
        loop_rate.sleep();
    }

    updater.save_static_map(0.2);

    cout<< "Static map building complete!" << endl;

    return 0;
}