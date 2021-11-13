//
// Created by shapelim on 21. 10. 18..
//

#include "erasor/erasor.h"
#include "tools/erasor_utils.hpp"
#include <boost/format.hpp>
#include <cstdlib>

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
//
    ros::NodeHandle nh;
    nh.param<string>("/data_dir", DATA_DIR, "/");
    nh.param<float>("/voxel_size", VOXEL_SIZE, 0.075);
    nh.param<int>("/init_idx", INIT_IDX, 0);
    nh.param<int>("/interval", INTERVAL, 2);

    std::string staticmap_path = std::getenv("HOME") + filename;

    // Set ROS visualization publishers
    ros::Publisher MapPublisher = nh.advertise<sensor_msgs::PointCloud2>("/map", 100);
    ros::Publisher ScanPublisher = nh.advertise<sensor_msgs::PointCloud2>("/scan", 100);
    ros::Publisher PosePublisher = nh.advertise<nav_msgs::Path>("/nav_pose",100);
    ros::Rate loop_rate(100);

    pcl::PointCloud<PointType> mapRaw;
    nav_msgs::Path nav_path;
    mapRaw.reserve(MAP_CLOUD_LARGE_ENOUGH);

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
    string raw_map_path = DATA_DIR + "/dense_global_map.pcd";
    string pose_path = DATA_DIR + "/poses_lidar2body.csv";
    string pcd_dir = DATA_DIR + "/pcds"; //
    // Load raw pointcloud

    std::cout << "[Initialization] Try to load naively accumulated map...(it may take some time)" << std::endl;
    erasor_utils::load_pcd(raw_map_path, mapRaw);


    // map_arranged: Output of the filtered map
    pcl::PointCloud<PointType>::Ptr mapArranged(new pcl::PointCloud<PointType>);
    *mapArranged = mapRaw;
    voxelize(mapArranged, mapRaw, 0.1);

    *mapArranged = mapRaw;
    cout<<raw_map_path<<endl;
    ErasorVel16.reset(new ERASOR(&nh));
    vector<Eigen::Matrix4f> poses;
    load_all_poses(pose_path, poses);
    MAX_RANGE = ErasorVel16->get_max_range();

    std::cout << "[Initialization] Loading complete" << std::endl;

    int N = poses.size();
    for (int i = INIT_IDX; i < N; ++i) {
        signal(SIGINT, erasor_utils::signal_callback_handler);
        if (i % INTERVAL != 0) continue;

        std::cout<<i<<" / "<< N<<" th operation"<<std::endl;
        Eigen::Matrix4f targetTf4x4 = poses[i];

        // Current scan
        pcl::PointCloud<PointType>::Ptr srcCloud(new pcl::PointCloud<PointType>);
        pcl::PointCloud<PointType>::Ptr queryVoI(new pcl::PointCloud<PointType>);
        string pcd_name = (boost::format("%s/%06d.pcd") % pcd_dir % i).str();
        erasor_utils::load_pcd(pcd_name, srcCloud);

        *queryVoI = *srcCloud;
        // To debug
        pcl::PointCloud<PointType>::Ptr ptrTransformedCloud(new pcl::PointCloud<PointType>);
        pcl::transformPointCloud(*srcCloud, *ptrTransformedCloud, targetTf4x4);
        sensor_msgs::PointCloud2 currCloudMsg = erasor_utils::cloud2msg(*ptrTransformedCloud);
        ScanPublisher.publish(currCloudMsg);

        // Set VoI
        pcl::PointCloud<PointType>::Ptr inliers(new pcl::PointCloud<PointType>);
        pcl::PointCloud<PointType>::Ptr mapVoI(new pcl::PointCloud<PointType>);
        pcl::PointCloud<PointType>::Ptr mapOutskirt(new pcl::PointCloud<PointType>);
        // ERASOR outputs
        pcl::PointCloud<PointType>::Ptr mapStaticPtsBody(new pcl::PointCloud<PointType>);
        pcl::PointCloud<PointType>::Ptr mapStaticPtsMap(new pcl::PointCloud<PointType>);
        pcl::PointCloud<PointType>::Ptr mapStaticEstimate(new pcl::PointCloud<PointType>);
        pcl::PointCloud<PointType>::Ptr mapEgocentricCompelment(new pcl::PointCloud<PointType>);
        pcl::PointCloud<PointType>::Ptr tmpMapRejected(new pcl::PointCloud<PointType>);
        pcl::PointCloud<PointType>::Ptr tmpCurrRejected(new pcl::PointCloud<PointType>);

        mapOutskirt->points.reserve(MAP_CLOUD_LARGE_ENOUGH);
        inliers->points.reserve(QUERY_CLOUD_LARGE_ENOUGH);
        mapVoI->points.reserve(QUERY_CLOUD_LARGE_ENOUGH);

        fetch_VoI(targetTf4x4, MAX_RANGE, *mapArranged, *inliers, *mapOutskirt, *mapVoI);

        ROS_INFO_STREAM("Before: \033[1;32m" << mapVoI->points.size() <<" vs " << queryVoI->points.size() << "\033[0m ");
        ErasorVel16->set_inputs(*mapVoI, *queryVoI);
        int erasor_version_ = 2;
        if (erasor_version_ == 2) {
            ErasorVel16->compare_vois_and_revert_ground(i);
            ErasorVel16->get_static_estimate(*mapStaticEstimate, *mapEgocentricCompelment);
        // Not in use
        } else if (erasor_version_ == 3) {
            ErasorVel16->compare_vois_and_revert_ground_w_block(i);
            ErasorVel16->get_static_estimate(*mapStaticEstimate, *mapEgocentricCompelment);
        } else {
            throw invalid_argument("Other version is not implemented!");
        }
        auto end = ros::Time::now().toSec();
        ErasorVel16->get_outliers(*tmpMapRejected, *tmpCurrRejected);

        *mapStaticPtsBody = *mapStaticEstimate + *mapEgocentricCompelment;
        pcl::transformPointCloud(*mapStaticPtsBody, *mapStaticPtsMap, targetTf4x4);
        mapArranged->clear();
        *mapArranged = *mapStaticPtsMap + *mapOutskirt;

        std::cout << "Voxelization conducted" << std::endl;
        pcl::PointCloud<PointType>::Ptr tmpMapVoxeled(new pcl::PointCloud<PointType>);
        voxelize(mapArranged, tmpMapVoxeled, VOXEL_SIZE);
//        std::cout << "Map cloud: " << mapCloud.points.size() << std::endl;
        static int cnt = 0;
        if (++cnt % 5 == 0) {
            sensor_msgs::PointCloud2 mapCloudMsg = erasor_utils::cloud2msg(*tmpMapVoxeled);
            MapPublisher.publish(mapCloudMsg);
            *mapArranged = *tmpMapVoxeled;
        }

        if (i > N - 10){
            pcl::io::savePCDFileASCII(staticmap_path, *mapArranged);
        }

        // Path
        set_nav_path(targetTf4x4, i, nav_path);
        PosePublisher.publish(nav_path);

        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}