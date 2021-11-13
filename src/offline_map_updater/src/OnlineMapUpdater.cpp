#include "erasor/OnlineMapUpdater.h"

using namespace erasor;

OnlineMapUpdater::OnlineMapUpdater() {
    sub_node = nh.subscribe<erasor::node>("/node/combined/optimized", 1000, &OnlineMapUpdater::callback_node, this);
    sub_flag = nh.subscribe<std_msgs::Float32>("/saveflag", 10, &OnlineMapUpdater::callback_flag, this);

    pub_path_corrected = nh.advertise<nav_msgs::Path>("/MapUpdater/path_corrected", 100);

    pub_map_init      = nh.advertise<sensor_msgs::PointCloud2>("/MapUpdater/map_init", 100);
    pub_map_rejected  = nh.advertise<sensor_msgs::PointCloud2>("/MapUpdater/map_rejected", 100);
    pub_curr_rejected = nh.advertise<sensor_msgs::PointCloud2>("/MapUpdater/curr_rejected", 100);

    pub_debug_pc2_curr = nh.advertise<sensor_msgs::PointCloud2>("/MapUpdater/pc2_curr", 100);

    pub_debug_map               = nh.advertise<sensor_msgs::PointCloud2>("/MapUpdater/debug/map", 100);
    pub_debug_query_egocentric  = nh.advertise<sensor_msgs::PointCloud2>("/MapUpdater/debug/pc_curr_body", 100);
    pub_debug_map_egocentric    = nh.advertise<sensor_msgs::PointCloud2>("/MapUpdater/debug/map_body", 100);
    pub_debug_map_arranged_init = nh.advertise<sensor_msgs::PointCloud2>("/MapUpdater/debug/map_init_arranged", 100);
    pub_init_inlier             = nh.advertise<sensor_msgs::PointCloud2>("/MapUpdater/debug/init_inlier", 100);
    pub_init_outlier            = nh.advertise<sensor_msgs::PointCloud2>("/MapUpdater/debug/init_outlier", 100);

    pub_dynamic_arranged = nh.advertise<sensor_msgs::PointCloud2>("/MapUpdater/dynamic", 100);
    pub_static_arranged  = nh.advertise<sensor_msgs::PointCloud2>("/MapUpdater/static", 100);

    query_voi.reserve(NUM_LARGE);
    map_voi.reserve(NUM_LARGE);
    map_outskirts_.reserve(NUM_LARGE);
    inliers_.reserve(NUM_LARGE);
    dynamicObjs_.reserve(NUM_LARGE);
    staticObjs_.reserve(NUM_LARGE);

    set_params();
    // Setting prebuilt map
    pcl::PointCloud<pcl::PointXYZI>::Ptr ptr_map(new pcl::PointCloud<pcl::PointXYZI>);
    int failure_flag = erasor_utils::load_pcd(map_name, ptr_map);
    if (failure_flag == -1){
        throw invalid_argument("Maybe intiial map path is not correct!");
    }
    std::cout << "Loading complete" << std::endl;
    num_pcs_init = ptr_map->points.size();
    map_init     = *ptr_map;

    // Erase the rooftop
    pcl::PointCloud<pcl::PointXYZI>::Ptr ptr_src(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr ptr_dst(new pcl::PointCloud<pcl::PointXYZI>);

    *ptr_src = map_init;
    if (environment == "outdoor") {
        map_arranged      = *ptr_src;
        map_arranged_init = map_arranged;

    } else if (environment == "indoor") {
        // ToDo It may works on large-scale map
        throw invalid_argument("Indoor mode is not perfect code!");
        pcl::PassThrough<pcl::PointXYZI> ptfilter;
        ptfilter.setInputCloud(ptr_src);
        ptfilter.setFilterFieldName("z");
        ptfilter.setFilterLimits(min_h, max_h);
        ptfilter.filter(*ptr_dst);
        map_arranged = *ptr_dst;

        map_arranged_init = map_arranged;

        ptfilter.setInputCloud(ptr_src);
        ptfilter.setFilterFieldName("z");
        ptfilter.setFilterLimits(min_h, max_h);
        ptfilter.setFilterLimitsNegative(true);
        ptfilter.filter(*ptr_dst);
        map_ceilings = *ptr_dst;
    }
    pub_map_init.publish(erasor_utils::cloud2msg(map_arranged_init));

    erasor.reset(new ERASOR(&nh));
}

void OnlineMapUpdater::set_params() {
    nh = ros::NodeHandle("~");
    // OnlineMapUpdater Parameters
    nh.param("/MapUpdater/query_voxel_size", query_voxel_size, 0.05);
    nh.param("/MapUpdater/map_voxel_size", map_voxel_size, 0.05);
    nh.param("/MapUpdater/voxelization_interval", global_voxelization_period, 10);
    nh.param("/MapUpdater/removal_interval", removal_interval, 2);
    nh.param<std::string>("/MapUpdater/data_name", data_name, "00");
    nh.param<std::string>("/MapUpdater/env", environment, "outdoor");
    nh.param<std::string>("/MapUpdater/initial_map_path", map_name, "/");
    nh.param<std::string>("/MapUpdater/save_path", save_path, "/");


    nh.param("/erasor/max_range", max_range, 60.0);
    nh.param("/erasor/max_h", max_h, 3.0);
    nh.param("/erasor/min_h", min_h, 0.0);
    nh.param("/erasor/version", erasor_version_, 3);

    nh.param("/verbose", verbose, true);
    std::cout << "Loading " << map_name << endl;
    std::cout << "Target env: " << environment << std::endl;
    std::cout << "\033[1;32m Version: \033[0m: " << erasor_version_ << std::endl;
    std::vector<double> lidar2body;

    if (nh.getParam("/tf/lidar2body", lidar2body)) {
        if (lidar2body.size() == 7) {
            geometry_msgs::Pose tmp_pose;
            tmp_pose.position.x    = lidar2body[0];
            tmp_pose.position.y    = lidar2body[1];
            tmp_pose.position.z    = lidar2body[2];
            tmp_pose.orientation.x = lidar2body[3];
            tmp_pose.orientation.y = lidar2body[4];
            tmp_pose.orientation.z = lidar2body[5];
            tmp_pose.orientation.w = lidar2body[6];
            Eigen::Matrix4f tmp_tf = Eigen::Matrix4f::Identity();
            tf_lidar2body = erasor_utils::geoPose2eigen(tmp_pose) * tmp_tf;
            std::cout << tf_lidar2body << std::endl;
            tmp_pose = erasor_utils::eigen2geoPose(tf_lidar2body);
//            std::cout<<tmp_pose.position.x<< ", "<<tmp_pose.position.y<< ", "<<tmp_pose.position.z<<std::endl;
        }
    }
}

OnlineMapUpdater::~OnlineMapUpdater() {
}

void OnlineMapUpdater::callback_flag(const std_msgs::Float32::ConstPtr &msg) {
    std::cout << "Flag comes!" << std::endl;
    // 1. Voxelization
    pcl::PointCloud<pcl::PointXYZI>::Ptr ptr_src(new pcl::PointCloud<pcl::PointXYZI>);
    *ptr_src = map_arranged;
    pcl::PointCloud<pcl::PointXYZI> mapOut;
    erasor_utils::voxelize_preserving_labels(ptr_src, mapOut, msg->data); // 0.05m is the criteria!
    // 2. Save the cloudmap
    mapOut.width  = mapOut.points.size();
    mapOut.height = 1;
    
    std::cout << "\033[1;32mTARGET: "<< save_path + "/" + data_name + "_result.pcd" << "\033[0m" << std::endl;
    std::cout << "Voxelization operated with " << msg->data << " voxel size" << std::endl;
    pcl::io::savePCDFileASCII(save_path + "/" + data_name + "_result.pcd", mapOut);
    std::cout << "\033[1;32mComplete to save the final static map\033[0m" << std::endl;
}

/**
 * @brief Callback function for node data
 */
void OnlineMapUpdater::callback_node(const erasor::node::ConstPtr &msg) {
    signal(SIGINT, erasor_utils::signal_callback_handler);

    static int stack_count = 0;
    stack_count++;

    if (stack_count % removal_interval == 0) {
        if (verbose) ROS_INFO_STREAM("\033[01;32m" << msg->header.seq << "th frame \033[0m is comming");


        pub_debug_map_arranged_init.publish(erasor_utils::cloud2msg(map_arranged_init));
        // Assumption: Coming nodes are equal to those that are used to build initial map
        tf_body2origin = erasor_utils::geoPose2eigen(msg->odom);
        // And there exists a premise that node is already somehow optimized
        set_path(path_corrected, "corrected", *msg, tf_body2origin);

        /**< 1. Set query pointcloud -> query VOI  */
        pcl::PointCloud<pcl::PointXYZI>::Ptr ptr_curr_raw(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PointCloud<pcl::PointXYZI>::Ptr ptr_curr_voxelized(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PointCloud<pcl::PointXYZI>::Ptr ptr_transformed(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PointCloud<pcl::PointXYZI>::Ptr ptr_curr_viz(new pcl::PointCloud<pcl::PointXYZI>);

        pcl::fromROSMsg(msg->lidar, *ptr_curr_raw);
        erasor_utils::voxelize_preserving_labels(ptr_curr_raw, *ptr_curr_voxelized, query_voxel_size);

        pcl::transformPointCloud(*ptr_curr_voxelized, *ptr_transformed, tf_lidar2body);
        query_voi = *ptr_transformed;
        body2origin(*ptr_curr_voxelized, *ptr_curr_viz);
        // - - - - - - - - - - - - - - - - - - - -

        /**< 2. Set map pointcloud -> map VOI  */
        double x_curr = tf_body2origin(0, 3);
        double y_curr = tf_body2origin(1, 3);

        auto start_tf = ros::Time::now().toSec();
        if (!map_voi.empty()) map_voi.clear();
        if (!map_outskirts_.empty()) map_outskirts_.clear();
        fetch_VoI(x_curr, y_curr, map_voi, map_outskirts_);
        auto end_tf = ros::Time::now().toSec();

        ROS_INFO_STREAM("\033[1;32mTf takes " << end_tf - start_tf << "s\033[0m");
        // ERASOR Procedure
        auto start  = ros::Time::now().toSec();

        ROS_INFO_STREAM(
                map_arranged.points.size() << " ->" << map_voi.points.size() << " vs " << query_voi.points.size());
        pub_debug_map_egocentric.publish(erasor_utils::cloud2msg(map_voi));
        pub_debug_query_egocentric.publish(erasor_utils::cloud2msg(query_voi));

        /**< Inputs should be transformed into egocentric frame >*/
        erasor->set_inputs(map_voi, query_voi);
        if (erasor_version_ == 2) {
            erasor->compare_vois_and_revert_ground(msg->header.seq);
            erasor->get_static_estimate(map_static_estimate, map_egocentric_complement);
        } else if (erasor_version_ == 3) {
            erasor->compare_vois_and_revert_ground_w_block(msg->header.seq);
            erasor->get_static_estimate(map_static_estimate, map_egocentric_complement);
        } else {
            throw invalid_argument("Other version is not implemented!");
        }
        auto end = ros::Time::now().toSec();
        erasor->get_outliers(tmp_map_rejected, tmp_curr_rejected);

        ROS_INFO_STREAM("\033[1;32mERASOR takes " << (end - start) << "s\033[0m");
        ROS_INFO_STREAM("ERASOR Input: \033[1;33m" << map_voi.points.size() << "\033[0m = ");
        ROS_INFO_STREAM(map_static_estimate.points.size() << " + " << map_egocentric_complement.points.size() << " - "
                                                          << tmp_map_rejected.points.size());
        ROS_INFO_STREAM(" = \033[1;33m" << map_static_estimate.points.size() + map_egocentric_complement.points.size() -
                                           tmp_map_rejected.points.size() << "\033[0m");

        map_filtered = map_static_estimate + map_egocentric_complement;

        body2origin(map_filtered, map_filtered);
        body2origin(tmp_map_rejected, tmp_map_rejected);
        body2origin(tmp_curr_rejected, tmp_curr_rejected);

        map_arranged = map_filtered + map_outskirts_;
        ROS_INFO_STREAM(map_arranged.points.size() << " " << map_filtered.points.size() << " \033[4;32m"
                                                   << map_outskirts_.points.size() << "\033[0m");

        map_rejected += tmp_map_rejected;
        curr_rejected += tmp_curr_rejected;

        if (stack_count % global_voxelization_period == 0) { // 1 indicates init voxelization
            pcl::PointCloud<pcl::PointXYZI>::Ptr ptr_src(new pcl::PointCloud<pcl::PointXYZI>);
            *ptr_src = map_arranged;
            auto num_origin = map_arranged.points.size();
            erasor_utils::voxelize_preserving_labels(ptr_src, map_arranged, map_voxel_size);
            map_arranged.width  = map_arranged.points.size();
            map_arranged.height = 1;
            ROS_INFO_STREAM("Global voxelization operated" << num_origin << " -> " << map_arranged.points.size());
        }
        pcl::PointCloud<pcl::PointXYZI>::Ptr map_viz(new pcl::PointCloud<pcl::PointXYZI>);
        if (environment == "outdoor") {
            *map_viz = map_arranged;
        } else if (environment == "indoor") {
            *map_viz = map_arranged + map_ceilings;
        }
        erasor_utils::parse_dynamic_obj(*map_viz, dynamicObjs_, staticObjs_);

        std::cout << "[Debug ] Total: " << map_viz->points.size() << std::endl;
        std::cout << "[Debug ] %: " << (double) dynamicObjs_.points.size() / num_pcs_init * 100 << " | "
                  << dynamicObjs_.points.size() << " / " << num_pcs_init << std::endl;
        std::cout << "[Debug ] %: " << (double) staticObjs_.points.size() / num_pcs_init * 100 << " | "
                  << staticObjs_.points.size() << " / " << num_pcs_init << std::endl;
        publish(*ptr_curr_viz, pub_debug_pc2_curr);
        publish(map_init, pub_map_init);
        publish(staticObjs_, pub_static_arranged);
        publish(dynamicObjs_, pub_dynamic_arranged);
        publish(map_rejected, pub_map_rejected);
        publish(curr_rejected, pub_curr_rejected);

        pub_path_corrected.publish(path_corrected);
    } else {
        std::cout << "\033[1;32m PASS! \033[0m" << std::endl;

    }
}

void OnlineMapUpdater::fetch_VoI(
        double x_criterion, double y_criterion, pcl::PointCloud<pcl::PointXYZI> &dst,
        pcl::PointCloud<pcl::PointXYZI> &outskirts, std::string mode) {
    // 1. Divide map_arranged into map_central and map_outskirts
    static double margin = 0;
    if (!dst.empty()) dst.clear();
    if (!outskirts.empty()) outskirts.clear();
    if (!inliers_.empty()) inliers_.clear(); // Inliers are still on the map frame

    if (mode == "naive") {
        double          max_dist_square = pow(max_range + margin, 2);
        for (auto const &pt : map_arranged.points) {
            double dist_square = pow(pt.x - x_criterion, 2) + pow(pt.y - y_criterion, 2);
            if (dist_square < max_dist_square) {
                inliers_.push_back(pt);
            } else {
                outskirts.push_back(pt);
            }
        }
    } else if (mode == "kdtree") {
        pcl::PointXYZI searchPoint;
        searchPoint.x = x_criterion;
        searchPoint.x = y_criterion;
        searchPoint.z = 0.5;
        std::cout << "\033[1;32mKDTREE mode " << map_arranged.points.size() << "\033[0m" << std::endl;
        std::vector<int>                     pointIdxRadiusSearch;
        std::vector<float>                   pointRadiusSquaredDistance;
        pcl::KdTreeFLANN<pcl::PointXYZI>     kdtree;
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
        *cloud = map_arranged;
        kdtree.setInputCloud(cloud);

        if (kdtree.radiusSearch(searchPoint, max_range + 0.5, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0) {
            // To get outlier
            std::vector<char> isTrue(map_arranged.points.size(), false);
            std::cout << "what?? " << pointIdxRadiusSearch.size();
            std::cout << "    " << isTrue.size();
            for (std::size_t i = 0; i < pointIdxRadiusSearch.size(); ++i) {
                auto pt = (*cloud)[pointIdxRadiusSearch[i]];
                inliers_.push_back(pt);
                isTrue[pointIdxRadiusSearch[i]] = true;
            }
            for (size_t      j = 0; j < map_arranged.points.size(); ++j) {
                if (!isTrue[j]) {
                    outskirts.push_back(map_arranged.points[j]);
                }
            }
        }
    }
    ROS_INFO_STREAM(map_arranged.points.size() << "=" << inliers_.points.size() + outskirts.points.size() << "| "
                                               << inliers_.points.size() << " + \033[4;32m" << outskirts.points.size()
                                               << "\033[0m");

    pcl::PointCloud<pcl::PointXYZI>::Ptr ptr_transformed(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::transformPointCloud(inliers_, *ptr_transformed, tf_body2origin.inverse());
    dst = *ptr_transformed;
}


void OnlineMapUpdater::body2origin(
        const pcl::PointCloud<pcl::PointXYZI> src,
        pcl::PointCloud<pcl::PointXYZI> &dst) {
    pcl::PointCloud<pcl::PointXYZI>::Ptr ptr_src(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr ptr_transformed(new pcl::PointCloud<pcl::PointXYZI>);
    *ptr_src = src;
    pcl::transformPointCloud(*ptr_src, *ptr_transformed, tf_body2origin);
    dst = *ptr_transformed;
}


void OnlineMapUpdater::set_path(
        nav_msgs::Path &path, std::string mode,
        const erasor::node &node, const Eigen::Matrix4f &body2mapprev) {
    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header          = node.header;
    pose_stamped.header.frame_id = "/map";
    pose_stamped.pose            = erasor_utils::eigen2geoPose(body2mapprev);

    path.header = pose_stamped.header;
    path.poses.push_back(pose_stamped);
}

void OnlineMapUpdater::publish(
        const sensor_msgs::PointCloud2 &map,
        const ros::Publisher &publisher) {
    pc2_map.header.frame_id = "/map";
    publisher.publish(map);
    if (verbose) ROS_INFO_STREAM("PC2 is Published!");

}

void OnlineMapUpdater::publish(
        const pcl::PointCloud<pcl::PointXYZI> &map,
        const ros::Publisher &publisher) {
    pcl::toROSMsg(map, pc2_map);
    pc2_map.header.frame_id = "/map";
    publisher.publish(pc2_map);
}
