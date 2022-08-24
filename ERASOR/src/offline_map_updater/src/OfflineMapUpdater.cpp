#include "erasor/OfflineMapUpdater.h"

using namespace erasor;

OfflineMapUpdater::OfflineMapUpdater() {
    sub_node_ = nh.subscribe<de_msg::node>("/deepmerge/map/3d/current", 50000, &OfflineMapUpdater::callback_node, this);
    sub_flag_ = nh.subscribe<std_msgs::Float32>("/saveflag", 10, &OfflineMapUpdater::callback_flag, this);

    pub_path_ = nh.advertise<nav_msgs::Path>("/MapUpdater/path_corrected", 100, true);
    pub_path_of_all_nodes_ = nh.advertise<nav_msgs::Path>("/MapUpdater/path_of_all_nodes", 100, true);

    pub_map_init_      = nh.advertise<sensor_msgs::PointCloud2>("/MapUpdater/map_init", 100, true);
    pub_map_rejected_  = nh.advertise<sensor_msgs::PointCloud2>("/MapUpdater/map_rejected", 100, true);
    pub_curr_rejected_ = nh.advertise<sensor_msgs::PointCloud2>("/MapUpdater/curr_rejected", 100, true);

    pub_debug_pc2_curr_ = nh.advertise<sensor_msgs::PointCloud2>("/MapUpdater/pc2_curr", 100, true);

    pub_debug_map_               = nh.advertise<sensor_msgs::PointCloud2>("/MapUpdater/debug/map", 100, true);
    pub_debug_query_egocentric_  = nh.advertise<sensor_msgs::PointCloud2>("/MapUpdater/debug/pc_curr_body", 100, true);
    pub_debug_map_egocentric_    = nh.advertise<sensor_msgs::PointCloud2>("/MapUpdater/debug/map_body", 100, true);
    pub_debug_map_arranged_init_ = nh.advertise<sensor_msgs::PointCloud2>("/MapUpdater/debug/map_init_arranged", 100, true);

    pub_dynamic_arranged_ = nh.advertise<sensor_msgs::PointCloud2>("/MapUpdater/est_dynamic_obj", 100, true);

    initialize_ptrs();

    set_params();

    erasor_.reset(new ERASOR(&nh));
}

OfflineMapUpdater::~OfflineMapUpdater() {
}

void OfflineMapUpdater::initialize_ptrs() {
    map_init_.reset(new pcl::PointCloud<pcl::PointXYZI>());
    map_arranged_.reset(new pcl::PointCloud<pcl::PointXYZI>());
    map_arranged_init_.reset(new pcl::PointCloud<pcl::PointXYZI>());
    map_arranged_global_.reset(new pcl::PointCloud<pcl::PointXYZI>());
    map_arranged_complement_.reset(new pcl::PointCloud<pcl::PointXYZI>());
    map_ceilings_.reset(new pcl::PointCloud<pcl::PointXYZI>());

    query_voi_.reset(new pcl::PointCloud<pcl::PointXYZI>());
    map_voi_.reset(new pcl::PointCloud<pcl::PointXYZI>());
    map_voi_wrt_origin_.reset(new pcl::PointCloud<pcl::PointXYZI>());
    map_outskirts_.reset(new pcl::PointCloud<pcl::PointXYZI>());

    map_static_estimate_.reset(new pcl::PointCloud<pcl::PointXYZI>());
    map_egocentric_complement_.reset(new pcl::PointCloud<pcl::PointXYZI>());
    map_filtered_.reset(new pcl::PointCloud<pcl::PointXYZI>());

    query_rejected_.reset(new pcl::PointCloud<pcl::PointXYZI>());
    map_rejected_.reset(new pcl::PointCloud<pcl::PointXYZI>());
    total_map_rejected_.reset(new pcl::PointCloud<pcl::PointXYZI>());

    map_init_->reserve(NUM_PTS_LARGE_ENOUGH_FOR_MAP);
    map_arranged_->reserve(NUM_PTS_LARGE_ENOUGH_FOR_MAP);
    map_arranged_init_->reserve(NUM_PTS_LARGE_ENOUGH_FOR_MAP);
    map_arranged_global_->reserve(NUM_PTS_LARGE_ENOUGH_FOR_MAP);
}

void OfflineMapUpdater::set_params() {
    nh = ros::NodeHandle("~");
    // OfflineMapUpdater Parameters
    nh.param("/MapUpdater/query_voxel_size", query_voxel_size_, 0.05);
    nh.param("/MapUpdater/map_voxel_size", map_voxel_size_, 0.05);
    nh.param("/MapUpdater/voxelization_interval", global_voxelization_period_, 10);
    nh.param("/MapUpdater/removal_interval", removal_interval_, 2);
    nh.param<std::string>("/MapUpdater/data_name", data_name_, "00");
    nh.param<std::string>("/MapUpdater/env", environment_, "outdoor");
    nh.param<std::string>("/MapUpdater/initial_map_path", map_name_, "/");
    nh.param<std::string>("/MapUpdater/save_path", save_path_, "/");

    nh.param<bool>("/large_scale/is_large_scale", is_large_scale_, false);
    nh.param("/large_scale/submap_size", submap_size_, 200.0);

    nh.param("/erasor/max_range", max_range_, 60.0);
    nh.param("/erasor/max_h", max_h_, 3.0);
    nh.param("/erasor/min_h", min_h_, 0.0);
    nh.param("/erasor/version", erasor_version_, 3);

    nh.param("/deep_express/trans_diff_thr", trans_diff_thr_, 1.5); // unit: m
    nh.param("/deep_express/rot_diff_thr", rot_diff_thr_, 15.0); // unit: deg.
    nh.param<double>("/deep_express/output_map_voxel_size", output_map_voxel_size_, 0.2);
    nh.param<int>("/deep_express/sliding_window_size", sliding_window_size_, 30);
    nh.param<int>("/deep_express/save_duration", save_duration_, 200);

    save_timer_ = nh.createTimer(ros::Duration(save_duration_), &OfflineMapUpdater::save_timer, this);

    nh.param("/verbose", verbose_, true);
    if (save_path_.length() == 0) {
        throw invalid_argument("Save path seems to be wrong! It should be end with '~.pcd'");
    } else {
        if (save_path_.substr(save_path_.length() - 3) != "pcd") {
            std::cout << "\033[1;31m" << save_path_ << std::endl;
            std::cout << save_path_ << std::endl;
            std::cout << save_path_ << "\033[0m" << std::endl;
            throw invalid_argument("Save path seems to be wrong! It should be end with '~.pcd'");
        }
    }
    std::cout << "\033[1;32mSAVE PATH: " << save_path_ << "\033[0m"<< std::endl;
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
            tf_lidar2body_ = erasor_utils::geoPose2eigen(tmp_pose) * tmp_tf;
            std::cout << tf_lidar2body_ << std::endl;
            tmp_pose = erasor_utils::eigen2geoPose(tf_lidar2body_);
        }
    }
}

void OfflineMapUpdater::callback_flag(const std_msgs::Float32::ConstPtr &msg) {
    std::cout << "Flag comes!" << std::endl;
    if (!map_arranged_->points.empty()) {
        save_static_map(msg->data);
    }
}

void OfflineMapUpdater::save_static_map(float voxel_size) {
    // 1. Voxelization
    pcl::PointCloud<pcl::PointXYZI>::Ptr ptr_src(new pcl::PointCloud<pcl::PointXYZI>);
    int num_pts_total = map_arranged_->points.size() + map_arranged_complement_->points.size();
    ptr_src->reserve(num_pts_total);

    if (is_large_scale_) {
        std::cout << "Merging submap and complements..." << std::endl;
        *ptr_src = *map_arranged_ + *map_arranged_complement_;
    } else {
        *ptr_src = *map_arranged_;
    }
    pcl::PointCloud<pcl::PointXYZI> map_to_be_saved;
    static pcl::VoxelGrid<pcl::PointXYZI> voxel_filter;
    voxel_filter.setInputCloud(ptr_src);
    voxel_filter.setLeafSize(voxel_size, voxel_size, voxel_size);
    voxel_filter.filter(map_to_be_saved);

    // 2. Save the cloudmap
    map_to_be_saved.width  = map_to_be_saved.points.size();
    map_to_be_saved.height = 1;

    std::cout << "\033[1;32mTARGET: " << save_path_ << "\033[0m" << std::endl;
    std::cout << "Voxelization operated with " << voxel_size << " voxel size" << std::endl;
    pcl::io::savePCDFileASCII(save_path_, map_to_be_saved);
    std::cout << "\033[1;32mComplete to save the final static map\033[0m" << std::endl;
}

void OfflineMapUpdater::save_timer(const ros::TimerEvent& event) {
    if (!map_arranged_->points.empty()) {
        save_static_map(output_map_voxel_size_);
    }
}

/**
 * @brief Callback function for node data
 */
void OfflineMapUpdater::callback_node(const de_msg::node::ConstPtr &msg) {
    signal(SIGINT, erasor_utils::signal_callback_handler);

    if (nodes_.empty()) {
        nodes_.push_back(*msg);
        return;
    }
    de_msg::node node_curr = *msg;
    Eigen::Matrix4f curr4x4 = erasor_utils::geoPose2eigen(node_curr.odom);
    Eigen::Matrix4f diff4x4 = erasor_utils::geoPose2eigen(nodes_.back().odom).inverse() * curr4x4;
    Eigen::Matrix3d rel_rot = diff4x4.block<3, 3>(0, 0).cast<double>();
    std::vector<double>  rel_rpy = erasor_utils::SO3ToEuler(rel_rot, "deg");

    float diff_trans  = sqrt(pow(diff4x4(0, 3),2) + pow(diff4x4(1, 3),2));
    float diff_angle = fabs(rel_rpy[2]);

    set_path(path_of_all_nodes_,  *msg, curr4x4);
    pub_path_of_all_nodes_.publish(path_of_all_nodes_);

    if((diff_trans < trans_diff_thr_) && (diff_angle < rot_diff_thr_))
    {
        ROS_INFO_STREAM("\033[1;32mNot enough movement! \033[0m");
        return; // Not enough interval
    }else{
        static long int count_frame = 0;
        if (verbose_) ROS_INFO_STREAM("\033[01;32m" << count_frame << "th frame\033[0m is coming");

        /*** 1. Accumulate submap (raw point cloud map) before */
        nodes_.push_back(*msg);
        nodes_.back().idx.data = count_frame++;
        pcl::PointCloud<pcl::PointXYZI>::Ptr ptr_query(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PointCloud<pcl::PointXYZI>::Ptr ptr_query_voxel(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PointCloud<pcl::PointXYZI>::Ptr ptr_query_body(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PointCloud<pcl::PointXYZI>::Ptr ptr_query_viz(new pcl::PointCloud<pcl::PointXYZI>);

        ptr_query->reserve(NUM_PTS_LARGE_ENOUGH);
        ptr_query_voxel->reserve(NUM_PTS_LARGE_ENOUGH);
        ptr_query_body->reserve(NUM_PTS_LARGE_ENOUGH);
        ptr_query_viz->reserve(NUM_PTS_LARGE_ENOUGH);

        pcl::fromROSMsg(msg->laser, *ptr_query);
        erasor_utils::voxelize_preserving_labels(ptr_query, *ptr_query_voxel, query_voxel_size_);
        Eigen::Matrix4f tf_body2origin = erasor_utils::geoPose2eigen(msg->odom);
        // In last mile project, tf_lidar2body_ is identity matrix
        pcl::transformPointCloud(*ptr_query_voxel, *ptr_query_body, tf_lidar2body_ * curr4x4);
        *map_arranged_ += *ptr_query_body;

        /*** 2. Once the sufficient nodes are not collected, ERASOR is not performed */
        if (nodes_.size() < sliding_window_size_) {
            return;
        }
        *query_voi_ = *ptr_query_body;

        Eigen::Matrix4f tf_midpoint = erasor_utils::geoPose2eigen(nodes_[sliding_window_size_ / 2].odom);
        double x_criteria = tf_midpoint(0, 3);
        double y_criteria = tf_midpoint(1, 3);

        if (is_large_scale_) {
            reassign_submap(x_criteria, y_criteria);
        }

        /**< 3. Static map building started */
        for (const auto& target_node: nodes_) {
            tf_body2origin_ = erasor_utils::geoPose2eigen(target_node.odom);
            pcl::fromROSMsg(target_node.laser, *ptr_query);
            erasor_utils::voxelize_preserving_labels(ptr_query, *ptr_query_voxel, query_voxel_size_);
            pcl::transformPointCloud(*ptr_query_voxel, *ptr_query_body, tf_lidar2body_);
            *query_voi_ = *ptr_query_body;

            double x_criteria = tf_body2origin_(0, 3);
            double y_criteria = tf_body2origin_(1, 3);
            fetch_VoI(x_criteria, y_criteria, *map_voi_, *map_outskirts_);

            pub_debug_map_egocentric_.publish(erasor_utils::cloud2msg(*map_voi_));
            pub_debug_query_egocentric_.publish(erasor_utils::cloud2msg(*query_voi_));

            /*** 4. Conduct Scan Ratio Test & set map_static_estimate and map_egocentric_complement
             * Note that inputs should be previously transformed into egocentric frame */
            auto start = ros::Time::now().toSec();
            erasor_->set_inputs(*map_voi_, *query_voi_);
            if (erasor_version_ == 2) {
                erasor_->compare_vois_and_revert_ground(msg->header.seq);
                erasor_->get_static_estimate(*map_static_estimate_, *map_egocentric_complement_);
            } else if (erasor_version_ == 3) {
                erasor_->compare_vois_and_revert_ground_w_block(msg->header.seq);
                erasor_->get_static_estimate(*map_static_estimate_, *map_egocentric_complement_);
            } else {
                throw invalid_argument("Other version is not implemented!");
            }
            auto middle = ros::Time::now().toSec();

            ROS_INFO_STREAM("\033[1;32m" << setw(22) << "ERASOR takes " << middle - start << "s\033[0m");

            *map_filtered_ = *map_static_estimate_ + *map_egocentric_complement_;

            /*** Get currently rejected points.
             * Since these are w.r.t. egocentric frame, so these are transformed into the world frame */
            erasor_->get_outliers(*map_rejected_, *query_rejected_);
            body2origin(*map_filtered_, *map_filtered_);
            body2origin(*map_rejected_, *map_rejected_);
            body2origin(*query_rejected_, *query_rejected_);

            *map_arranged_ = *map_filtered_ + *map_outskirts_;

            /*** Just for debugging */
            *total_map_rejected_ += *map_rejected_;

            if (environment_ != "outdoor") { throw invalid_argument("Other modes are not supported"); }

            publish(*ptr_query_viz, pub_debug_pc2_curr_);
            publish(*total_map_rejected_, pub_dynamic_arranged_);
            publish(*map_rejected_, pub_map_rejected_);
            publish(*query_rejected_, pub_curr_rejected_);

            if (target_node.idx.data > path_.poses.size() ) {
                /*** Just for visualization due to overlapped scenes */
                set_path(path_, target_node, tf_body2origin_);
                pub_path_.publish(path_);
            }
        }

        for (int ll = 0; ll < sliding_window_size_/2; ++ll) {
            nodes_.pop_front();
        }

        publish(*map_arranged_, pub_map_init_);
    }
}

void OfflineMapUpdater::reassign_submap(double pose_x, double pose_y){
    if (is_submap_not_initialized_) {
        submap_center_x_ = pose_x;
        submap_center_y_ = pose_y;
        is_submap_not_initialized_ = false;

        ROS_INFO_STREAM("\033[1;32mComplete to initialize submap!\033[0m");
        ROS_INFO_STREAM(map_arranged_global_->points.size() <<" to " << map_arranged_->points.size() <<" | " <<map_arranged_complement_->points.size());
    } else {
        double diff_x = abs(submap_center_x_ - pose_x);
        double diff_y = abs(submap_center_y_ - pose_y);
        static double half_size = submap_size_ / 2.0;
        if ( (diff_x > half_size) ||  (diff_y > half_size) ) {
            // Reassign submap
            int num_pts_total = map_arranged_->points.size() + map_arranged_complement_->points.size();
            map_arranged_global_.reset(new pcl::PointCloud<pcl::PointXYZI>());
            map_arranged_global_->reserve(num_pts_total);
            *map_arranged_global_ = *map_arranged_ + *map_arranged_complement_;
            set_submap(*map_arranged_global_, *map_arranged_, *map_arranged_complement_, pose_x, pose_y, submap_size_);
            submap_center_x_ = pose_x;
            submap_center_y_ = pose_y;
            ROS_INFO_STREAM("\033[1;32mComplete to initialize submap!\033[0m");
            ROS_INFO_STREAM(map_arranged_global_->points.size() <<" to " << map_arranged_->points.size() <<" | " <<map_arranged_complement_->points.size());
        }
    }
}

void OfflineMapUpdater::set_submap(
        const pcl::PointCloud<pcl::PointXYZI> &map_global, pcl::PointCloud<pcl::PointXYZI>& submap,
        pcl::PointCloud<pcl::PointXYZI>& submap_complement,
        double x, double y, double submap_size) {

    submap.clear();
    submap.reserve(NUM_PTS_LARGE_ENOUGH_FOR_MAP);
    submap_complement.clear();
    submap_complement.reserve(NUM_PTS_LARGE_ENOUGH_FOR_MAP);

    for (const auto pt: map_global.points) {
        double diff_x = fabs(x - pt.x);
        double diff_y = fabs(y - pt.y);
        if ((diff_x < submap_size) && (diff_y < submap_size)) {
            submap.points.emplace_back(pt);
        } else {
            submap_complement.points.emplace_back(pt);
        }
    }
}

void OfflineMapUpdater::fetch_VoI(
        double x_criterion, double y_criterion, pcl::PointCloud<pcl::PointXYZI> &dst,
        pcl::PointCloud<pcl::PointXYZI> &outskirts, std::string mode) {
    // 1. Divide map_arranged into map_central and map_outskirts
    static double margin = 0;
    if (!dst.empty()) dst.clear();
    if (!outskirts.empty()) outskirts.clear();
    if (!map_voi_wrt_origin_->points.empty()) map_voi_wrt_origin_->points.clear(); // Inliers are still on the map frame

    if (mode == "naive") {
        double max_dist_square = pow(max_range_ + margin, 2);

        for (auto const &pt : (*map_arranged_).points) {
            double dist_square = pow(pt.x - x_criterion, 2) + pow(pt.y - y_criterion, 2);
            if (dist_square < max_dist_square) {
                map_voi_wrt_origin_->points.emplace_back(pt);
            } else {
                outskirts.points.emplace_back(pt);
            }
        }
    } else if (mode == "kdtree") {
        pcl::PointXYZI searchPoint;
        searchPoint.x = x_criterion;
        searchPoint.x = y_criterion;
        searchPoint.z = 0.5;
        std::cout << "\033[1;32mKDTREE mode " << (*map_arranged_).points.size() << "\033[0m" << std::endl;
        std::vector<int>                     pointIdxRadiusSearch;
        std::vector<float>                   pointRadiusSquaredDistance;
        pcl::KdTreeFLANN<pcl::PointXYZI>     kdtree;
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
        *cloud = *map_arranged_;
        kdtree.setInputCloud(cloud);

        if (kdtree.radiusSearch(searchPoint, max_range_ + 0.5, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0) {
            // To get outlier
            std::vector<char> isTrue(map_arranged_->points.size(), false);
            std::cout << "what?? " << pointIdxRadiusSearch.size();
            std::cout << "    " << isTrue.size();
            for (std::size_t i = 0; i < pointIdxRadiusSearch.size(); ++i) {
                auto pt = (*cloud)[pointIdxRadiusSearch[i]];
                map_voi_wrt_origin_->points.emplace_back(pt);
                isTrue[pointIdxRadiusSearch[i]] = true;
            }
            for (size_t      j = 0; j < map_arranged_->points.size(); ++j) {
                if (!isTrue[j]) {
                    outskirts.push_back(map_arranged_->points[j]);
                }
            }
        }
    }
    ROS_INFO_STREAM(map_arranged_->points.size() << "=" << map_voi_wrt_origin_->points.size() + outskirts.points.size() << "| "
                                                 << map_voi_wrt_origin_->points.size() << " + \033[4;32m" << outskirts.points.size()
                                                 << "\033[0m");

    pcl::PointCloud<pcl::PointXYZI>::Ptr ptr_transformed(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::transformPointCloud(*map_voi_wrt_origin_, *ptr_transformed, tf_body2origin_.inverse());
    dst = *ptr_transformed;
}


void OfflineMapUpdater::body2origin(
        const pcl::PointCloud<pcl::PointXYZI> src,
        pcl::PointCloud<pcl::PointXYZI> &dst) {
    pcl::PointCloud<pcl::PointXYZI>::Ptr ptr_src(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr ptr_transformed(new pcl::PointCloud<pcl::PointXYZI>);
    *ptr_src = src;
    pcl::transformPointCloud(*ptr_src, *ptr_transformed, tf_body2origin_);
    dst = *ptr_transformed;
}

void OfflineMapUpdater::set_path(
        nav_msgs::Path &path, const de_msg::node &node, const Eigen::Matrix4f &body2mapprev) {
    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header          = node.header;
    pose_stamped.header.frame_id = "/map";
    pose_stamped.pose            = erasor_utils::eigen2geoPose(body2mapprev);

    path.header = pose_stamped.header;
    path.poses.push_back(pose_stamped);
}

void OfflineMapUpdater::publish(
        const sensor_msgs::PointCloud2 &map,
        const ros::Publisher &publisher) {
    pc2_map_.header.frame_id = "/map";
    publisher.publish(map);
    if (verbose_) ROS_INFO_STREAM("PC2 is Published!");
}

void OfflineMapUpdater::publish(
        const pcl::PointCloud<pcl::PointXYZI> &map,
        const ros::Publisher &publisher) {
    pcl::toROSMsg(map, pc2_map_);
    pc2_map_.header.frame_id = "/map";
    publisher.publish(pc2_map_);
}
