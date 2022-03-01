#include "erasor/OfflineMapUpdater.h"

using namespace erasor;

OfflineMapUpdater::OfflineMapUpdater() {
    sub_node_ = nh.subscribe<erasor::node>("/node/combined/optimized", 2000, &OfflineMapUpdater::callback_node, this);
    sub_flag_ = nh.subscribe<std_msgs::Float32>("/saveflag", 10, &OfflineMapUpdater::callback_flag, this);

    pub_path_ = nh.advertise<nav_msgs::Path>("/MapUpdater/path_corrected", 100);

    pub_map_init_      = nh.advertise<sensor_msgs::PointCloud2>("/MapUpdater/map_init", 100);
    pub_map_rejected_  = nh.advertise<sensor_msgs::PointCloud2>("/MapUpdater/map_rejected", 100);
    pub_curr_rejected_ = nh.advertise<sensor_msgs::PointCloud2>("/MapUpdater/curr_rejected", 100);

    pub_debug_pc2_curr_ = nh.advertise<sensor_msgs::PointCloud2>("/MapUpdater/pc2_curr", 100);

    pub_debug_map_               = nh.advertise<sensor_msgs::PointCloud2>("/MapUpdater/debug/map", 100);
    pub_debug_query_egocentric_  = nh.advertise<sensor_msgs::PointCloud2>("/MapUpdater/debug/pc_curr_body", 100);
    pub_debug_map_egocentric_    = nh.advertise<sensor_msgs::PointCloud2>("/MapUpdater/debug/map_body", 100);
    pub_debug_map_arranged_init_ = nh.advertise<sensor_msgs::PointCloud2>("/MapUpdater/debug/map_init_arranged", 100);

    pub_dynamic_arranged_ = nh.advertise<sensor_msgs::PointCloud2>("/MapUpdater/dynamic", 100);
    pub_static_arranged_  = nh.advertise<sensor_msgs::PointCloud2>("/MapUpdater/static", 100);

    initialize_ptrs();

    set_params();

    load_global_map();

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
    total_query_rejected_.reset(new pcl::PointCloud<pcl::PointXYZI>());
    total_map_rejected_.reset(new pcl::PointCloud<pcl::PointXYZI>());

    dynamic_objs_to_viz_.reset(new pcl::PointCloud<pcl::PointXYZI>());
    static_objs_to_viz_.reset(new pcl::PointCloud<pcl::PointXYZI>());
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

    nh.param("/verbose", verbose_, true);
    std::cout << "Loading " << map_name_ << endl;
    std::cout << "Target env: " << environment_ << std::endl;
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
            tf_lidar2body_ = erasor_utils::geoPose2eigen(tmp_pose) * tmp_tf;
            std::cout << tf_lidar2body_ << std::endl;
            tmp_pose = erasor_utils::eigen2geoPose(tf_lidar2body_);
        }
    }
}

void OfflineMapUpdater::load_global_map() {
    /***
     * map_init         : Raw initial map
     * map_arranged_init: Initial map to be compared with map arranged
     *                    In case of indoor env, map_arranged_init = map_init - ceilings
     *                    In case of outdoor env, map_arranged_init = map_init
     * map_arranged     : Target cloud to be filtered via ERASOR
     */

    cout<<"[MapUpdater] On loading naively accumulated map...it takes few seconds..."<<endl;
    map_init_->reserve(NUM_PTS_LARGE_ENOUGH_FOR_MAP);
    map_arranged_->reserve(NUM_PTS_LARGE_ENOUGH_FOR_MAP);
    map_arranged_init_->reserve(NUM_PTS_LARGE_ENOUGH_FOR_MAP);
    map_arranged_global_->reserve(NUM_PTS_LARGE_ENOUGH_FOR_MAP);

    int failure_flag = erasor_utils::load_pcd(map_name_, map_init_);

    if (failure_flag == -1) {
        throw invalid_argument("Maybe intiial map path is not correct!");
    } else {
        std::cout << "Loading global map complete!" << std::endl;
    }

    num_pcs_init_ = map_init_->points.size();

    if (environment_ == "outdoor") {
        *map_arranged_      = *map_init_;
        *map_arranged_init_ = *map_arranged_;
        if (is_large_scale_) {
            /*** In case of large-scale static map building,
             * `map_arranged_` is used as submap
             * Thereafter, `map_arranged_` is updated to the `map_arranged_global_`
             */
            *map_arranged_global_ = *map_arranged_;
            std::cout << "Large-scale mode is on!" << std::endl;
            std::cout << "Submap size is: " << submap_size_ << std::endl;
        }

    } else if (environment_ == "indoor") {
        // Erase the rooftop in case of indoor environments
        pcl::PointCloud<pcl::PointXYZI>::Ptr ptr_dst(new pcl::PointCloud<pcl::PointXYZI>);
        // ToDo It may not work on the large-scale map or env which includes slope regions
        throw invalid_argument("This `indoor` mode is not perfect!");
        pcl::PassThrough<pcl::PointXYZI> ptfilter;
        ptfilter.setInputCloud(map_init_);
        ptfilter.setFilterFieldName("z");
        ptfilter.setFilterLimits(min_h_, max_h_);
        ptfilter.filter(*ptr_dst);

        *map_arranged_init_ = *map_init_;
        *map_arranged_      = *map_arranged_init_;

        ptfilter.setFilterLimitsNegative(true);
        ptfilter.filter(*ptr_dst);
        *map_ceilings_ = *ptr_dst;
    }
    if (!is_large_scale_) {
        // In case of large scale, it takes too time...
        pub_map_init_.publish(erasor_utils::cloud2msg(*map_arranged_init_));
    }
}

void OfflineMapUpdater::callback_flag(const std_msgs::Float32::ConstPtr &msg) {
    std::cout << "Flag comes!" << std::endl;
    save_static_map(msg->data);
}

void OfflineMapUpdater::save_static_map(float voxel_size) {
    // 1. Voxelization
    pcl::PointCloud<pcl::PointXYZI>::Ptr ptr_src(new pcl::PointCloud<pcl::PointXYZI>);
    ptr_src->reserve(num_pcs_init_);

    if (is_large_scale_) {
        std::cout << "Merging submap and complements..." << std::endl;
        *ptr_src = *map_arranged_ + *map_arranged_complement_;
    } else {
        *ptr_src = *map_arranged_;
    }
    pcl::PointCloud<pcl::PointXYZI> map_to_be_saved;
    erasor_utils::voxelize_preserving_labels(ptr_src, map_to_be_saved, voxel_size); // 0.05m is the criteria!
    // 2. Save the cloudmap
    map_to_be_saved.width  = map_to_be_saved.points.size();
    map_to_be_saved.height = 1;

    std::cout << "\033[1;32mTARGET: " << save_path_ + "/" + data_name_ + "_result.pcd" << "\033[0m" << std::endl;
    std::cout << "Voxelization operated with " << voxel_size << " voxel size" << std::endl;
    pcl::io::savePCDFileASCII(save_path_ + "/" + data_name_ + "_result.pcd", map_to_be_saved);
    std::cout << "\033[1;32mComplete to save the final static map\033[0m" << std::endl;

}



/**
 * @brief Callback function for node data
 */
void OfflineMapUpdater::callback_node(const erasor::node::ConstPtr &msg) {
    signal(SIGINT, erasor_utils::signal_callback_handler);

    static int stack_count = 0;
    stack_count++;

    if (stack_count % removal_interval_ == 0) {
        if (verbose_) ROS_INFO_STREAM("\033[01;32m" << msg->header.seq << "th frame\033[0m is comming");

        if (!is_large_scale_) {
            pub_debug_map_arranged_init_.publish(erasor_utils::cloud2msg(*map_arranged_init_));
        }
        /***
         * Assumption: Coming nodes are equal to those that are used to build initial map
         * And there exists a premise that node is already somehow optimized
         */
        tf_body2origin_ = erasor_utils::geoPose2eigen(msg->odom);
        set_path(path_, "corrected", *msg, tf_body2origin_);

        /**<
         * 1. Set query pointcloud -> query VOI
         *    Note that query is on lidar frame.
         *    So the coordinate is tranformed from lidar coord. to body coord. (in fact, it depends on your own env.)
         * */
        pcl::PointCloud<pcl::PointXYZI>::Ptr ptr_query(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PointCloud<pcl::PointXYZI>::Ptr ptr_query_voxel(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PointCloud<pcl::PointXYZI>::Ptr ptr_query_body(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PointCloud<pcl::PointXYZI>::Ptr ptr_query_viz(new pcl::PointCloud<pcl::PointXYZI>);

        ptr_query->reserve(NUM_PTS_LARGE_ENOUGH);
        ptr_query_voxel->reserve(NUM_PTS_LARGE_ENOUGH);
        ptr_query_body->reserve(NUM_PTS_LARGE_ENOUGH);
        ptr_query_viz->reserve(NUM_PTS_LARGE_ENOUGH);

        pcl::fromROSMsg(msg->lidar, *ptr_query);
        erasor_utils::voxelize_preserving_labels(ptr_query, *ptr_query_voxel, query_voxel_size_);

        pcl::transformPointCloud(*ptr_query_voxel, *ptr_query_body, tf_lidar2body_);
        *query_voi_ = *ptr_query_body;
        body2origin(*ptr_query_body, *ptr_query_viz);
        // - - - - - - - - - - - - - - - - - - - -

        /**< 2. Set map pointcloud -> map VOI  */
        double x_curr = tf_body2origin_(0, 3);
        double y_curr = tf_body2origin_(1, 3);

        if (is_large_scale_) {
            reassign_submap(x_curr, y_curr);
        }

        auto start_voi = ros::Time::now().toSec();
        fetch_VoI(x_curr, y_curr, *map_voi_, *map_outskirts_);
        auto end_voi = ros::Time::now().toSec();

        ROS_INFO_STREAM("\033[1;32m" << setw(22) << "Extracting VoI takes " << end_voi - start_voi << "s\033[0m");

        pub_debug_map_egocentric_.publish(erasor_utils::cloud2msg(*map_voi_));
        pub_debug_query_egocentric_.publish(erasor_utils::cloud2msg(*query_voi_));

        /**< 3. Conduct Scan Ratio Test & set map_static_estimate and map_egocentric_complement
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

        /*** Get currently rejected pts */
        erasor_->get_outliers(*map_rejected_, *query_rejected_);

        body2origin(*map_filtered_, *map_filtered_);
        body2origin(*map_rejected_, *map_rejected_);
        body2origin(*query_rejected_, *query_rejected_);

        *map_arranged_ = *map_filtered_ + *map_outskirts_;

        auto end = ros::Time::now().toSec();

        erasor_utils::parse_dynamic_obj(*map_arranged_, *dynamic_objs_to_viz_, *static_objs_to_viz_);

        /*** Just for debugging */
        *total_map_rejected_ += *map_rejected_;
        *total_query_rejected_ += *query_rejected_;

//        if (stack_count % global_voxelization_period_ == 0) { // 1 indicates init voxelization
//            pcl::PointCloud<pcl::PointXYZI>::Ptr ptr_src(new pcl::PointCloud<pcl::PointXYZI>);
//            *ptr_src = *map_arranged_;
//            auto num_origin  = map_arranged_->points.size();
//            auto start_voxel = ros::Time::now().toSec();
//            erasor_utils::voxelize_preserving_labels(ptr_src, *map_arranged_, map_voxel_size_);
//            auto end_voxel = ros::Time::now().toSec();
//            map_arranged_->width  = map_arranged_->points.size();
//            map_arranged_->height = 1;
////            ROS_INFO_STREAM("Global voxelization operated: " << setw(10) << num_origin << " -> " << map_arranged_->points.size());
//            ROS_INFO_STREAM(setw(22) <<"Global voxel. takes " << end_voxel - start_voxel << "s");
//        }
        if (environment_ != "outdoor") { throw invalid_argument("Other modes are not supported"); }

        print_status();

        publish(*ptr_query_viz, pub_debug_pc2_curr_);
        publish(*static_objs_to_viz_, pub_static_arranged_);
        publish(*dynamic_objs_to_viz_, pub_dynamic_arranged_);
        publish(*map_rejected_, pub_map_rejected_);
        publish(*query_rejected_, pub_curr_rejected_);

        if (!is_large_scale_) {
            publish(*map_init_, pub_map_init_);
        }

        pub_path_.publish(path_);
    } else {
        ROS_INFO_STREAM("\033[1;32m PASS! \033[0m");
    }
}

void OfflineMapUpdater::reassign_submap(double pose_x, double pose_y){
    if (is_submap_not_initialized_) {
        set_submap(*map_arranged_global_, *map_arranged_, *map_arranged_complement_, pose_x, pose_y, submap_size_);
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
            map_arranged_global_.reset(new pcl::PointCloud<pcl::PointXYZI>());
            map_arranged_global_->reserve(num_pcs_init_);
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

void OfflineMapUpdater::print_status() {
    ROS_INFO_STREAM("ERASOR Input: \033[1;33m" << map_voi_->points.size() << "\033[0m = ");
    ROS_INFO_STREAM(map_static_estimate_->points.size() << " + " << map_egocentric_complement_->points.size() << " - "
                                                        << map_rejected_->points.size());
    ROS_INFO_STREAM(" = \033[1;33m" << map_static_estimate_->points.size() + map_egocentric_complement_->points.size() -
                                       map_rejected_->points.size() << "\033[0m");
    ROS_INFO_STREAM(map_arranged_->points.size() << " " << map_filtered_->points.size() << " \033[4;32m"
                                                 << map_outskirts_->points.size() << "\033[0m");

    std::cout << "[Debug] Total: " << map_arranged_->points.size() << std::endl;
    std::cout << "[Debug] " << (double) dynamic_objs_to_viz_->points.size() / num_pcs_init_ * 100 << setw(7) << " % <- "
              << dynamic_objs_to_viz_->points.size() << " / " << num_pcs_init_ << std::endl;
    std::cout << "[Debug] " << (double) static_objs_to_viz_->points.size() / num_pcs_init_ * 100 << setw(7) << "% <- "
              << static_objs_to_viz_->points.size() << " / " << num_pcs_init_ << std::endl;
}

void OfflineMapUpdater::set_path(
        nav_msgs::Path &path, std::string mode,
        const erasor::node &node, const Eigen::Matrix4f &body2mapprev) {
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
