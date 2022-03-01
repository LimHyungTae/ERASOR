#ifndef OFFLINEMAPUPDATER_H
#define OFFLINEMAPUPDATER_H

#include "erasor.h"

#define NUM_PTS_LARGE_ENOUGH 200000
#define NUM_PTS_LARGE_ENOUGH_FOR_MAP 20000000
namespace erasor {
    class OfflineMapUpdater {
    private:
        /**< Parameters of MapUpdater from the rosparam */
        double query_voxel_size_;
        double map_voxel_size_;
        /**< ERASOR does not conduct dynamic object removal at all time steps!
         * removal_interval needs some heursitics */
        int    removal_interval_;
        int    global_voxelization_period_;
        bool   verbose_;
        bool   is_large_scale_;
        bool   is_submap_not_initialized_ = true;

        /**< Params. of Volume of Interest (VoI) */
        double max_range_;
        double min_h_;
        double max_h_;
        double submap_size_;
        double submap_center_x_;
        double submap_center_y_;

        /**< ERASOR Version
         * v2: Naive
         * v3: More conservative way*/
        int erasor_version_;

        std::string data_name_;
        std::string map_name_;
        std::string environment_;
        std::string save_path_;

        unique_ptr<ERASOR> erasor_;

        /**< ------------------------------------------ */
        int num_pcs_init_;

        ros::NodeHandle nh;

        ros::Subscriber sub_node_;
        ros::Subscriber sub_flag_;

        ros::Publisher pub_path_;

        ros::Publisher pub_map_init_;
        ros::Publisher pub_static_arranged_, pub_dynamic_arranged_;
        ros::Publisher pub_map_rejected_;
        ros::Publisher pub_curr_rejected_;

        ros::Publisher pub_debug_map_arranged_init_;
        ros::Publisher pub_debug_query_egocentric_;
        ros::Publisher pub_debug_map_egocentric_;

        ros::Publisher pub_debug_pc2_curr_;
        ros::Publisher pub_debug_map_;

        /***
         * Variables ralted with global map
         */
        pcl::PointCloud<pcl::PointXYZI>::Ptr map_init_;
        pcl::PointCloud<pcl::PointXYZI>::Ptr map_arranged_;
        pcl::PointCloud<pcl::PointXYZI>::Ptr map_arranged_init_;
        pcl::PointCloud<pcl::PointXYZI>::Ptr map_arranged_global_;
        pcl::PointCloud<pcl::PointXYZI>::Ptr map_arranged_complement_;
        pcl::PointCloud<pcl::PointXYZI>::Ptr map_ceilings_;

        /*** Inputs of ERASOR */
        pcl::PointCloud<pcl::PointXYZI>::Ptr query_voi_;
        pcl::PointCloud<pcl::PointXYZI>::Ptr map_voi_;
        pcl::PointCloud<pcl::PointXYZI>::Ptr map_voi_wrt_origin_; // w.r.t origin, i.e. map frame
        pcl::PointCloud<pcl::PointXYZI>::Ptr map_outskirts_;

        /*** Outputs of ERASOR
         * map_filtered_ = map_static_estimate + map_egocentric_complement
         */
        pcl::PointCloud<pcl::PointXYZI>::Ptr map_static_estimate_;
        pcl::PointCloud<pcl::PointXYZI>::Ptr map_egocentric_complement_;
        pcl::PointCloud<pcl::PointXYZI>::Ptr map_filtered_;

        pcl::PointCloud<pcl::PointXYZI>::Ptr query_rejected_;
        pcl::PointCloud<pcl::PointXYZI>::Ptr map_rejected_;
        pcl::PointCloud<pcl::PointXYZI>::Ptr total_query_rejected_;
        pcl::PointCloud<pcl::PointXYZI>::Ptr total_map_rejected_;

        pcl::PointCloud<pcl::PointXYZI>::Ptr dynamic_objs_to_viz_;
        pcl::PointCloud<pcl::PointXYZI>::Ptr static_objs_to_viz_;

        // Published msgs
        sensor_msgs::PointCloud2 pc2_map_;
        nav_msgs::Path           path_;

        Eigen::Matrix4f tf_lidar2body_; /**< Transformation matrix between the initial pose to the origin */
        Eigen::Matrix4f tf_body2origin_;

        void set_params(); /**< Set parameters loaded from launch file */

        void initialize_ptrs();

        void load_global_map();

        void callback_node(const erasor::node::ConstPtr &msg);

        /**< Flag is used when saving result pointcloud into pcd file */
        void callback_flag(const std_msgs::Float32::ConstPtr &msg);

        void body2origin(
                const pcl::PointCloud<pcl::PointXYZI> src,
                pcl::PointCloud<pcl::PointXYZI> &dst);

        void reassign_submap(double pose_x, double pose_y);

        void set_submap(
                const pcl::PointCloud<pcl::PointXYZI> &map_global,
                pcl::PointCloud<pcl::PointXYZI>& submap,
                pcl::PointCloud<pcl::PointXYZI>& submap_complement,
                double x, double y, double submap_size);

        void fetch_VoI(
                double x_criterion, double y_criterion,
                pcl::PointCloud<pcl::PointXYZI> &dst, pcl::PointCloud<pcl::PointXYZI> &outskirts,
                std::string mode = "naive");

        void print_status();

        void set_path(
                nav_msgs::Path &path, std::string mode,
                const erasor::node &node, const Eigen::Matrix4f &body2mapprev);

        void publish(
                const sensor_msgs::PointCloud2 &map,
                const ros::Publisher &publisher);

        void publish(
                const pcl::PointCloud<pcl::PointXYZI> &map,
                const ros::Publisher &publisher);

        geometry_msgs::Pose pose_curr;

    public:
        OfflineMapUpdater();

        ~OfflineMapUpdater();

        void save_static_map(float voxel_size);
    };

}


#endif

