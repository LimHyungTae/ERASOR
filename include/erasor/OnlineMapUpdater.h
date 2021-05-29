#ifndef OFFLINEMAPUPDATER_H
#define OFFLINEMAPUPDATER_H

#include "erasor.h"

#define NUM_LARGE 1000000
namespace erasor {
    class OnlineMapUpdater {
    private:
        /**< Parameters of MapUpdater from the rosparam */
        double query_voxel_size;
        double map_voxel_size;
        /**< ERASOR does not conduct dynamic object removal at all time steps!
         * removal_interval needs some heursitics */
        int    removal_interval;
        int    global_voxelization_period;
        bool   verbose;

        /**< Params. of Volume of Interest (VoI) */
        double max_range;
        double min_h;
        double max_h;

        /**< ERASOR Version
         * v2: Naive
         * v3: More conservative way*/
        int    erasor_version_;

        std::string data_name;
        std::string map_name;
        std::string environment;
        std::string save_path;

        unique_ptr<ERASOR> erasor;

        /**< ------------------------------------------ */
        int  num_pcs_init;

        ros::NodeHandle nh;

        ros::Publisher pub_path_corrected;

        ros::Publisher pub_debug_pc2_curr;
        ros::Publisher pub_map_init;
        ros::Publisher pub_static_arranged, pub_dynamic_arranged;
        ros::Publisher pub_map_rejected;
        ros::Publisher pub_curr_rejected;
        ros::Publisher pub_debug_map;
        ros::Publisher pub_debug_query_egocentric;
        ros::Publisher pub_debug_map_egocentric;
        ros::Publisher pub_debug_map_arranged_init;
        ros::Publisher pub_init_inlier, pub_init_outlier;

        ros::Subscriber          sub_node;
        ros::Subscriber          sub_flag;

        pcl::PointCloud<pcl::PointXYZI> map_init, map_arranged_init;
        pcl::PointCloud<pcl::PointXYZI> query_voi, map_voi, map_outskirts_;
        pcl::PointCloud<pcl::PointXYZI> inliers_; // w.r.t origin, i.e. map frame
        pcl::PointCloud<pcl::PointXYZI> dynamicObjs_, staticObjs_;
        // Outputs of ERASOR
        pcl::PointCloud<pcl::PointXYZI> map_complement, map_egocentric_complement;
        pcl::PointCloud<pcl::PointXYZI> map_arranged, tmp_map_arranged;
        pcl::PointCloud<pcl::PointXYZI> map_filtered, map_static_estimate;

        pcl::PointCloud<pcl::PointXYZI> curr_rejected, tmp_curr_rejected;
        pcl::PointCloud<pcl::PointXYZI> map_rejected, tmp_map_rejected;
        pcl::PointCloud<pcl::PointXYZI> map_ceilings;
        pcl::PointCloud<pcl::PointXYZI> init_inlier, init_outlier;
        sensor_msgs::PointCloud2        pc2_map;

        nav_msgs::Path path_corrected;

        std::vector<int> DYNAMIC_CLASSES = {252, 253, 254, 255, 256, 257, 258, 259};

        Eigen::Matrix4f     tf_lidar2body; /**< Transformation matrix between the initial pose to the origin */

        void set_params(); /**< Set parameters loaded from launch file */

        void callback_node(const erasor::node::ConstPtr &msg);
        /**< Flag is used when saving result pointcloud into pcd file */
        void callback_flag(const std_msgs::Float32::ConstPtr &msg);

        Eigen::Matrix4f tf_body2origin;

        void body2origin(
                const pcl::PointCloud<pcl::PointXYZI> src,
                pcl::PointCloud<pcl::PointXYZI> &dst);

        void fetch_VoI(
                double x_criterion, double y_criterion,
                pcl::PointCloud<pcl::PointXYZI> &dst, pcl::PointCloud<pcl::PointXYZI> &outskirts,
                std::string mode = "naive");

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
        OnlineMapUpdater();
        ~OnlineMapUpdater();
    };

}


#endif

