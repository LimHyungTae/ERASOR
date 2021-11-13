#include "tools/erasor_utils.hpp"

std::vector<int> DYNAMIC_CLASSES = {252, 253, 254, 255, 256, 257, 258, 259};

namespace erasor_utils {
    geometry_msgs::Pose eigen2geoPose(Eigen::Matrix4f pose) {
        geometry_msgs::Pose geoPose;


        tf::Matrix3x3 m;
        m.setValue((double) pose(0, 0),
                   (double) pose(0, 1),
                   (double) pose(0, 2),
                   (double) pose(1, 0),
                   (double) pose(1, 1),
                   (double) pose(1, 2),
                   (double) pose(2, 0),
                   (double) pose(2, 1),
                   (double) pose(2, 2));

        tf::Quaternion q;
        m.getRotation(q);
        geoPose.orientation.x = q.getX();
        geoPose.orientation.y = q.getY();
        geoPose.orientation.z = q.getZ();
        geoPose.orientation.w = q.getW();

        geoPose.position.x = pose(0, 3);
        geoPose.position.y = pose(1, 3);
        geoPose.position.z = pose(2, 3);

        return geoPose;
    }

    Eigen::Matrix4f geoPose2eigen(geometry_msgs::Pose geoPose) {
        Eigen::Matrix4f result = Eigen::Matrix4f::Identity();
        tf::Quaternion  q(geoPose.orientation.x, geoPose.orientation.y, geoPose.orientation.z, geoPose.orientation.w);
        tf::Matrix3x3   m(q);
        result(0, 0) = m[0][0];
        result(0, 1) = m[0][1];
        result(0, 2) = m[0][2];
        result(1, 0) = m[1][0];
        result(1, 1) = m[1][1];
        result(1, 2) = m[1][2];
        result(2, 0) = m[2][0];
        result(2, 1) = m[2][1];
        result(2, 2) = m[2][2];
        result(3, 3) = 1;

        result(0, 3) = geoPose.position.x;
        result(1, 3) = geoPose.position.y;
        result(2, 3) = geoPose.position.z;

        return result;
    }

    void parse_dynamic_obj(
            const pcl::PointCloud<pcl::PointXYZI> &cloudIn, pcl::PointCloud<pcl::PointXYZI> &dynamicOut,
            pcl::PointCloud<pcl::PointXYZI> &staticOut) {
        dynamicOut.points.clear();
        staticOut.points.clear();

        for (const auto &pt: cloudIn.points) {
            uint32_t float2int      = static_cast<uint32_t>(pt.intensity);
            uint32_t semantic_label = float2int & 0xFFFF;
            uint32_t inst_label     = float2int >> 16;
            bool     is_static      = true;
            for (int class_num: DYNAMIC_CLASSES) {
                if (semantic_label == class_num) { // 1. check it is in the moving object classes
                    dynamicOut.points.push_back(pt);
                    is_static = false;
                }
            }
            if (is_static) {
                staticOut.points.push_back(pt);
            }
        }
    }

    void voxelize_preserving_labels(pcl::PointCloud<pcl::PointXYZI>::Ptr src, pcl::PointCloud<pcl::PointXYZI> &dst, double leaf_size) {
        /**< IMPORTANT
         * Because PCL voxlizaiton just does average the intensity of point cloud,
         * so this function is to conduct voxelization followed by nearest points search to re-assign the label of each point */
        pcl::PointCloud<pcl::PointXYZI>::Ptr ptr_voxelized(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PointCloud<pcl::PointXYZI>::Ptr ptr_reassigned(new pcl::PointCloud<pcl::PointXYZI>);

        // 1. Voxelization
        static pcl::VoxelGrid<pcl::PointXYZI> voxel_filter;
        voxel_filter.setInputCloud(src);
        voxel_filter.setLeafSize(leaf_size, leaf_size, leaf_size);
        voxel_filter.filter(*ptr_voxelized);

        // 2. Find nearest point to update intensity (index and id)
        pcl::KdTreeFLANN<pcl::PointXYZI> kdtree;
        kdtree.setInputCloud(src);

        ptr_reassigned->points.reserve(ptr_voxelized->points.size());

        int K = 1;

        std::vector<int>   pointIdxNKNSearch(K);
        std::vector<float> pointNKNSquaredDistance(K);

        // Set dst <- output
        for (const auto &pt: ptr_voxelized->points) {
            if (kdtree.nearestKSearch(pt, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0) {
                auto updated = pt;
                // Update meaned intensity to original intensity
                updated.intensity = (*src)[pointIdxNKNSearch[0]].intensity;
                ptr_reassigned->points.emplace_back(updated);
            }
        }
        dst = *ptr_reassigned;
    }

    void count_stat_dyn(const pcl::PointCloud<pcl::PointXYZI> &cloudIn, int &num_static, int &num_dynamic) {
        int             tmp_static  = 0;
        int             tmp_dynamic = 0;
        for (const auto &pt: cloudIn.points) {
            uint32_t float2int      = static_cast<uint32_t>(pt.intensity);
            uint32_t semantic_label = float2int & 0xFFFF;
            uint32_t inst_label     = float2int >> 16;
            bool     is_static      = true;
            for (int class_num: DYNAMIC_CLASSES) {
                if (semantic_label == class_num) { // 1. check it is in the moving object classes
                    is_static = false;
                }
            }
            if (is_static) {
                tmp_static++;
            } else {
                tmp_dynamic++;
            }

        }
        num_static  = tmp_static;
        num_dynamic = tmp_dynamic;
    }

    void signal_callback_handler(int signum) {
    cout << "Caught Ctrl + c " << endl;
    exit(signum);
    }
}