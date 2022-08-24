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

    std::vector<double> SO3ToEuler(const Eigen::Matrix3d &rot_mat, std::string output_unit)
    {
        Eigen::Matrix<double, 3, 1> _ang;
        Eigen::Quaterniond q(rot_mat);
        Eigen::Vector4d q_data = {q.x(), q.y(), q.z(), q.w()};
        //scalar w=rot_mat.coeffs[3], x=rot_mat.coeffs[0], y=rot_mat.coeffs[1], z=rot_mat.coeffs[2];
        double sqw = q_data[3]*q_data[3];
        double sqx = q_data[0]*q_data[0];
        double sqy = q_data[1]*q_data[1];
        double sqz = q_data[2]*q_data[2];
        double unit = sqx + sqy + sqz + sqw; // if normalized is one, otherwise is correction factor
        double test = q_data[3]*q_data[1] - q_data[2]*q_data[0];

        double conversion = output_unit == "rad"? 1: 57.3;
        if (test > 0.49999*unit) { // singularity at north pole

            _ang << 2 * std::atan2(q_data[0], q_data[3]), M_PI/2, 0;
            std::vector<double> euler_ang  = {_ang[0] * conversion, _ang[1] * conversion, _ang[2] * conversion};
            return euler_ang;
        }
        if (test < -0.49999*unit) { // singularity at south pole
            _ang << -2 * std::atan2(q_data[0], q_data[3]), -M_PI/2, 0;
            std::vector<double> euler_ang = {_ang[0] * conversion, _ang[1] * conversion, _ang[2] * conversion};
            return euler_ang;
        }
        // https://stackoverflow.com/questions/5782658/extracting-yaw-from-a-quaternion
        // float roll  = atan2(2.0 * (q.q3 * q.q2 + q.q0 * q.q1) , 1.0 - 2.0 * (q.q1 * q.q1 + q.q2 * q.q2));
        // float pitch = asin(2.0 * (q.q2 * q.q0 - q.q3 * q.q1));
        // float yaw   = atan2(2.0 * (q.q3 * q.q0 + q.q1 * q.q2) , - 1.0 + 2.0 * (q.q0 * q.q0 + q.q1 * q.q1));
        _ang <<
             std::atan2(2*q_data[0]*q_data[3]+2*q_data[1]*q_data[2] , -sqx - sqy + sqz + sqw),
                std::asin (2*test/unit),
                std::atan2(2*q_data[2]*q_data[3]+2*q_data[1]*q_data[0] , sqx - sqy - sqz + sqw);
        std::vector<double> euler_ang = {_ang[0] * conversion, _ang[1] * conversion, _ang[2] * conversion};
        // euler_ang[0] = roll, euler_ang[1] = pitch, euler_ang[2] = yaw
        return euler_ang;
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