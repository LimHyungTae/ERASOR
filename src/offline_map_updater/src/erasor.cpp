#include "erasor/erasor.h"

using namespace std;

ERASOR::ERASOR() {
}

ERASOR::~ERASOR() {
}

double ERASOR::xy2theta(const double &x, const double &y) { // 0 ~ 2 * PI
    if (y >= 0) {
        return atan2(y, x); // 1, 2 quadrant
    } else {
        return 2 * PI + atan2(y, x);// 3, 4 quadrant
    }
}

double ERASOR::xy2radius(const double &x, const double &y) {
    return sqrt(pow(x, 2) + pow(y, 2));
}

void ERASOR::clear(pcl::PointCloud<pcl::PointXYZI> &pt_cloud) {
    if (!pt_cloud.empty()) {
        pt_cloud.clear();
    }
}

void ERASOR::init(R_POD &r_pod) {
    if (!r_pod.empty()) {
        r_pod.clear();
    }
    Ring ring;
    Bin  bin = {-INF, INF, 0, 0, false, static_cast<bool>(NOT_ASSIGNED)};
    bin.points.reserve(ENOUGH_NUM);
    for (int i = 0; i < num_sectors; i++) {
        ring.emplace_back(bin);
    }
    for (int j = 0; j < num_rings; j++) {
        r_pod.emplace_back(ring);
    }
}

void ERASOR::clear_bin(Bin &bin) {
    bin.max_h       = -INF;
    bin.min_h       = INF;
    bin.x           = 0;
    bin.y           = 0;
    bin.is_occupied = false;
    bin.status      = NOT_ASSIGNED;
    if (!bin.points.empty()) bin.points.clear();
}

/**
 * @brief Inputs should be the transformed pointcloud!
 */
void ERASOR::set_inputs(
        const pcl::PointCloud<pcl::PointXYZI> &map_voi,
        const pcl::PointCloud<pcl::PointXYZI> &query_voi) {

    clear(debug_curr_rejected);
    clear(debug_map_rejected);
    clear(map_complement);

    for (int theta = 0; theta < num_sectors; ++theta) {
        for (int r = 0; r < num_rings; ++r) {
            clear_bin(r_pod_map[r][theta]);
            clear_bin(r_pod_curr[r][theta]);
            clear_bin(r_pod_selected[r][theta]);
        }
    }
    voi2r_pod(query_voi, r_pod_curr);
    voi2r_pod(map_voi, r_pod_map, map_complement);

    int      debug_total_num = 0;
    for (int theta           = 0; theta < num_sectors; theta++) {
        for (int r = 0; r < num_rings; r++) {
            Bin &bin_map = r_pod_map[r][theta];
            debug_total_num += bin_map.points.size();
        }
    }
//  ROS_INFO_STREAM("ERASOR_SRC: \033[1;36m"<<map_voi.points.size()<<"\033[0m =");
//  ROS_INFO_STREAM(" \033[1;36m"<<debug_total_num + map_complement.points.size()<<"\033[0m | "<<debug_total_num << " + \033[1;34m"<<map_complement.points.size()<<"\033[0m");

}

void ERASOR::pt2r_pod(const pcl::PointXYZI &pt, Bin &bin) {
    bin.is_occupied = true;
    bin.points.push_back(pt);
    if (pt.z >= bin.max_h) {
        bin.max_h = pt.z;
        bin.x     = pt.x;
        bin.y     = pt.y;
    }
    if (pt.z <= bin.min_h) {
        bin.min_h = pt.z;
    }
}

void ERASOR::voi2r_pod(
        const pcl::PointCloud<pcl::PointXYZI> &src,
        R_POD &r_pod) {
    for (auto const &pt : src.points) {
        if (pt.z < max_h && pt.z > min_h) {
            double r = xy2radius(pt.x, pt.y);
            if (r <= max_r) {
                double theta = xy2theta(pt.x, pt.y);

                int sector_idx = min(static_cast<int>((theta / sector_size)), num_sectors - 1);
                int ring_idx   = min(static_cast<int>((r / ring_size)), num_rings - 1);

                pt2r_pod(pt, r_pod.at(ring_idx).at(sector_idx));
            }
        }
    }

    // For debugging
    pcl::PointCloud<pcl::PointXYZI> curr_init;
    r_pod2pc(r_pod, curr_init);
    sensor_msgs::PointCloud2 pc2_curr_init = erasor_utils::cloud2msg(curr_init);
    pub_curr_init.publish(pc2_curr_init);
}

void ERASOR::voi2r_pod(
        const pcl::PointCloud<pcl::PointXYZI> &src,
        R_POD &r_pod, pcl::PointCloud<pcl::PointXYZI> &complement) {
    for (auto const                 &pt : src.points) {
        if (pt.z < max_h && pt.z > min_h) { // range of z?
            double r = xy2radius(pt.x, pt.y);
            if (r <= max_r) {
                double theta      = xy2theta(pt.x, pt.y);
//        int sector_idx = min(static_cast<int>((theta / sector_size) + 0.5), num_sectors - 1);
//        int ring_idx = min(static_cast<int>((r / ring_size) + 0.5), num_rings - 1);
                int    sector_idx = min(static_cast<int>((theta / sector_size)), num_sectors - 1);
                int    ring_idx   = min(static_cast<int>((r / ring_size)), num_rings - 1);
                pt2r_pod(pt, r_pod.at(ring_idx).at(sector_idx));
            } else { complement.points.push_back(pt); }
        } else { complement.points.push_back(pt); }
    }
    pcl::PointCloud<pcl::PointXYZI> map_init;
    r_pod2pc(r_pod, map_init);
    sensor_msgs::PointCloud2 pc2_map_init = erasor_utils::cloud2msg(map_init);
    pub_map_init.publish(pc2_map_init);
}

void ERASOR::viz_pseudo_occupancy() {
    jsk_recognition_msgs::PolygonArray map_r_pod, curr_r_pod;
    map_r_pod.header.frame_id = "map";
    map_r_pod.header.stamp    = ros::Time::now();

    curr_r_pod.header.frame_id = "map";
    curr_r_pod.header.stamp    = ros::Time::now();

    for (int theta = 0; theta < num_sectors; theta++) {
        for (int r = 0; r < num_rings; r++) {
            Bin  &bin_curr = r_pod_curr[r][theta];
            Bin  &bin_map  = r_pod_map[r][theta];
            auto polygons  = set_polygons(r, theta, 3);
            polygons.header = curr_r_pod.header;
            map_r_pod.polygons.push_back(polygons);
            curr_r_pod.polygons.push_back(polygons);

            if (bin_curr.is_occupied) {
                double curr_pod = bin_curr.max_h - bin_curr.min_h;
                curr_r_pod.likelihood.push_back(curr_pod / (max_h - min_h));
            } else {
                curr_r_pod.likelihood.push_back(LITTLE_NUM);
            }
            if (bin_map.is_occupied) {
                double map_pod = bin_map.max_h - bin_map.min_h;
                map_r_pod.likelihood.push_back(map_pod / (max_h - min_h));
            } else {
                map_r_pod.likelihood.push_back(LITTLE_NUM);
            }
        }
    }

    pub_map_marker.publish(map_r_pod);
    pub_curr_marker.publish(curr_r_pod);
}


void ERASOR::estimate_plane_(const pcl::PointCloud<pcl::PointXYZI> &ground) {
    Eigen::Matrix3f cov;
    Eigen::Vector4f pc_mean;
    pcl::computeMeanAndCovarianceMatrix(ground, cov, pc_mean);
    // Singular Value Decomposition: SVD
    Eigen::JacobiSVD<Eigen::MatrixXf> svd(cov, Eigen::DecompositionOptions::ComputeFullU);
    // use the least singular vector as normal
    normal_ = (svd.matrixU().col(2));
    // mean ground seeds value
    Eigen::Vector3f seeds_mean = pc_mean.head<3>();

    // according to normal.T*[x,y,z] = -d
    d_         = -(normal_.transpose() * seeds_mean)(0, 0);
    // set distance threhold to `th_dist - d`
    th_dist_d_ = th_dist_ - d_;
}

bool point_cmp(pcl::PointXYZI a, pcl::PointXYZI b) {
    return a.z < b.z;
}

void ERASOR::extract_initial_seeds_(
        const pcl::PointCloud<pcl::PointXYZI> &p_sorted,
        pcl::PointCloud<pcl::PointXYZI> &init_seeds) {
    init_seeds.points.clear();
    pcl::PointCloud<pcl::PointXYZI> g_seeds_pc;

    // LPR is the mean of low point representative
    double sum = 0;
    int    cnt = 0;

    // Calculate the mean height value.

    for (int i          = num_lowest_pts; i < p_sorted.points.size() && cnt < num_lprs_; i++) {
        sum += p_sorted.points[i].z;
        cnt++;
    }
    double   lpr_height = cnt != 0 ? sum / cnt : 0;// in case divide by 0
    g_seeds_pc.clear();
    // iterate pointcloud, filter those height is less than lpr.height+th_seeds_
    for (int i = 0; i < p_sorted.points.size(); i++) {
        if (p_sorted.points[i].z < lpr_height + th_seeds_heights_) {
            g_seeds_pc.points.push_back(p_sorted.points[i]);
        }
    }
//  std::cout<<"hey!! g seeds"<<g_seeds_pc.points.size()<<std::endl;
    // return seeds points
    init_seeds = g_seeds_pc;
}

void ERASOR::extract_ground(
        const pcl::PointCloud<pcl::PointXYZI> &src,
        pcl::PointCloud<pcl::PointXYZI> &dst, pcl::PointCloud<pcl::PointXYZI> &outliers) {
    if (!dst.empty()) dst.clear();
    if (!outliers.empty()) outliers.clear();

    auto src_copy = src;
    std::sort(src_copy.points.begin(), src_copy.points.end(), point_cmp);
    // 1. remove_outliers;
    auto     it = src_copy.points.begin();
    for (int i  = 0; i < src_copy.points.size(); i++) {
        // if(src_copy.points[i].z < -0.5*SENSOR_HEIGHTS){
        if (src_copy.points[i].z < min_h) {
            it++;
        } else {
            break;
        }
    }
    src_copy.points.erase(src_copy.points.begin(), it);

    // 2. set seeds!
    if (!ground_pc_.empty()) ground_pc_.clear();
    if (!non_ground_pc_.empty()) non_ground_pc_.clear();

    extract_initial_seeds_(src_copy, ground_pc_);
//  std::cout<<"\033[1;032m [Ground] num: "<<ground_pc.points.size()<<std::endl;
    // 3. Extract ground
    for (int i = 0; i < iter_groundfilter_; i++) {
        estimate_plane_(ground_pc_);
        ground_pc_.clear();

        //pointcloud to matrix
        Eigen::MatrixXf points(src.points.size(), 3);
        int             j      = 0;
        for (auto       p:src.points) {
            points.row(j++) << p.x, p.y, p.z;
        }
        // ground plane model
        Eigen::VectorXf result = points * normal_;
        // threshold filter
        for (int        r      = 0; r < result.rows(); r++) {
            if (result[r] < th_dist_d_) {
                ground_pc_.points.push_back(src[r]);
            } else {
                if (i == (iter_groundfilter_ - 1)) { // Last iteration
                    non_ground_pc_.points.push_back(src[r]);
                }
            }
        }
//      std::cout<<"\033[1;032m [Ground] ith: "<< i<<" | num: "<<ground_pc.points.size()<<std::endl;
    }
    dst      = ground_pc_;
    outliers = non_ground_pc_;
//  if (normal_(2,0) > 0.707){// perpendicular to ground
//    dst = ground_pc_;
//    outliers = non_ground_pc_;
//  }else{ // reject results. Do not discern
//    std::cout<<"rejected!"<<std::endl;
//    dst = ground_pc_ + non_ground_pc_;
//  }

}

void ERASOR::merge_bins(const Bin &src1, const Bin &src2, Bin &dst) {
    dst.max_h       = max(src1.max_h, src2.max_h);
    dst.min_h       = min(src1.min_h, src2.min_h);
    dst.is_occupied = true;
    dst.points.clear();
    for (auto const &pt : src1.points) {
        dst.points.push_back(pt);
    }
    for (auto const &pt : src2.points) {
        dst.points.push_back(pt);
    }
}

void ERASOR::r_pod2pc(const R_POD &sc, pcl::PointCloud<pcl::PointXYZI> &pc) {
    pc.points.clear();
    for (int theta = 0; theta < num_sectors; theta++) {
        for (int r = 0; r < num_rings; r++) {
            if (sc.at(r).at(theta).is_occupied) {
                for (auto const &pt : sc.at(r).at(theta).points) {
                    pc.points.push_back(pt);
                }
            }
        }
    }
}

void ERASOR::get_outliers(
        pcl::PointCloud<pcl::PointXYZI> &map_rejected,
        pcl::PointCloud<pcl::PointXYZI> &curr_rejected) {
    map_rejected  = debug_map_rejected;
    curr_rejected = debug_curr_rejected;
}

// Version 2.
// Retrieve piecewise ground!
// Color maybe changed
void ERASOR::compare_vois_and_revert_ground(int frame) {
    jsk_recognition_msgs::PolygonArray poly_list;
    poly_list.header.frame_id = "map";
    poly_list.header.stamp    = ros::Time::now();

    int        dynamic_count;
    static int cnt            = 0;
    ground_viz.points.clear();

//  std::string filename = "/home/shapelim/debug/" + std::to_string(frame) +".csv";
//  ofstream output(filename.data());
//  output<<"which_one_is_higher,r,theta,min_h,max_h,diff_h,min_h,max_h,diff_h,ratio\n";

    cnt++;
    for (int theta = 0; theta < num_sectors; theta++) {
        for (int r = 0; r < num_rings; r++) {

            // VISUALIZATION
            Bin &bin_curr = r_pod_curr[r][theta];
            Bin &bin_map  = r_pod_map[r][theta];

            // Min. num of pts criteria.
            if (bin_curr.points.size() < minimum_num_pts) {
                r_pod_selected[r][theta] = bin_map;
//        debug_curr_rejected += bin_curr.points;

                auto polygons = set_polygons(r, theta, 3);
                polygons.header = poly_list.header;
                poly_list.polygons.push_back(polygons);
                poly_list.likelihood.push_back(LITTLE_NUM);

                continue;

            }
            if (bin_curr.is_occupied && bin_map.is_occupied) {
                // ---------------------------------
                //          Scan Ratio Test
                // ---------------------------------
                double map_h_diff  = bin_map.max_h - bin_map.min_h;
                double curr_h_diff = bin_curr.max_h - bin_curr.min_h;
                double scan_ratio  = min(map_h_diff / curr_h_diff,
                                         curr_h_diff / map_h_diff);

                if (scan_ratio < scan_ratio_threshold) { // find dynamic!

                    if (map_h_diff >= curr_h_diff) { // Occupied -> Disappear  <<green>>
                        auto polygons = set_polygons(r, theta, 3);
                        polygons.header = poly_list.header;
                        poly_list.polygons.push_back(polygons);
                        poly_list.likelihood.push_back(MAP_IS_HIGHER);

                        if (bin_map.max_h > th_bin_max_h) {
                            r_pod_selected[r][theta] = bin_curr;
                            // ---------------------------------
                            //     R-GPF is operated!
                            //     NOTE: Ground is retrieved!
                            // ---------------------------------
                            if (!piecewise_ground_.empty()) piecewise_ground_.clear();
                            if (!non_ground_.empty()) non_ground_.clear();
                            extract_ground(bin_map.points, piecewise_ground_, non_ground_);
                            r_pod_selected[r][theta].points += piecewise_ground_;
                            ground_viz += piecewise_ground_;
                            debug_map_rejected += non_ground_;
                        } else {
                            r_pod_selected[r][theta] = bin_map;
                        }
                    } else if (map_h_diff <= curr_h_diff) { // No objects exist -> Appear! <<red>>
                        auto polygons = set_polygons(r, theta, 3);
                        polygons.header = poly_list.header;
                        poly_list.polygons.push_back(polygons);
                        poly_list.likelihood.push_back(CURR_IS_HIGHER);

                        r_pod_selected[r][theta] = bin_map;
                        if (bin_curr.max_h > th_bin_max_h) {
                            //            r_pod_selected[r][theta] = bin_curr;
                            debug_curr_rejected += bin_curr.points;
                        }
                    }

                } else {
                    auto polygons = set_polygons(r, theta, 3);
                    polygons.header = poly_list.header;
                    poly_list.polygons.push_back(polygons);
                    poly_list.likelihood.push_back(MERGE_BINS);
                    Bin bin_merged;
                    merge_bins(bin_curr, bin_map, bin_merged);
                    r_pod_selected[r][theta] = bin_merged; // present scene is better I think?
                }
            } else if (bin_curr.is_occupied) {
                r_pod_selected[r][theta] = bin_curr;
            } else if (bin_map.is_occupied) {
                r_pod_selected[r][theta] = bin_map;
            }

        }
    }
    // For debugging
    sensor_msgs::PointCloud2 pc2_map_r  = erasor_utils::cloud2msg(debug_map_rejected);
    sensor_msgs::PointCloud2 pc2_curr_r = erasor_utils::cloud2msg(debug_curr_rejected);
    pub_map_rejected.publish(pc2_map_r);
    pub_curr_rejected.publish(pc2_curr_r);
    pub_viz_bin_marker.publish(poly_list);
}

// Version 3.
// Retrieve piecewise with blocking!
void ERASOR::compare_vois_and_revert_ground_w_block(int frame) {
    jsk_recognition_msgs::PolygonArray poly_list;
    poly_list.header.frame_id = "map";
    poly_list.header.stamp    = ros::Time::now();

    int dynamic_count;

    ground_viz.points.clear();

    // 1. Update status!!
    for (int theta = 0; theta < num_sectors; theta++) {
        for (int r = 0; r < num_rings; r++) {
            // Min. num of pts criteria.
            Bin &bin_curr = r_pod_curr[r][theta];
            Bin &bin_map  = r_pod_map[r][theta];

            if (bin_map.points.empty()) {
                r_pod_selected[r][theta].status = LITTLE_NUM;
                continue;
            }

            if (bin_curr.points.size() < minimum_num_pts) {
                r_pod_selected[r][theta].status = LITTLE_NUM;
            } else {
                double map_h_diff  = bin_map.max_h - bin_map.min_h;
                double curr_h_diff = bin_curr.max_h - bin_curr.min_h;
                double scan_ratio  = min(map_h_diff / curr_h_diff,
                                         curr_h_diff / map_h_diff);
                // ---------------------------------
                //          Scan Ratio Test
                // ---------------------------------
                if (bin_curr.is_occupied && bin_map.is_occupied) {
                    if (scan_ratio < scan_ratio_threshold) { // find dynamic!
                        if (map_h_diff >= curr_h_diff) { // Occupied -> Disappear  <<BLUE>>
                            r_pod_selected[r][theta].status = MAP_IS_HIGHER;
                        } else if (map_h_diff <= curr_h_diff) { // No objects exist -> Appear! <<GREEN>>
                            r_pod_selected[r][theta].status = CURR_IS_HIGHER;
                        }
                    } else {
                        r_pod_selected[r][theta].status = MERGE_BINS;
                    }
                } else if (bin_map.is_occupied) { // Maybe redundant?
                    r_pod_selected[r][theta].status = LITTLE_NUM;
                }
            }

        }
//    out_map<<"\n";  out_curr<<"\n";
    }

//  pcl::PointCloud<pcl::PointXYZI> origin_total, ground_total, dummy_non_ground;
    int num_origin_stat, num_origin_dyn;
    int num_ground_stat, num_ground_dyn;

    // 2. set bins!
    for (int theta = 0; theta < num_sectors; theta++) {
        for (int r = 0; r < num_rings; r++) {
            //visualization
            auto polygons = set_polygons(r, theta, 3);
            polygons.header = poly_list.header;
            poly_list.polygons.push_back(polygons);

            Bin &bin_curr = r_pod_curr[r][theta];
            Bin &bin_map  = r_pod_map[r][theta];

            double OCCUPANCY_STATUS = r_pod_selected[r][theta].status;
            if (OCCUPANCY_STATUS == LITTLE_NUM) {
                r_pod_selected[r][theta] = bin_map;
                r_pod_selected[r][theta].status = LITTLE_NUM;

                poly_list.likelihood.push_back(LITTLE_NUM);

            } else if (OCCUPANCY_STATUS == MAP_IS_HIGHER) {
                if ((bin_map.max_h - bin_map.min_h) > 0.5) {
                    r_pod_selected[r][theta] = bin_curr;
                    r_pod_selected[r][theta].status = MAP_IS_HIGHER;
                    // ---------------------------------
                    //     NOTE: Ground is retrieved!
                    // ---------------------------------

                    if (!piecewise_ground_.empty()) piecewise_ground_.clear();
                    if (!non_ground_.empty()) non_ground_.clear();

                    extract_ground(bin_map.points, piecewise_ground_, non_ground_);
                    /*** It potentially requires lots of memories... */
                    r_pod_selected[r][theta].points += piecewise_ground_;

                    /*** Thus, voxelization is conducted */
                    pcl::PointCloud<pcl::PointXYZI>::Ptr tmp(new pcl::PointCloud<pcl::PointXYZI>);
                    *tmp = r_pod_selected[r][theta].points;
                    erasor_utils::voxelize_preserving_labels(tmp, r_pod_selected[r][theta].points, map_voxel_size_);

                    erasor_utils::count_stat_dyn(piecewise_ground_, num_ground_stat, num_ground_dyn);
                    ground_viz += piecewise_ground_;
                    debug_map_rejected += non_ground_;

                    poly_list.likelihood.push_back(MAP_IS_HIGHER);
                } else {
                    r_pod_selected[r][theta] = bin_map;
                    r_pod_selected[r][theta].status = NOT_ASSIGNED;

                    poly_list.likelihood.push_back(NOT_ASSIGNED);
                }

            } else if (OCCUPANCY_STATUS == CURR_IS_HIGHER) {
                r_pod_selected[r][theta] = bin_map;
                r_pod_selected[r][theta].status = CURR_IS_HIGHER;

                poly_list.likelihood.push_back(CURR_IS_HIGHER);

            } else if (OCCUPANCY_STATUS == MERGE_BINS) {
                if (is_dynamic_obj_close(r_pod_selected, r, theta, 1, 1)) {
                    r_pod_selected[r][theta] = bin_map;
                    r_pod_selected[r][theta].status = BLOCKED;

                    poly_list.likelihood.push_back(BLOCKED);
                } else {
                    // NOTE the dynamic object comes ....:(
                    r_pod_selected[r][theta] = bin_map;
                    r_pod_selected[r][theta].status = MERGE_BINS;

                    poly_list.likelihood.push_back(MERGE_BINS);
                }
            }
        }
    }

    // For debugging
    sensor_msgs::PointCloud2 pc2_map_r  = erasor_utils::cloud2msg(debug_map_rejected);
    sensor_msgs::PointCloud2 pc2_curr_r = erasor_utils::cloud2msg(debug_curr_rejected);
    pub_map_rejected.publish(pc2_map_r);
    pub_curr_rejected.publish(pc2_curr_r);
    pub_viz_bin_marker.publish(poly_list);
}

bool ERASOR::is_dynamic_obj_close(R_POD &r_pod_selected, int r_target, int theta_target, int r_range, int theta_range) {
    // Set thetas
    std::vector<int> theta_candidates;
    for (int         j = theta_target - theta_range; j <= theta_target + theta_range; j++) {
        if (j < 0) {
            theta_candidates.push_back(j + num_rings);
        } else if (j >= num_sectors) {
            theta_candidates.push_back(j - num_rings);
        } else {
            theta_candidates.push_back(j);
        }
    }
    for (int         r = std::max(0, r_target - r_range); r <= std::min(r_target + r_range, num_rings - 1); r++) {
        for (const auto &theta:theta_candidates) {
            if ((r == r_target) && (theta == theta_target)) continue;

            if (r_pod_selected[r][theta].status == CURR_IS_HIGHER) { // Dynamic object is near
                return true;
            }
        }
    }
    return false;
}

bool ERASOR::has_dynamic(Bin &bin) {
    for (const auto &pt:bin.points) {
        uint32_t float2int      = static_cast<uint32_t>(pt.intensity);
        uint32_t semantic_label = float2int & 0xFFFF;

        for (int class_num: DYNAMIC_CLASSES) {
            if (semantic_label == class_num) { // 1. check it is in the moving object classes
                return true;
            }
        }
    }
    return false;
}


void ERASOR::get_static_estimate(
        pcl::PointCloud<pcl::PointXYZI> &arranged,
        pcl::PointCloud<pcl::PointXYZI> &complement) {
    r_pod2pc(r_pod_selected, arranged);
    arranged += ground_viz;
    if(ground_viz.size() != 0){
        sensor_msgs::PointCloud2 pc2_ground = erasor_utils::cloud2msg(ground_viz);
        pub_ground.publish(pc2_ground);
    }

    complement = map_complement;
    sensor_msgs::PointCloud2 pc2_arranged       = erasor_utils::cloud2msg(arranged);
    sensor_msgs::PointCloud2 pc2_map_complement = erasor_utils::cloud2msg(complement);
    pub_arranged.publish(pc2_arranged);
}

double ERASOR::get_max_range() {return max_r;}

geometry_msgs::PolygonStamped ERASOR::set_polygons(int r_idx, int theta_idx, int num_split) {
    geometry_msgs::PolygonStamped polygons;
    // Set point of polygon. Start from RL and ccw
    geometry_msgs::Point32        point;

    point.z = max_h + 0.5;
    // RL
    double r_len = r_idx * ring_size;
    double angle = theta_idx * sector_size;

    point.x = r_len * cos(angle);
    point.y = r_len * sin(angle);
    polygons.polygon.points.push_back(point);
    // RU
    r_len = r_len + ring_size;
    point.x = r_len * cos(angle);
    point.y = r_len * sin(angle);
    polygons.polygon.points.push_back(point);

    // RU -> LU
    for (int idx = 1; idx <= num_split; ++idx) {
        angle = angle + sector_size / num_split;
        point.x = r_len * cos(angle);
        point.y = r_len * sin(angle);
        polygons.polygon.points.push_back(point);
    }

    r_len = r_len - ring_size;
    point.x = r_len * cos(angle);
    point.y = r_len * sin(angle);
    polygons.polygon.points.push_back(point);

    for (int idx = 1; idx < num_split; ++idx) {
        angle = angle - sector_size / num_split;
        point.x = r_len * cos(angle);
        point.y = r_len * sin(angle);
        polygons.polygon.points.push_back(point);
    }

    return polygons;
}


