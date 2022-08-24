#include "tools/erasor_utils.hpp"

#define INF 10000000000000.0
#define PI 3.1415926535
#define ENOUGH_NUM 8000

#define EMPTY 0
#define MAP 1
#define PC_CURR 2
// COLORS:
// 0 -> BLUE
#define MAP_IS_HIGHER 0.5
#define CURR_IS_HIGHER 1.0
#define LITTLE_NUM 0.0       // For viz: blue - not activated
#define BLOCKED 0.8         // For viz

#define MERGE_BINS 0.25
#define NOT_ASSIGNED 0.0
// ground params


using namespace std;

struct Bin {
    double max_h;
    double min_h;
    double x;
    double y;
    double status;
    bool   is_occupied;

    pcl::PointCloud<pcl::PointXYZI> points;
};

struct DynamicBinIdx {
    int r;
    int theta;
};

typedef vector<vector<Bin> > R_POD;
typedef vector<Bin>          Ring;

class ERASOR {
public:

    ERASOR(ros::NodeHandle *nodehandler) : nh(*nodehandler) {
        nh.param("/erasor/max_range", max_r, 10.0);
        nh.param("/erasor/num_rings", num_rings, 20);
        nh.param("/erasor/num_sectors", num_sectors, 60);
        nh.param("/erasor/max_h", max_h, 3.0);
        nh.param("/erasor/min_h", min_h, 0.0);
        nh.param("/erasor/th_bin_max_h", th_bin_max_h, 0.39);
        nh.param("/erasor/scan_ratio_threshold", scan_ratio_threshold, 0.22);
        nh.param("/erasor/num_lowest_pts", num_lowest_pts, 5);
        nh.param("/erasor/minimum_num_pts", minimum_num_pts, 4);
        nh.param("/erasor/rejection_ratio", rejection_ratio, 0.33);
        nh.param("/erasor/gf_dist_thr", th_dist_, 0.05);
        nh.param("/erasor/gf_iter", iter_groundfilter_, 3);
        nh.param("/erasor/gf_num_lpr", num_lprs_, 10);
        nh.param("/erasor/gf_th_seeds_height", th_seeds_heights_, 0.5);
        nh.param("/erasor/map_voxel_size", map_voxel_size_, 0.2);

        ring_size   = max_r / num_rings;
        sector_size = 2 * PI / num_sectors;

        // SCDR is our project's name
        pub_map_rejected  = nh.advertise<sensor_msgs::PointCloud2>("/SCDR/debug/map_rejected", 100);
        pub_curr_rejected = nh.advertise<sensor_msgs::PointCloud2>("/SCDR/debug/curr_rejected", 100);
        pub_map_init      = nh.advertise<sensor_msgs::PointCloud2>("/SCDR/debug/map_init", 100);
        pub_curr_init     = nh.advertise<sensor_msgs::PointCloud2>("/SCDR/debug/curr_init", 100);

        pub_map_marker     = nh.advertise<jsk_recognition_msgs::PolygonArray>("/SCDR/debug/map_marker", 100);
        pub_curr_marker    = nh.advertise<jsk_recognition_msgs::PolygonArray>("/SCDR/debug/curr_marker", 100);
        pub_viz_bin_marker = nh.advertise<jsk_recognition_msgs::PolygonArray>("/SCDR/debug/polygons_marker", 100);

        pub_ground   = nh.advertise<sensor_msgs::PointCloud2>("/SCDR/debug/ground", 100);
        pub_arranged = nh.advertise<sensor_msgs::PointCloud2>("/SCDR/debug/arranged", 100);

        std::cout << "-----\033[1;32mParams. of ERASOR\033[0m-----" << std::endl;
        std::cout << "max range: " << max_r << std::endl;
        std::cout << "num_rings: " << num_rings << std::endl;
        std::cout << "num_sectors: " << num_sectors << std::endl;
        std::cout << "min_h: " << min_h << std::endl;
        std::cout << "max_h: " << max_h << std::endl;
        std::cout << "scan_ratio_threshold: " << scan_ratio_threshold << std::endl;
        std::cout << "th_bin_max_h: " << th_bin_max_h << std::endl;
        std::cout << "minimum_num_pts: " << minimum_num_pts << std::endl;
        std::cout << "rejection_ratio: " << rejection_ratio << std::endl;
        std::cout << th_dist_ << std::endl;
        std::cout << iter_groundfilter_ << std::endl;
        std::cout << num_lprs_ << std::endl;
        std::cout << th_seeds_heights_ << std::endl;
        std::cout << "-----------------------" << std::endl;
        // Initalization of R-POD
        init(r_pod_map);
        init(r_pod_curr);
        init(r_pod_selected);

        piecewise_ground_.reserve(130000);
        non_ground_.reserve(130000);
        ground_pc_.reserve(130000);
        non_ground_pc_.reserve(130000);
    }
    ERASOR();

    ~ERASOR();

    // Inputs: transformed & cut pcs
    void set_inputs(
            const pcl::PointCloud<pcl::PointXYZI> &map_voi,
            const pcl::PointCloud<pcl::PointXYZI> &query_voi);

    void compare_then_select(int frame);

    void get_pcs(
            pcl::PointCloud<pcl::PointXYZI> &arranged,
            pcl::PointCloud<pcl::PointXYZI> &complement);

    // -------------------------------
    // Version 2 algorithm!
    void compare_vois_and_revert_ground(int frame);

    void get_static_estimate(
            pcl::PointCloud<pcl::PointXYZI> &arranged,
            pcl::PointCloud<pcl::PointXYZI> &complement);

    pcl::PointCloud<pcl::PointXYZI> ground_viz; // Visualized in pcs_v2!
    // -------------------------------
    // Version 3 algorithm!
    void compare_vois_and_revert_ground_w_block(int frame);

    bool is_dynamic_obj_close(R_POD &r_pod_selected, int r_target, int theta_target, int r_range, int theta_range);

    // -------------------------------
    void get_outliers(
            pcl::PointCloud<pcl::PointXYZI> &map_rejected,
            pcl::PointCloud<pcl::PointXYZI> &curr_rejected);

    pcl::PointCloud<pcl::PointXYZI> debug_curr_rejected;
    pcl::PointCloud<pcl::PointXYZI> debug_map_rejected;
    pcl::PointCloud<pcl::PointXYZI> map_complement;

    R_POD r_pod_map; // R_POD of Map
    R_POD r_pod_curr; // R_POD of current pointcloud
    R_POD r_pod_selected; // R_POD of current pointcloud

    double get_max_range();

private:
    std::vector<int> DYNAMIC_CLASSES = {252, 253, 254, 255, 256, 257, 258, 259};

    ros::NodeHandle nh;
    ros::Publisher  pub_map_rejected;
    ros::Publisher  pub_curr_rejected;
    ros::Publisher  pub_map_init, pub_curr_init;
    ros::Publisher  pub_map_marker, pub_curr_marker, pub_viz_bin_marker;
    ros::Publisher  pub_ground, pub_arranged;

    double max_r;
    int    num_rings;
    int    num_sectors;
    int    num_lowest_pts;

    double ring_size;
    double sector_size;

    double min_h;
    double max_h;
    double scan_ratio_threshold;
    double th_bin_max_h;
    int    minimum_num_pts;
    double rejection_ratio;
    double map_voxel_size_;

    double xy2theta(const double &x, const double &y);

    double xy2radius(const double &x, const double &y);

    void init(R_POD &r_pod);

    void clear_bin(Bin &bin);

    void clear(pcl::PointCloud<pcl::PointXYZI> &pt_cloud);

    pcl::PointCloud<pcl::PointXYZI> piecewise_ground_, non_ground_;
    pcl::PointCloud<pcl::PointXYZI> ground_pc_, non_ground_pc_;

    void pt2r_pod(const pcl::PointXYZI &pt, Bin &bin);

    void voi2r_pod(const pcl::PointCloud<pcl::PointXYZI> &src, R_POD &r_pod);

    void voi2r_pod(
            const pcl::PointCloud<pcl::PointXYZI> &src, R_POD &r_pod,
            pcl::PointCloud<pcl::PointXYZI> &complement);

    void viz_pseudo_occupancy();


    // ------ Ground extraction ------------------------
    double th_dist_; // params!
    int    iter_groundfilter_; // params!
    int    num_lprs_;
    double th_seeds_heights_;

    Eigen::MatrixXf normal_;
    double          th_dist_d_, d_;

    void estimate_plane_(const pcl::PointCloud<pcl::PointXYZI> &ground);

    void extract_initial_seeds_(
            const pcl::PointCloud<pcl::PointXYZI> &p_sorted,
            pcl::PointCloud<pcl::PointXYZI> &init_seeds);

    void extract_ground(
            const pcl::PointCloud<pcl::PointXYZI> &src,
            pcl::PointCloud<pcl::PointXYZI> &dst,
            pcl::PointCloud<pcl::PointXYZI> &outliers);


    bool has_dynamic(Bin &bin);

    void merge_bins(const Bin &src1, const Bin &src2, Bin &dst);

    void r_pod2pc(const R_POD &sc, pcl::PointCloud<pcl::PointXYZI> &pc);


    geometry_msgs::PolygonStamped set_polygons(int r_idx, int theta_idx, int num_split = 3);
};


