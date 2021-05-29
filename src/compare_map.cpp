#include "tools/erasor_utils.hpp"

using namespace std;
int main(int argc, char **argv)
{
    ros::init(argc, argv, "mapviz");
    std::cout<<"KiTTI MAPVIZ STARTED"<<std::endl;


    std::string rawName, octoMapName, pplName, removertName, erasorName;
    std::string ABS_DIR, SEQ;

    ros::NodeHandle nodeHandler;
    nodeHandler.param<std::string>("/abs_dir", ABS_DIR, "/home/shapelim");
    nodeHandler.param<std::string>("/seq", SEQ, "/home/shapelim");

    cout<<"Abs. directory: "<<ABS_DIR<<endl;
    cout<<"Target sequence: \033[1;32m"<<SEQ<<"\033[0m"<<endl;
    rawName = ABS_DIR + "/gt/" + SEQ + "_voxel_0_2.pcd";
    octoMapName = ABS_DIR + "/estimate/" + SEQ + "_octomap.pcd";
    pplName = ABS_DIR + "/estimate/" + SEQ + "_pplremover.pcd";
    removertName = ABS_DIR + "/estimate/" + SEQ + "_Removert_rv1.pcd";
    erasorName = ABS_DIR + "/estimate/" + SEQ + "_ERASOR.pcd";

    ros::Publisher msPublisher = nodeHandler.advertise<sensor_msgs::PointCloud2>("/map/static", 100);
    ros::Publisher mdPublisher = nodeHandler.advertise<sensor_msgs::PointCloud2>("/map/dynamic", 100);

    ros::Publisher osPublisher = nodeHandler.advertise<sensor_msgs::PointCloud2>("/octomap/static", 100);
    ros::Publisher odPublisher = nodeHandler.advertise<sensor_msgs::PointCloud2>("/octomap/dynamic", 100);

    ros::Publisher psPublisher = nodeHandler.advertise<sensor_msgs::PointCloud2>("/pplremover/static", 100);
    ros::Publisher pdPublisher = nodeHandler.advertise<sensor_msgs::PointCloud2>("/pplremover/dynamic", 100);

    ros::Publisher rsPublisher = nodeHandler.advertise<sensor_msgs::PointCloud2>("/removert/static", 100);
    ros::Publisher rdPublisher = nodeHandler.advertise<sensor_msgs::PointCloud2>("/removert/dynamic", 100);

    ros::Publisher esPublisher = nodeHandler.advertise<sensor_msgs::PointCloud2>("/erasor/static", 100);
    ros::Publisher edPublisher = nodeHandler.advertise<sensor_msgs::PointCloud2>("/erasor/dynamic", 100);


    ////////////////////////////////////////////////////////////////////
    std::cout<<"\033[1;32mLoading map..."<<std::endl;
    // load original src
    pcl::PointCloud<pcl::PointXYZI>::Ptr ptrSrc(new pcl::PointCloud<pcl::PointXYZI>);
    erasor_utils::load_pcd(rawName, ptrSrc);

    // Removert
    pcl::PointCloud<pcl::PointXYZI>::Ptr ptrOcto(new pcl::PointCloud<pcl::PointXYZI>);
    erasor_utils::load_pcd(octoMapName, ptrOcto);

    // pplremover
    pcl::PointCloud<pcl::PointXYZI>::Ptr ptrPpl(new pcl::PointCloud<pcl::PointXYZI>);
    erasor_utils::load_pcd(pplName, ptrPpl);

    // Removert
    pcl::PointCloud<pcl::PointXYZI>::Ptr ptrRemovert(new pcl::PointCloud<pcl::PointXYZI>);
    erasor_utils::load_pcd(removertName, ptrRemovert);

    // Erasor
    pcl::PointCloud<pcl::PointXYZI>::Ptr ptrErasor(new pcl::PointCloud<pcl::PointXYZI>);
    erasor_utils::load_pcd(erasorName, ptrErasor);
    std::cout<<"\033[1;32mLoad complete \033[0m"<<std::endl;

    ////////////////////////////////////////////////////////////////////
    pcl::PointCloud<pcl::PointXYZI> mapStatic, mapDynamic;
    pcl::PointCloud<pcl::PointXYZI> octoStatic, octoDynamic;
    pcl::PointCloud<pcl::PointXYZI> pplStatic, pplDynamic;
    pcl::PointCloud<pcl::PointXYZI> removertStatic, removertDynamic;
    pcl::PointCloud<pcl::PointXYZI> erasorStatic, erasorDynamic;

    erasor_utils::parse_dynamic_obj(*ptrErasor, erasorDynamic, erasorStatic);
    erasor_utils::parse_dynamic_obj(*ptrRemovert, removertDynamic, removertStatic);
    erasor_utils::parse_dynamic_obj(*ptrPpl, pplDynamic, pplStatic);
    erasor_utils::parse_dynamic_obj(*ptrOcto, octoDynamic, octoStatic);
    erasor_utils::parse_dynamic_obj(*ptrSrc, mapDynamic, mapStatic);

    auto esmsg = erasor_utils::cloud2msg(erasorStatic);
    auto edmsg = erasor_utils::cloud2msg(erasorDynamic);
    auto rsmsg = erasor_utils::cloud2msg(removertStatic);
    auto rdmsg = erasor_utils::cloud2msg(removertDynamic);
    auto psmsg = erasor_utils::cloud2msg(pplStatic);
    auto pdmsg = erasor_utils::cloud2msg(pplDynamic);
    auto osmsg = erasor_utils::cloud2msg(octoStatic);
    auto odmsg = erasor_utils::cloud2msg(octoDynamic);
    auto msmsg = erasor_utils::cloud2msg(mapStatic);
    auto mdmsg = erasor_utils::cloud2msg(mapDynamic);

    ros::Rate loop_rate(2);
    static int count_ = 0;
    while (ros::ok())
    {
      esPublisher.publish(esmsg);       edPublisher.publish(edmsg);
      rsPublisher.publish(rsmsg);       rdPublisher.publish(rdmsg);
      psPublisher.publish(psmsg);       pdPublisher.publish(pdmsg);
      osPublisher.publish(osmsg);       odPublisher.publish(odmsg);
      msPublisher.publish(msmsg);       mdPublisher.publish(mdmsg);

      if (++count_ %2 == 0) std::cout<<"On "<<count_<<"th publish!"<<std::endl;

      ros::spinOnce();
      loop_rate.sleep();
    }
    return 0;
}
