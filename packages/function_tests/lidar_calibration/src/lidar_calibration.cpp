// LIDAR Transformer node for babs_lidar_wobbler.
// Pulled from a completed homework assignment for Modern Robotics Programming by Trent Ziemer, heavily based on a minimal node written by Dr. Wyatt Newman.
// Original node name was lidar_transformer. This is ...2 because it's just a variant on that node from a different package.

#include <math.h>
#include <stdlib.h>
#include <string>
#include <vector>
#include <ros/ros.h>

#include <std_msgs/Int16.h>

#include <tf/transform_listener.h>
#include <xform_utils/xform_utils.h>
#include <sensor_msgs/LaserScan.h>

// Trents Point cloud
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

using namespace std;

// These are global pointers
ros::NodeHandle * nh_ptr;
pcl::PointCloud<pcl::PointXYZ> cloud;
ros::Publisher * pubCloud_ptr;
tf::TransformListener *g_listener_ptr; //a transform listener
XformUtils xformUtils; //instantiate an object of XformUtils
vector <Eigen::Vector3d> g_pt_vecs_wrt_lidar_frame; //will hold 3-D points in LIDAR frame
vector <Eigen::Vector3d> g_pt_vecs_wrt_world_frame; //will hold 3_D points in world frame
ros::Publisher *pub_ptr;

// These are global variables used as parameters for the wobblers angle
double wobbler_angle;
double scanning_upwards;
double last_wobbler_angle;

// This callback function is called whenever we receive a laser scan message
//  It will publish the scan as a point cloud. This cloud is a 2D slice of the final point cloud that results from the wobbler sweeping and getting stitched together.
void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan_in) {
   
}

// Program starting point.
int main(int argc, char** argv) {
    ros::init(argc, argv, "lidar_wobbler_transformer");
    ros::NodeHandle nh;
    nh_ptr = &nh;

    ros::Publisher pub = nh.advertise<sensor_msgs::PointCloud2> ("scan_cloud", 1);

    pub_ptr = &pub;

    g_listener_ptr = new tf::TransformListener;
    tf::StampedTransform stfLidar2World;
    bool tferr = true;
    ROS_INFO("trying to get tf of lidar_link w/rt world: ");
    //topic /scan has lidar data in frame_id: lidar_link
    while (tferr) {
        tferr = false;
        try {
            g_listener_ptr->lookupTransform("lidar_link", "laser", ros::Time(0), stfLidar2World);
        } catch (tf::TransformException &exception) {

            ROS_WARN("%s; retrying...", exception.what());
            tferr = true;
            ros::Duration(0.5).sleep(); // sleep for half a second
            ros::spinOnce();

        }
    }
    ROS_INFO("transform received; ready to process lidar scans");

    ros::Subscriber lidar_subscriber = nh.subscribe("scan", 1, scanCallback);

    ros::spin();

    return 0;
}
