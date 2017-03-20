// Created Feb 19th ish by Trent Ziemer
#include <math.h>
#include <stdlib.h>
#include <string>
#include <vector>
#include <ros/ros.h>
#include <std_msgs/Int16.h>
#include <tf/transform_listener.h>
#include <xform_utils/xform_utils.h>
#include <sensor_msgs/LaserScan.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

// These are global pointers
ros::NodeHandle * nh_ptr;
pcl::PointCloud<pcl::PointXYZ> cloud;
ros::Publisher * pubCloud_ptr;
tf::TransformListener *g_listener_ptr; //a transform listener
XformUtils xformUtils; //instantiate an object of XformUtils
std::vector <Eigen::Vector3d> g_pt_vecs_wrt_lidar_frame; //will hold 3-D points in LIDAR frame
std::vector <Eigen::Vector3d> g_pt_vecs_wrt_world_frame; //will hold 3_D points in world frame
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
    ros::init(argc, argv, "lidar_calibration");
    ros::NodeHandle nh;
    nh_ptr = &nh;

    float x_dist = 1.6;
    float y_dist = 5.63;
    float z_dist = -0.63;
    float roll = 0.05011123;
    float pitch = 1.12;
    float yaw = -0.1511;







    bool calibration_done = true;

    nh_ptr->setParam("/lidar_calibration_test/x_dist", x_dist);
    nh_ptr->setParam("/lidar_calibration_test/y_dist", y_dist);
    nh_ptr->setParam("/lidar_calibration_test/z_dist", z_dist);
    nh_ptr->setParam("/lidar_calibration_test/roll", roll);
    nh_ptr->setParam("/lidar_calibration_test/pitch", pitch);
    nh_ptr->setParam("/lidar_calibration_test/yaw", yaw);
    
    nh_ptr->setParam("/calibration_done", calibration_done);

    ros::spin();
    return 0;
}
