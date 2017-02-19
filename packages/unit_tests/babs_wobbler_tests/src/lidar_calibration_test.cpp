
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

using namespace std;


// These are global variables used as parameters for the wobblers angle
double wobbler_angle;
double scanning_upwards;
double last_wobbler_angle;

// This callback function is called whenever we receive a laser scan message
//  It will publish the scan as a point cloud. This cloud is a 2D slice of the final point cloud that results from the wobbler sweeping and getting stitched together.
void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan_in) {
   



}

int main(int argc, char** argv) {

    ros::init(argc, argv, "lidar_calibration_unit_test");
    ros::NodeHandle nh;
    nh_ptr = &nh;

    // That node should compare a set of SICK and wobbler point clouds (2D) and 
    ROS_INFO("Starting lidar calibration test. ")

    //ros::Publisher pub = nh.advertise<sensor_msgs::PointCloud2> ("scan_cloud", 1);
   
    ros::Subscriber lidar_subscriber = nh.subscribe("scan", 1, scanCallback);

    ros::spin();

    return 0;
}
