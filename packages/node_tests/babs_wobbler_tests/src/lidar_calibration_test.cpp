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
using namespace std;
ros::NodeHandle * nh_ptr;

// These are global variables used as parameters for the wobblers angle
double wobbler_angle;
double scanning_upwards;
double last_wobbler_angle;

// This callback function is called whenever we receive a laser scan message
//  It will publish the scan as a point cloud. This cloud is a 2D slice of the final point cloud that results from the wobbler sweeping and getting stitched together.
void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan_in) 
{

}

bool checkCalibVal(std::string calibValName, float true_val)
{
    float val; // Input from parameter server
    float min_val; // parameter
    float max_val; // parameter
    float error_bound = 0.05;

    if (nh_ptr->getParam("/lidar_calibration_test/" + calibValName, val))
    {
    	ROS_INFO("Found a %s distance of %f", calibValName.c_str(), val);
    	min_val = true_val - (true_val * error_bound);
    	max_val = true_val + (true_val * error_bound);
    	if ((val > min_val) && (val < max_val) || (val < min_val) && (val > max_val) )
    	{
	    	ROS_INFO("and it is within acceptable bounds!");
	    	return true;
    	}
    	else
    	{
    		//ROS_INFO("min val: %f, val: %f, max val: %f.", min_val, val, max_val);
    		ROS_INFO("...but it is not within acceptable bounds.");
    	}
    }
    else
    {
    	ROS_WARN("No %s parameter found, whats up with that?", calibValName.c_str());
    }
    return false;
}

int main(int argc, char** argv) {

    ros::init(argc, argv, "lidar_calibration_unit_test");
    ros::NodeHandle nh;
    nh_ptr = &nh;


    // That node should compare a set of SICK and wobbler point clouds (2D) from topics and give transform parameters on the pserver, and identify the lengths of test fixture walls
    // Test fixture: \_/ , with lidars looking at it from "below"

    ROS_INFO("Starting lidar 2d-3d calibration test.");

    ROS_INFO("Waiting for calibration to complete");

    // Maximum error fraction that is considered acceptable 
    float error_bound = 0.05;

    float true_x_dist = 1.6; // Actual physical distance
    float true_y_dist = 5.63; // Actual physical distance
    float true_z_dist = -0.63; // Actual physical distance
    float true_roll = 0.05011123; // Actual physical distance
    float true_pitch = 1.12; // Actual physical distance
    float true_yaw = -0.1511; // Actual physical distance

    while(ros::ok())
    {
    	if(nh_ptr->hasParam("/calibration_done"))
    	{
    		break;
    	}
    	else
    	{
    		ROS_INFO("waiting for calibration to complete...");
    	}
    }

	if( checkCalibVal("x_dist", true_x_dist) &&
		checkCalibVal("y_dist", true_y_dist) &&
		checkCalibVal("z_dist", true_z_dist) &&
		checkCalibVal("roll", true_roll) &&
		checkCalibVal("pitch", true_pitch) &&
		checkCalibVal("yaw", true_yaw))
	{
		ROS_INFO("TEST PASSED: All values are within bounds");
	}
	else
	{
		ROS_WARN("TEST FAILED: Some values not found or not within bounds");
	}

    //ros::Publisher pub = nh.advertise<sensor_msgs::PointCloud2> ("scan_cloud", 1);
   
    //ros::Subscriber lidar_subscriber = nh.subscribe("scan", 1, scanCallback);

    ROS_INFO("Closing out of tests...");

    return 0;
}
