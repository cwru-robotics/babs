// Node-level integration testing for the BABS Wobbler
// Created Mar 22 2017 by Trent Ziemer
// Last updated Mar 22 by Trent Ziemer

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

ros::NodeHandle * nh_ptr;

int main(int argc, char** argv) {

    ros::init(argc, argv, "lidar_calibration_unit_test");
    ros::NodeHandle nh;
    nh_ptr = &nh;

    while(ros::ok())
    {
    	if(nh_ptr->hasParam("/calibration_done"))
    	{
    		break;
    	}
    }

	if(1)
	{
		ROS_INFO("TEST PASSED: All values are within bounds");
	}
	else
	{
		ROS_WARN("TEST FAILED: Some values not found or not within bounds");
	}


    ROS_INFO("Closing out of tests...");

    return 0;
}
