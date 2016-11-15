#include <ros/ros.h>
#include <stdlib.h>
#include <math.h>
#include <sensor_msgs/PointCloud2.h> 
#include <pcl_ros/point_cloud.h> //to convert between PCL and ROS
#include <pcl/conversions.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
//#include <pcl/PCLPointCloud2.h> //PCL is migrating to PointCloud2 
#include <pcl/common/common_headers.h>
#include <pcl-1.7/pcl/point_cloud.h>
#include <pcl-1.7/pcl/PCLHeader.h>

int counter;


void take_snapshot(const sensor_msgs::PointCloud2& cloud) {
	
	pcl::PointCloud< pcl::PointXYZ > PointCloudXYZ;
	pcl::fromROSMsg(cloud,PointCloudXYZ);
	ROS_INFO("width: %d", PointCloudXYZ.width);
	ROS_INFO("height: %d", PointCloudXYZ.height);

	int cloudsize = (PointCloudXYZ.width) * (PointCloudXYZ.height);

	for (int i=0; i< cloudsize; i++) {

		ROS_INFO("(x,y,z) = %f,%f,%f", PointCloudXYZ.points[i].x,PointCloudXYZ.points[i].y,PointCloudXYZ.points[i].z);
		std::cout << "(x,y,z) = " << PointCloudXYZ.points[i] << std::endl;




	}

}


void pointCloudCallback(const sensor_msgs::PointCloud2& cloud) {


	if (counter%200 == 0) {
		take_snapshot(cloud);
	}

	counter++;
}

int main(int argc, char** argv){

	counter = 0;  

	ROS_WARN("running data collection loop!");
	//TODO spawn in random spot
	//check accerometer, speed, collisions to see if in a valid pose (aka upright, not colliding)
	//save point cloud
	//attempt to drive a meter or something, while checking for tipping, failure to progress, collision, etc 
	//determine whether successful, saved pt cloud success label together as a pair

	ros::init(argc, argv, "data_collection");

	ros::NodeHandle n;
	ros::Subscriber pcl_subscriber = n.subscribe("/kinect/depth/points", 1, pointCloudCallback);



	ros::spin();
	return 0;
}
