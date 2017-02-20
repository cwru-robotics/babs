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
    ros::init(argc, argv, "lidar_calibration");
    ros::NodeHandle nh;
    nh_ptr = &nh;

    float x_dist = 99;
    float y_dist = 99;
    float z_dist = 99;
    float roll = 99;
    float pitch = 99;
    float yaw = 99;

    bool calibration_done = true;

    nh_ptr->setParam("/calibration_done", calibration_done);

    nh_ptr->setParam("/lidar_calibration_test/x_dist", x_dist);
    nh_ptr->setParam("/lidar_calibration_test/y_dist", y_dist);
    nh_ptr->setParam("/lidar_calibration_test/z_dist", z_dist);
    nh_ptr->setParam("/lidar_calibration_test/roll", roll);
    nh_ptr->setParam("/lidar_calibration_test/pitch", pitch);
    nh_ptr->setParam("/lidar_calibration_test/yaw", yaw);
    
    ros::spin();
    return 0;
}
