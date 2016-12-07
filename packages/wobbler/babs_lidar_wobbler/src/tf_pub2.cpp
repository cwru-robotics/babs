#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include<std_msgs/Int16.h> 

// Transform publisher between the "wobbler_joint" frame (the location of the axis of rotation of the wobbler), 
//    and the "wobbler_joint", which is the axis of rotation of the front wobbler.

// Shamelessly stolen from some tutorial

double motorEncodedAngle;

double factor;
double factor2;

// Gives the best idea of the transform between the babs lidar link frame,
//  and the location of the frame. Static transform.
ros::NodeHandle * node_ptr;

void poseCallback(const std_msgs::Int16& msg){
	static tf::TransformBroadcaster br;
	tf::Transform transform;
	tf::Quaternion q;

	motorEncodedAngle = msg.data;

	// Distance between wobbler axis of rotation and laser source is about 0.0889m, constant.
	transform.setOrigin( tf::Vector3(0.0, 0.0, 0.0889) ); 

    if(!node_ptr->getParam("factor", factor))
    {
        ROS_WARN("Wobbler point cloud could not find wobbler minimum angle (min_ang) on param server. Using arbitrary value.");
        factor = 1;
    }
    if(!node_ptr->getParam("factor2", factor2))
    {
        ROS_WARN("Wobbler point cloud could not find wobbler minimum angle (min_ang) on param server. Using arbitrary value.");
        factor2 = 0.5;
    }

	q.setRPY(0, (motorEncodedAngle*factor + 1024)/1303.8/factor2, 0);
	
	transform.setRotation(q);
	br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "wobbler_joint", "laser"));
}

int main(int argc, char** argv){
	ros::init(argc, argv, "my_tf_b2");

	ros::NodeHandle node;
	node_ptr = &node;
	ros::Subscriber sub = node.subscribe("/dynamixel_motor1_ang", 10, &poseCallback);

	ros::spin();
	return 0;
};
