#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/Int16.h> 

// Transform publisher between the "lidar_link" (the 2D scanning SICK LIDAR), 
//    and the "wobbler_joint", which is the axis of rotation of the front wobbler.

// Shamelessly stolen from some tutorial

double motorEncodedAngle;
ros::NodeHandle * node_ptr;

// Gives the best idea of the transform between the babs lidar link frame,
//  and the location of the frame. Static transform. Needs updated.
void poseCallback(const std_msgs::Int16& msg){
	static tf::TransformBroadcaster br;
	tf::Transform transform;
	tf::Quaternion q;

	motorEncodedAngle = msg.data;

	// Get transform parameters from server.
	double dist_to_joint_x;
	if(!node_ptr->getParam("dist_to_joint_x", dist_to_joint_x))
	{
		// "Good guess" values if we can't find a stored param.
		dist_to_joint_x = -0.14;
	}

	double dist_to_joint_y;
	if(!node_ptr->getParam("dist_to_joint_y", dist_to_joint_y))
	{
		// "Good guess" values if we can't find a stored param.
		dist_to_joint_y = 0.0;
	}

	double dist_to_joint_z;
	if(!node_ptr->getParam("dist_to_joint_z", dist_to_joint_z))
	{
		// "Good guess" values if we can't find a stored param.
		dist_to_joint_z = 0.634925;
	}

	double rot_to_joint_r;
	if(!node_ptr->getParam("rot_to_joint_r", rot_to_joint_r))
	{
		// "Good guess" values if we can't find a stored param.
		rot_to_joint_r = 0.0;
	}

	double rot_to_joint_p;
	if(!node_ptr->getParam("rot_to_joint_p", rot_to_joint_p))
	{
		// "Good guess" values if we can't find a stored param.
		rot_to_joint_p = 0.0;
	}

	double rot_to_joint_y;
	if(!node_ptr->getParam("rot_to_joint_y", rot_to_joint_y))
	{
		// "Good guess" values if we can't find a stored param.
		rot_to_joint_y = 3.14159265359; // = pi
	}

	transform.setOrigin( tf::Vector3(dist_to_joint_x, dist_to_joint_y, dist_to_joint_z) ); 
	q.setRPY(rot_to_joint_r, rot_to_joint_p, rot_to_joint_y);
	
	transform.setRotation(q);
	br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "lidar_link", "wobbler_joint"));
}

int main(int argc, char** argv){
	ros::init(argc, argv, "wobbler_joint_static_tf");

	ros::NodeHandle node;
	node_ptr = &node;
	ros::Subscriber sub = node.subscribe("/dynamixel_motor1_ang", 10, &poseCallback);

	ros::spin();
	return 0;
};
