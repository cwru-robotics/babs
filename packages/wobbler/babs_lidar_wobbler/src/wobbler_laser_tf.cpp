#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/Int16.h> 

// Transform publisher between the "wobbler_joint" frame (the location of the axis of rotation of the wobbler), 
//    and "laser", which is the approximate location of the source of the wobbling LIDAR laser.

// Shamelessly stolen from some tutorial - TZ

double motorEncodedAngle;

double factor;
double factor2;

std::string name;

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

	// Magic numbers that you CAN change if needed, but should NOT change unless you empirically think it's needed.
    if(!node_ptr->getParam("factor", factor))
    {
        ROS_WARN("Using arbitrary value1.");
        factor = 1;
    }
    if(!node_ptr->getParam("factor2", factor2))
    {
        ROS_WARN("Using arbitrary value2.");
        factor2 = 0.5;
    }

	q.setRPY(0, (motorEncodedAngle*factor + 1024)/1303.8/factor2, 0);
	
	transform.setRotation(q);
	br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), name + "wobbler_joint", name + "wobbler_laser"));
}

int main(int argc, char** argv){
	ros::init(argc, argv, "my_tf_b2");

	ros::NodeHandle node("~");
	node_ptr = &node;

	ros::Subscriber sub = node.subscribe("angle", 1, &poseCallback);

	ros::spin();
	return 0;
};
