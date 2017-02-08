#include <ros/ros.h>
#include <std_msgs/Int16.h>
#include <math.h>

// Shamelessly copied from a WSN class package's code (sine commander)

int main(int argc, char **argv) {
    ros::init(argc, argv, "dynamixel_wobbler_commander"); 
    ros::NodeHandle node; 

    // Each motor has it's own ID
    int motor_id = 1;

    char cmd_topic_name[50];
    sprintf(cmd_topic_name,"dynamixel_motor%d_cmd",motor_id);
    ROS_INFO("Using command topic: %s",cmd_topic_name);

    ros::Publisher dyn_pub = node.advertise<std_msgs::Int16>(cmd_topic_name, 1);
    
    std_msgs::Int16 int_angle; 
   double dt = 0.01; // repeat at freq 1/dt
   ros::Rate naptime(1/dt); //create a ros object from the ros “Rate” class; 

    int_angle.data = 0.0;

    short int start_ang = 1000;
    short int int_ang = start_ang;

    bool increasing = true;
    int loop_counter = 0;
    // The lowest and highest angles to go to
    double min_ang;
    double max_ang;
    // How much to icnrease the angle command by each iteration. This controls the wobblers angular speed (rad/s) as a proxy variable (numbers/iteration)
    double change_ang;

    
    // do work here in infinite loop (desired for this example)
    while (ros::ok()) 
    {
        // Only check params every X loops
        if (loop_counter % 10 == 0)
        {

            if(!node.getParam("min_angle", min_ang))
            {
                min_ang = 500;
            }


            if(!node.getParam("max_angle", max_ang))
            {
                max_ang = 1000;  
            }


            if(!node.getParam("wobble_speed", change_ang))
            {
                change_ang = 2;
            }
        }
        loop_counter++;

min_ang = (short int)min_ang;
max_ang = (short int)max_ang;
change_ang = (short int)change_ang;

if(increasing == true)
{
	int_ang = int_ang + change_ang;
}
else
{
    int_ang = int_ang - change_ang;
}

if(int_ang >= max_ang)
{
	increasing = false;
}
if(int_ang <= min_ang)
{
	increasing = true;
}

        int_angle.data = int_ang;
        ROS_INFO("sending: %d",int_ang);
        dyn_pub.publish(int_angle); // publish the value--of type Float64-- 
	naptime.sleep(); 
    }
}

