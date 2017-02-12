#include <ros/ros.h>
#include <std_msgs/Int16.h>
#include <math.h>

// Shamelessly copied from a WSN class package's code (sine commander) and heavily modified
// By TZ

int main(int argc, char **argv) {
    ros::init(argc, argv, "dynamixel_motor_wobble"); 
    ros::NodeHandle node; 

/* Dude, FUCK name mangling
    char front_cmd_topic_name[50];
    char rear_cmd_topic_name[50];
    sprintf(front_cmd_topic_name,"motor%d_cmd", front_motor_id);
    ROS_INFO("Using front command topic: %s",front_cmd_topic_name);

    sprintf(rear_cmd_topic_name,"motor%d_cmd", rear_motor_id);
    ROS_INFO("And rear command topic: %s",rear_cmd_topic_name);
*/

    ros::Publisher front_motor_pub = node.advertise<std_msgs::Int16>("front_wobbler/cmd", 1);

    ros::Publisher rear_motor_pub = node.advertise<std_msgs::Int16>("rear_wobbler/cmd", 1);
    
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
    //ROS_INFO("sending: %d",int_ang);
    // Move both motors by the same angle, for convenienve and simplicity
    front_motor_pub.publish(int_angle); 
    rear_motor_pub.publish(int_angle);
    naptime.sleep(); 
    }
}

