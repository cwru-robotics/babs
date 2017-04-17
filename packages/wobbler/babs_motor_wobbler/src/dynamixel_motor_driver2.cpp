// Trent Ziemer 2/8/17 for Sentry Wobbler
// Based on basic given WSN Low-level C code for Dynamixel motor communication with ROS nodes

#include<ros/ros.h> 
#include<std_msgs/Int16.h> 
#include <linux/serial.h>
#include <termios.h>

// Default settings
#define DEFAULT_BAUDNUM		1 // code "1" --> 1Mbps
#define DEFAULT_ID		1 //this is the motor ID
#define DEFAULT_TTY_NUM			0 // typically, 0 for /dev/ttyUSB0

// Prevent C++ name mangling of Dynamixels C header and C source files that we want to use
extern "C" { 
  int send_dynamixel_goal(short int motor_id, short int goalval); 
  int open_dxl(int deviceIndex, int baudnum);
  //make these global, so connection info is not lost after fnc call
  char	gDeviceName[20];
  struct termios newtio;
  struct serial_struct serinfo;
  char dev_name[100] = {0, };
  void dxl_terminate(void);
  short int read_position(short int motor_id);
}

//globals:
short int motor_id = DEFAULT_ID;
std::vector<short int> motor_ids;
short int baudnum = DEFAULT_BAUDNUM;
int ttynum = DEFAULT_TTY_NUM;
short int g_goal_angle=0;

int g_front_motor_id;
int g_rear_motor_id;
short int g_front_goal_angle=0;
short int g_rear_goal_angle=0;

void sendFrontMotorCmd(const std_msgs::Int16& goal_angle_msg) 
{ 
  g_front_goal_angle = goal_angle_msg.data; // for use by main()
  send_dynamixel_goal(g_front_motor_id,goal_angle_msg.data); // When we receive a command message from the cmd topic, send that out on serial line using dynamixel code
} 

void sendRearMotorCmd(const std_msgs::Int16& goal_angle_msg) 
{ 
  g_rear_goal_angle = goal_angle_msg.data; // for use by main()
  send_dynamixel_goal(g_rear_motor_id,goal_angle_msg.data); // When we receive a command message from the cmd topic, send that out on serial line using dynamixel code
} 

int main(int argc, char **argv) 
{
  ros::init(argc,argv, "dynamixel_motor_driver2"); //name this node 

  ros::NodeHandle node; // need this to establish communications with our new node 
  ros::Publisher pub_jnt = node.advertise<std_msgs::Int16>("front_wobbler/angle", 1);
  //ros::Publisher pub_jnt2 = node.advertise<std_msgs::Int16>("rear_wobbler/angle", 1);
  
  ros::Subscriber subscriber = node.subscribe("front_wobbler/cmd", 1, sendFrontMotorCmd); 
  //ros::Subscriber subscriber2 = node.subscribe("rear_wobbler/cmd", 1, sendRearMotorCmd); 

  // A good frequency at which to read motor position and publish that to the controller is 100 Hz
  double dt = 0.01;

  // Open appropriate USB port that has the dynamixel device in it
  ROS_INFO("Attempting to open /dev/ttyUSB%d",999);
  bool open_success = open_dxl(999,34);

  // If we fail to open the device file, warn user and exit
  if (!open_success) {
    ROS_WARN("Could not open /dev/ttyUSB%d; check permissions?",999);
    return 0;
  }

  // ROS msgs for each wobbler command
  std_msgs::Int16 front_motor_angle;
  //std_msgs::Int16 rear_motor_angle;

  unsigned short front_motor_ang = 0;
 // short int rear_motor_ang = 0;

  ROS_INFO("Attempting communication with following motors:");
  ROS_INFO("  @ motor_id %d at baudrate code %d",0, 34);
  //ROS_INFO("  @ motor_id %d at baudrate code %d",g_rear_motor_id, motor_baud);

  // Main program loop to read motor positions from device and push them to ROS topics for each motor
  while(ros::ok()) 
  {
    // For each of the motors that we have...

    // Use Dynamixel library to read the particular motors current position and store it
    front_motor_ang = read_position(0);

    //rear_motor_ang = read_position(g_rear_motor_id);

    // Check if the received value is bad, holdover from older times...it may be important to make a note of for debugging
    if (front_motor_ang>=4096) 
    {
      ROS_WARN("Extremely likely read error from front Dynamixel: angular value of %d at cmd %d, ignoring?",front_motor_ang, g_front_goal_angle);
    }

    // Load read positions into ROS messages for publishing to topics
    front_motor_angle.data = front_motor_ang;
   //rear_motor_angle.data = rear_motor_ang;

    pub_jnt.publish(front_motor_angle);
   // pub_jnt2.publish(rear_motor_angle);

    // Take a break, you've had a long day...
    ros::Duration(dt).sleep();
    ros::spinOnce();
  }
   
  // Should not happen until ROS cuts out entirely
  dxl_terminate();
  return 0; 
} 
