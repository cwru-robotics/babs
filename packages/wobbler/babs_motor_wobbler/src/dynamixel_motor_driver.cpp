// Trent Ziemer 2/8/17 for sentry wobbler
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

void dynamixelCB(const std_msgs::Int16& goal_angle_msg) 
{ 
  short int goal_angle = goal_angle_msg.data;
  g_goal_angle = goal_angle; // for use by main()
     send_dynamixel_goal(motor_id,goal_angle);
} 

int main(int argc, char **argv) 
{ 
  std::string dash_m = "-m";
  std::string dash_tty = "-tty";
  std::string dash_baud = "-baud";

  char node_name[50];
  char in_topic_name[50];
  char out_topic_name[50];
  sprintf(node_name,"motor%d",motor_id);
  ROS_INFO("node name: %s",node_name);
  sprintf(in_topic_name,"motor%d_cmd",motor_id);
  ROS_INFO("input command topic: %s",in_topic_name);
  sprintf(out_topic_name,"motor%d_ang",motor_id);
  ROS_INFO("output topic: %s",out_topic_name);

  ros::init(argc,argv,node_name); //name this node 

  ros::NodeHandle node; // need this to establish communications with our new node 
  ros::Publisher pub_jnt = node.advertise<std_msgs::Int16>(out_topic_name, 1);
  

  bool use_cmd_line_args = false;
  
  if (use_cmd_line_args == true)
  {
    if (argc<2) {
      ROS_INFO("using default motor_id %d, baud code %d, via /dev/ttyUSB%d",motor_id,baudnum,ttynum);
      ROS_INFO("may run with command args, e.g.: -m 2 -tty 1 for motor_id=2 on /dev/ttyUSB1");
    }
    else {
      std::vector <std::string> sources;
      for (int i = 1; i < argc; ++i) { // argv[0] is the path to the program, we want from argv[1] onwards
      sources.push_back(argv[i]); 
      }
      for (int i=0;i<argc-2;i++) {  // if have a -m or -tty, MUST have at least 2 args
      std::cout<<sources[i]<<std::endl;
      if (sources[i]==dash_m) {
      std::cout<<"found dash_m"<<std::endl;
      motor_id = atoi(sources[i+1].c_str()); 
      }
      if (sources[i]==dash_tty) {
      std::cout<<"found dash_tty"<<std::endl;
      ttynum = atoi(sources[i+1].c_str()); 
      }
      if (sources[i]==dash_baud) {
      std::cout<<"found dash_baud"<<std::endl;
      baudnum = atoi(sources[i+1].c_str()); 
      }
      }
      ROS_INFO("using motor_id %d at baud code %d via /dev/ttyUSB%d",motor_id,baudnum,ttynum);
    }
  }
  else
  {
    int front_motor_id;
    int rear_motor_id;

    if(!node.getParam("front_motor_id", front_motor_id) || !node.getParam("rear_motor_id", rear_motor_id))
    {
        front_motor_id = 1;
        rear_motor_id = 2;
    }
  }
  
  // A good frequency at which to read motor position and publish that to the controller is 100 Hz
  double dt= 0.01;

  // Open appropriate USB port that has the dynamixel device in it
  ROS_INFO("attempting to open /dev/ttyUSB%d",ttynum);
  bool open_success = open_dxl(ttynum,baudnum);

  // If we fail, warn user and exit
  if (!open_success) {
    ROS_WARN("Could not open /dev/ttyUSB%d; check permissions?",ttynum);
    return 0;
  }

  // A vector of ROS standard message for the motor angle, one for each motor
  std::vector<std_msgs::Int16> motor_ang_msg;
  // A vector of integers for the motor angle, one for each motor
  std::vector<short int> sensed_motor_ang = 0;

  // Restate to user the motor communications parameters for each motor ID
  ROS_INFO("Attempting communication with following:");
  for (int i = 0; i < motor_ids.size(); i++)
  {
    ROS_INFO("-motor_id %d at baudrate code %d",motor_id,baudnum);
    ros::Subscriber subscriber = node.subscribe(in_topic_name,1,dynamixelCB); 
  }

  while(ros::ok()) 
  {
    for (int i = 0; i < motor_ids.size(); i++)
    {
      sensed_motor_ang = read_position(motor_id);
      if (sensed_motor_ang>4096) 
      {
        ROS_WARN("Extremely likely read error from Dynamixel: angular value of %d at cmd %d, ignoring.",sensed_motor_ang-4096,g_goal_angle);
      }
      motor_ang_msg.data = sensed_motor_ang;
      pub_jnt.publish(motor_ang_msg);
    }
   ros::Duration(dt).sleep();
   ros::spinOnce();
   }
  dxl_terminate();
  ROS_INFO("Goodbye!");
  return 0; 
} 
