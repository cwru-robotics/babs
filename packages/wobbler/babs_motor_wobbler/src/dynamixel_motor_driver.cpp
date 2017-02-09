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
#define DEFAULT_MOTOR_COUNT 2 // We expect the sentry to have exactly two motors, one each for front and rear wobbler

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

  int motor_tty;
  int motor_baud;

  // Only change to true if you want to remove reliance on parameter server and use cmd line arguments instead
  bool use_cmd_line_args = false;
   
  // Traditional command line way? Or new parameter based way?
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

    char motor_id_param[50];
    std::vector<int> motor_ids;
    int motor_id;


    for (int i = 0; i < DEFAULT_MOTOR_COUNT; i++)
    {
      // Use a hacky trick I saw to name mangle the motor ID parameter names
      sprintf(motor_id_param,"motor%d_id",i);
      ROS_INFO("node name: %s",node_name);

      if(!node.getParam(motor_id_param, motor_id))
      {
        ROS_WARN("Warning, could not find appropriate motor ID on motor %d", i);
      }
      else
      {
        motor_ids.push_back(motor_id);
      }
    }

    if(!node.getParam("motor_tty", motor_tty))
    {
      ROS_WARN("Warning, could not find dynamixel tty device number to connect to. Could cause motors to not function.");
      motor_tty = 999;
    }

    if(!node.getParam("motor_baud", motor_baud))
    {
      ROS_WARN("Warning, could not find appropriate dynamixel motor baud rate to connect at. Could cause motors to not function.");
      motor_baud = 34;
    }

  }
  
  // A good frequency at which to read motor position and publish that to the controller is 100 Hz
  double dt = 0.01;

  // Open appropriate USB port that has the dynamixel device in it
  ROS_INFO("attempting to open /dev/ttyUSB%d",motor_tty);
  bool open_success = open_dxl(motor_tty,motor_baud);

  // If we fail to open the device file, warn user and exit
  if (!open_success) {
    ROS_WARN("Could not open /dev/ttyUSB%d; check permissions?",ttynum);
    return 0;
  }

  // A vector of ROS standard message for the motor angle, one for each motor
  std::vector<std_msgs::Int16> motor_angle_commands;
  // A vector of integers for the motor angle, one for each motor
  std::vector<short int> sensed_motor_angles;

  for (int i = 0; i < motor_ids.size(); i++)
  {
    sensed_motor_angles.push_back(0);
  }

  // Restate to user the motor communications parameters for each motor ID
  ROS_INFO("Attempting communication with following motors:");
  for (int i = 0; i < motor_ids.size(); i++)
  {
    ROS_INFO("-motor_id %d at baudrate code %d",motor_id, motor_baud);
    ros::Subscriber subscriber = node.subscribe(in_topic_name, 1, dynamixelCB); 
  }

  // Main program loop to read motor positions from device and push them to ROS topics for each motor
  while(ros::ok()) 
  {
    // For each of the motors that we have (should be two for this)
    for (int i = 0; i < motor_ids.size(); i++)
    {
      // Use Dynamixel library to read the particular motors current position and store it in proper location of the vector
      sensed_motor_angles[i] = read_position(motor_ids[i]);

      // Check if the received value is bad, holdover from older times...its marginally important to make a note of for debugging
      if (sensed_motor_angles[i]>4096) 
      {
        ROS_WARN("Extremely likely read error from Dynamixel: angular value of %d at cmd %d, ignoring.",sensed_motor_angles[i]-4096,g_goal_angle);
      }
      // Prepare to publish the received data value
      motor_angle_commands[i].data = sensed_motor_angles[i];
      pub_jnt.publish(motor_angle_commands[i]);
    }
    // Take a break, you've had a long day...
   ros::Duration(dt).sleep();
   ros::spinOnce();
   }
   // Should never happen until ros cuts out entirely
  dxl_terminate();
  ROS_INFO("Goodbye!");
  return 0; 
} 
