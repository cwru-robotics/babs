#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv){
  ros::init(argc, argv, "simple_navigation_goals");

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  //wait for the action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal goal;

  //we'll send a goal to the robot to move 1 meter forward
  goal.target_pose.header.frame_id = "base_link";

  goal.target_pose.header.stamp = ros::Time::now();
  goal.target_pose.pose.position.x = 30.0;
  goal.target_pose.pose.position.y = 0.0;
  goal.target_pose.pose.orientation.w = 1.0;
  ROS_INFO("Sending goal");
  ac.sendGoal(goal);
  ac.waitForResult();
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("GOAL 1 SUCCESSS!");
  else
    ROS_INFO("GOAL 1 FAIL!");


  // goal.target_pose.header.stamp = ros::Time::now();
  // goal.target_pose.pose.position.x = 3.0;
  // goal.target_pose.pose.position.y = 1.0;
  // goal.target_pose.pose.orientation.w = 1.0;
  // ROS_INFO("2Sending goal");
  // ac.sendGoal(goal);
  // ac.waitForResult();
  // if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
  //   ROS_INFO("GOAL 2 SUCCESSS!");
  // else
  //   ROS_INFO("GOAL 2 FAIL!");

  return 0;
}