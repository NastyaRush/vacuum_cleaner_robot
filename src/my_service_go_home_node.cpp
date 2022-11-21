#include "ros/ros.h"
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include "my_service/MoveToBase.h"
#include "point.h"


typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>
    MoveBaseClient;

bool add(my_service::MoveToBase::Request &req,
         my_service::MoveToBase::Response &res) {

  // tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  // wait for the action server to come up
  while (!ac.waitForServer(ros::Duration(5.0))) {
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal goal;

  // we'll send a goal to the robot to move to a particular point
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();

  // set robot orientation
  goal.target_pose.pose.orientation.w = 1.0;

  goal.target_pose.pose.position.x = req.x;
  goal.target_pose.pose.position.y = req.y;

  ROS_INFO("Sending goal");
  ac.sendGoal(goal);
  ac.waitForResult();

  bool result = true;

  if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("The robot has reached home");
  else {
    ROS_INFO("The robot has not reached home");
    result = false;
  }

  res.result = result;
  return true;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "go_to_base");
  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService("go_to_base", add);
  ROS_INFO("Ready to go to base.");
  ros::spin();

  return 0;
}