#pragma once

#include "actionlib_msgs/GoalStatusArray.h"
#include "point.h"
#include "room.h"
#include <actionlib/client/simple_action_client.h>
#include <cstdlib>
#include <move_base_msgs/MoveBaseAction.h>
#include <ros/ros.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>
    MoveBaseClient;

// class for algorithm
class Algorithm {

  // accepted robot size, the smaller is this number
  // the greater is the accuracy of cleaning
  float robotSize = 0.32;

  // the greater is the number
  // the greater is the accuracy of cleaning
  unsigned xParts = 5;

  // robot has more time to reach the first goal
  unsigned firstGoalWaitTime = 40;

  unsigned regularWaitTime = 15;
  bool firstGoal;
  MoveBaseClient *ac;
  std::set<int> *nextGoalStatuses;
  std::vector<Point *> goalPoints;
  unsigned curGoalPoint;
  Room *goalRoom;
  void initGoalPoints();

public:
  Algorithm(Room *, std::set<int> *, MoveBaseClient *);
  void algorithmCycle();
  void chatterCallback(const actionlib_msgs::GoalStatusArray::ConstPtr &msg);
};