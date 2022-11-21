#include "ros/ros.h"
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include "actionlib_msgs/GoalStatusArray.h"
#include "my_service/CleanRoom.h"
#include "algorithm.h"
#include "point.h"
#include "process_data.h"
#include "room.h"

#include <iostream>
#include <sstream>
#include <tuple>
#include <unistd.h>

#define PREEMPTED 2
#define SUCCEEDED 3
#define ABORTED 4
#define REJECTED 5

std::set<int> nextGoalStatuses;

// settings for reading file
std::string roomsFilename = "rooms.csv";
char valueDelimiter = ',';
char lineDelimiter = '\n';

std::vector<Room *> rooms;

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>
    MoveBaseClient;

bool clean_room(my_service::CleanRoom::Request &req,
         my_service::CleanRoom::Response &res) {

  std::string result;

  // tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  // wait for the action server to come up
  while (!ac.waitForServer(ros::Duration(5.0))) {
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  // create flag for case with uncorrect point name
  bool match;
  unsigned room_index;
  /////////////////////////////////////////////////

    int roomNameLength = req.roomName.length();
 
    // declaring character array
    char char_array[roomNameLength + 1];
 
    // copying the contents of the
    // string to char array
    strcpy(char_array, req.roomName.c_str());

  std::tie(match, room_index) = findRoom(&rooms, char_array);
  

  // if point name is uncorrect than show list of correct room names
  if (match == false) {
    showCorrectRoomNames(&rooms);
    result = "Uncorrect room name was provided.";
  }
  // if point name is correct than send goal and wait for result
  else {
    ros::NodeHandle n;
    Algorithm algorithm(rooms[room_index], &nextGoalStatuses, &ac);
    ros::Subscriber sub = n.subscribe("move_base/status", 1000,
                                      &Algorithm::chatterCallback, &algorithm);
    result = "Cleaning " + req.roomName + " was finished.";
    ros::spinOnce();
  }

  res.result = result;
  return true;
}

int main(int argc, char **argv) {
  // file with rooms must be in working directory
  std::stringstream roomsStream = readFile(roomsFilename);
  if (!roomsStream.str().empty()) {
    nextGoalStatuses.insert({PREEMPTED, SUCCEEDED, ABORTED, REJECTED});

    rooms = getRooms(&roomsStream, lineDelimiter, valueDelimiter);

    ros::init(argc, argv, "clean_room");
    ros::NodeHandle n; 

    ros::ServiceServer service = n.advertiseService("clean_room", clean_room);
    ROS_INFO("Ready to clean rooms.");
    ros::spin();    
  }
  return 0;
}