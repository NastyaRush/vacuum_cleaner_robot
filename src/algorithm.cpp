#include "algorithm.h"

Algorithm::Algorithm(Room *room, std::set<int> *nextGoalStatuses,
                     MoveBaseClient *ac) {
  this->goalRoom = room;
  this->nextGoalStatuses = nextGoalStatuses;
  this->ac = ac;
  this->firstGoal = true;
  initGoalPoints();
};

// init points which robot will visit
void Algorithm::initGoalPoints() {
  float curX = this->goalRoom->getX();
  float curY = this->goalRoom->getY();
  this->goalPoints.push_back(new Point(curX, curY));

  bool yMove;
  if (this->goalRoom->getY() < this->goalRoom->getWidth()) {
    yMove = curY < this->goalRoom->getWidth();
  } else {
    yMove = curY > this->goalRoom->getWidth();
  }
  unsigned xStep = 0;
  float xStepValue =
      abs(abs(this->goalRoom->getX()) - abs(this->goalRoom->getHeight())) /
      this->xParts;

  // if x coordinate increases
  bool xRising;
  if (this->goalRoom->getX() < this->goalRoom->getHeight()) {
    xRising = true;
  } else {
    xRising = false;
  }

  // while start y coordinate will not reach finish y coordinate do
  while (yMove) {
    // modify x coordinate depending on direction
    if (xRising) {
      curX += xStepValue;
    } else {
      curX -= xStepValue;
    }
    if (xStep == this->xParts - 1) {
      this->goalPoints.push_back(new Point(curX, curY));

      // modify y coordinate and condition of cycle depending on start and finish value
      if (this->goalRoom->getY() < this->goalRoom->getWidth()) {
        curY += this->robotSize / 2;
        yMove = curY < this->goalRoom->getWidth();
      } else {
        curY -= this->robotSize / 2;
        yMove = curY > this->goalRoom->getWidth();
      }
      xStep = 0;
      xRising = !xRising;
    } else {
      xStep += 1;
    }

    this->goalPoints.push_back(new Point(curX, curY));
  }

  Point *finishPoint =
      new Point(this->goalRoom->getHeight(), this->goalRoom->getWidth());
  this->goalPoints.push_back(finishPoint);
  this->curGoalPoint = 0;
    this->goalPoints.push_back(new Point(curX, curY));
  
  for (unsigned i = 0; i < this->goalPoints.size(); i++) {
    if (i > 4) {
      this->goalPoints.erase(std::next(this->goalPoints.begin(), i));
    }
  }
}

void Algorithm::algorithmCycle() {
  move_base_msgs::MoveBaseGoal goal;
  // we'll send a goal to the robot to move to a particular Room
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();

  goal.target_pose.pose.orientation.w = 1.0;
  goal.target_pose.pose.position.x =
      this->goalPoints[this->curGoalPoint]->getX();
  goal.target_pose.pose.position.y =
      this->goalPoints[this->curGoalPoint]->getY();
  this->ac->sendGoal(goal);
  
  if (this->firstGoal)  {
    this->ac->waitForResult(ros::Duration(this->firstGoalWaitTime));
  } else {
    // if the goal wasn't reached during this time then skip this goal
    this->ac->waitForResult(ros::Duration(this->regularWaitTime));
  }
  return;
}

void Algorithm::chatterCallback(
    const actionlib_msgs::GoalStatusArray::ConstPtr &msg) {
  // separate serving of the first goal
  if (this->firstGoal) {
    this->algorithmCycle();
    this->curGoalPoint += 1;
    this->firstGoal = !this->firstGoal;
  }

  // cycle repeats till there are points for visiting them
  // if there was detected status to move to next goal then move to next goal
  while (this->curGoalPoint + 1 < this->goalPoints.size()) {
    if (int(msg->status_list.size()) > 0) {
      int status = msg->status_list[0].status;
      if (this->nextGoalStatuses->find(status) !=
          this->nextGoalStatuses->end()) {
        this->algorithmCycle();
        this->curGoalPoint += 1;
      }
    }
  }
}
