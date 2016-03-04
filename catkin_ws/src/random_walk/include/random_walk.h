#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"

class RandomWalk
{
public:
  // Tunable parameters
  const static double FORWARD_SPEED_MPS = 0.2;
  const static double ANGLE_SPEED_RPS = 2;
  double       ANGLE_DIRECTION;
  const static double MIN_SCAN_ANGLE_RAD = -30.0/180*M_PI;
  const static double MAX_SCAN_ANGLE_RAD = +30.0/180*M_PI;
  double MIN_PROXIMITY_RANGE_M; // Should be smaller than sensor_msgs::LaserScan::range_max

  RandomWalk();
  void startMoving();

private:
  ros::NodeHandle node;
  ros::Publisher commandPub; // Publisher to the simulated robot's velocity command topic
  ros::Subscriber laserSub; // Subscriber to the simulated robot's laser scan topic
  bool possibleCrash; // Indicates whether the robot should continue moving

  void moveForward();
  void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan);
};
