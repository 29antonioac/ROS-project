/*
 * RandomWalk.cpp
 *
 *  Created on: Oct 25, 2013
 *      Author: roiyeho
 */

#include "random_walk.h"
#include "geometry_msgs/Twist.h"

RandomWalk::RandomWalk()
{
	possibleCrash = false;

	// Advertise a new publisher for the simulated robot's velocity command topic
	commandPub = node.advertise<geometry_msgs::Twist>("cmd_vel", 10);

	// Subscribe to the simulated robot's laser scan topic
	laserSub = node.subscribe("base_scan", 1, &RandomWalk::scanCallback, this);
}

// Send a velocity command
void RandomWalk::moveForward() {
	geometry_msgs::Twist msg; // The default constructor will set all commands to 0
	msg.linear.x = FORWARD_SPEED_MPS;

  if(possibleCrash)
    msg.angular.z = ANGLE_SPEED_RPS;

  commandPub.publish(msg);
};

// Process the incoming laser scan message
void RandomWalk::scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
	// Find the closest range between the defined minimum and maximum angles
	int minIndex = ceil((MIN_SCAN_ANGLE_RAD - scan->angle_min) / scan->angle_increment);
	int maxIndex = floor((MAX_SCAN_ANGLE_RAD - scan->angle_min) / scan->angle_increment);

	float closestRange = scan->ranges[minIndex];
	for (int currIndex = minIndex + 1; currIndex <= maxIndex; currIndex++) {
		if (scan->ranges[currIndex] < closestRange) {
			closestRange = scan->ranges[currIndex];
		}
	}

	ROS_INFO_STREAM("Closest range: " << closestRange);

	if (closestRange < MIN_PROXIMITY_RANGE_M) {
		ROS_INFO("Turn!");
    possibleCrash = true;
	}
  else{
    possibleCrash = false;
  }
}

void RandomWalk::startMoving()
{
	ros::Rate rate(10);
	ROS_INFO("Start moving");

	// Keep spinning loop until user presses Ctrl+C or the robot got too close to an obstacle
	while (ros::ok()) {
		moveForward();
		ros::spinOnce(); // Need to call this function often to allow ROS to process incoming messages
		rate.sleep();
	}
}
