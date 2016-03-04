/*
 * run_stopper.cpp
 *
 *  Created on: Oct 25, 2013
 *      Author: roiyeho
 */

#include "random_walk.h"

int main(int argc, char **argv) {
	// Initiate new ROS node named "random_walk"
	ros::init(argc, argv, "random_walk");

	// Create new stopper object
	RandomWalk random_walk;

	// Start the movement
	random_walk.startMoving();

	return 0;
};
