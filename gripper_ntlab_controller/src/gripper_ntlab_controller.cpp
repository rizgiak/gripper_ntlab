#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <termios.h>
#include <string.h>
#include <fcntl.h>
#include <errno.h>

#include <iostream>
#include <fstream>
#include <math.h>

#include "ros/ros.h"
#include <gripper_ntlab_controller/JointPosition.h>
#include <std_msgs/Int8.h>

using namespace std;

int main(int argc, char **argv){

	ros::init(argc, argv, "gripper_ntlab_controller");

  	ros::NodeHandle n;
	ros::Rate rate(100);
	while(ros::ok()){
		rate.sleep();
		ros::spinOnce();
	}
	return 0;
}