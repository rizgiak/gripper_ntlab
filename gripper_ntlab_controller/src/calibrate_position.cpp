#include <gripper_ntlab_msgs/JointPosition.h>
#include <std_msgs/Float64.h>

#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "std_msgs/String.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "calibrate_position");

    ros::NodeHandle n;
    ros::Publisher fake_controller = n.advertise<gripper_ntlab_msgs::JointPosition>("gripper_ntlab/SetPosition", 10);
    ros::Rate loop_rate(0.2);

    while (ros::ok()) {
        gripper_ntlab_msgs::JointPosition msg;
        msg.mode = 124;
        fake_controller.publish(msg);
        loop_rate.sleep();
        ros::spinOnce();
    }

    return 0;
}
