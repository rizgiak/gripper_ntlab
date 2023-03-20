
#include <gripper_ntlab_msgs/CartesianPosition.h>
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
    ros::init(argc, argv, "move_gripper_ik_test");

    ros::NodeHandle n;
    ros::Publisher fake_controller = n.advertise<gripper_ntlab_msgs::CartesianPosition>("cobotta/hand_set_cartesian", 10);
    ros::Rate loop_rate(30);

    gripper_ntlab_msgs::CartesianPosition msg;
    msg.torque = true;
    msg.x1 = 0.2240;
    msg.y1 = -0.0537;
    msg.x2 = 0.2205;
    msg.y2 = -0.0274;
    msg.rad = 0;

    double dir = 1;

    while (ros::ok()) {
        fake_controller.publish(msg);

        msg.y1 = msg.y1 + 0.0002 * dir;
        msg.y2 = msg.y2 + 0.0002 * dir;

        if (msg.y1 >= 0.0319 || msg.y1 <= -0.0787 || msg.y2 >= 0.0750 || msg.y2 <= -0.0307) {
            dir *= -1;
        }

        loop_rate.sleep();
        ros::spinOnce();
    }

    return 0;
}
