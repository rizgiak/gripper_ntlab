
#include <gripper_ntlab_controller/JointPosition.h>
#include <std_msgs/Float64.h>

#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "std_msgs/String.h"

#define PI (3.141592653589793)
#define DOF 5

float present_position[DOF] = {0};
float last_position[DOF] = {0};
std::stringstream ss;

bool sub_flag = false;

void jointSubCallback(const sensor_msgs::JointState::ConstPtr& msg) {
    ss.str(std::string());
    ss << "[JointPosition]: ";
    for (int i = 0; i < DOF; i++) {
        present_position[i] = msg->position[i];
        ss << present_position[i] << ", ";
    }
    sub_flag = true;
    ROS_INFO_STREAM(ss.str());
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "move_gripper_ik_test");

    ros::NodeHandle n;
    ros::Publisher fake_controller = n.advertise<gripper_ntlab_controller::JointPosition>("cobotta/hand_set_cartesian", 10);
    ros::Rate loop_rate(30);

    std::vector<double> position;
    position.push_back(0.2240);
    position.push_back(-0.0767);
    position.push_back(0.2205);
    position.push_back(-0.0274);
    position.push_back(0);
    double dir = 1;

    while (ros::ok()) {
        gripper_ntlab_controller::JointPosition msg;
        msg.mode = 101;
        msg.position = position;
        fake_controller.publish(msg);

        position[1] = position[1] + 0.0001 * dir;
        position[3] = position[3] + 0.0001 * dir;

        if (position[1] >= 0.0319 || position[1] <= -0.0787 || position[3] >= 0.0750 || position[3] <= -0.0307) {
            dir *= -1;
        }

        loop_rate.sleep();
        ros::spinOnce();
    }

    return 0;
}
