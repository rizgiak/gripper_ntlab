#include <errno.h>
#include <fcntl.h>
#include <gripper_ntlab_msgs/JointPosition.h>
#include <math.h>
#include <std_msgs/Int8.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>

#include <fstream>
#include <iostream>
#include <vector>

#include "moveit_msgs/DisplayRobotState.h"
#include "moveit_msgs/ObjectColor.h"
#include "ros/ros.h"

#define PI (3.141592653589793)

using namespace std;

const int DOF = 5;

sensor_msgs::JointState joint_state;

moveit_msgs::DisplayRobotState displayRobot() {
    moveit_msgs::DisplayRobotState msg;

    msg.state.joint_state = joint_state;
    return msg;
}

double deg2rad(double degree) {
    return degree * (PI / 180);
}

double pulse2rad(double val) {
    return deg2rad((val / 4096) * 360);
}

double pulse2pos(double val) {
    double circle = 2 * PI * 12 / 1000;
    return (val / 4096) * circle;
}

const int direction[DOF] = {-1, 1, 1, -1, 1};
const int xacro_offset[DOF] = {0, 0, -500, 500};
const int mimic_direction[6] = {1, -1, 1, -1, 1, -1};
const string mimic_joint[6] = {
    "l_base_hand_r",
    "r_base_hand_r",
    "l_hand_rod_b",
    "l_rod_a_finger",
    "r_hand_rod_b",
    "r_rod_a_finger"};

void jointSubCallback(const sensor_msgs::JointState::ConstPtr& msg) {
    sensor_msgs::JointState joint_state_msg;

    // hand slide
    for (int i = 0; i < 2; i++) {
        joint_state_msg.name.push_back(msg->name[i]);
        joint_state_msg.position.push_back(pulse2pos(msg->position[i] + xacro_offset[i]) * direction[i]);
        joint_state_msg.effort.push_back(msg->effort[i]);
        joint_state_msg.velocity.push_back(msg->velocity[i]);
    }

    // pinion rotate (mimic)
    for (int i = 0; i < 2; i++) {
        joint_state_msg.name.push_back(mimic_joint[i]);
        joint_state_msg.position.push_back(pulse2rad(msg->position[i] + xacro_offset[i]) * mimic_direction[i]);
        joint_state_msg.effort.push_back(msg->effort[i]);
        joint_state_msg.velocity.push_back(msg->velocity[i]);
    }

    // finger rotate
    for (int i = 2; i < 4; i++) {
        joint_state_msg.name.push_back(msg->name[i]);
        joint_state_msg.position.push_back(pulse2rad(msg->position[i] + xacro_offset[i]) * direction[i]);
        joint_state_msg.effort.push_back(msg->effort[i]);
        joint_state_msg.velocity.push_back(msg->velocity[i]);
    }

    // left finger rotate (mimic)
    for (int i = 2; i < 4; i++) {
        joint_state_msg.name.push_back(mimic_joint[i]);
        joint_state_msg.position.push_back(pulse2rad(msg->position[2] + xacro_offset[2]) * mimic_direction[i]);
        joint_state_msg.effort.push_back(msg->effort[2]);
        joint_state_msg.velocity.push_back(msg->velocity[2]);
    }

    // right finger rotate (mimic)
    for (int i = 4; i < 6; i++) {
        joint_state_msg.name.push_back(mimic_joint[i]);
        joint_state_msg.position.push_back(pulse2rad(msg->position[3] + xacro_offset[3]) * mimic_direction[i]);
        joint_state_msg.effort.push_back(msg->effort[3]);
        joint_state_msg.velocity.push_back(msg->velocity[3]);
    }

    joint_state = joint_state_msg;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "gripper_ntlab_msgs");
    ros::NodeHandle n;

    ros::Publisher robot_pub = n.advertise<moveit_msgs::DisplayRobotState>("gripper_ntlab/display_robot_state", 10);
    ros::Subscriber robot_sub = n.subscribe("gripper_ntlab/JointState", 10, jointSubCallback);

    ros::Rate rate(100);
    while (ros::ok()) {
        robot_pub.publish(displayRobot());
        rate.sleep();
        ros::spinOnce();
    }
    return 0;
}