#include <errno.h>
#include <fcntl.h>
#include <gripper_ntlab_controller/JointPosition.h>
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

sensor_msgs::JointState gripper_joint_state, cobotta_joint_state, joint_states;

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
const int mimic_direction[7] = {1, -1, 1, -1, 1, -1, -1};
const string mimic_joint[7] = {
    "l_base_hand_r",
    "r_base_hand_r",
    "l_hand_rod_b",
    "l_rod_a_finger",
    "r_hand_rod_b",
    "r_rod_a_finger",
    "r_finger_roll"};

void gripperJointSubCallback(const sensor_msgs::JointState::ConstPtr& msg) {
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

    //assign left roll
    joint_state_msg.name.push_back(msg->name[4]);
    joint_state_msg.position.push_back(pulse2rad(msg->position[4]));
    joint_state_msg.effort.push_back(msg->effort[4]);
    joint_state_msg.velocity.push_back(msg->velocity[4]);

    //mimic right roll
    joint_state_msg.name.push_back(mimic_joint[6]);
    joint_state_msg.position.push_back(pulse2rad(msg->position[4]) * mimic_direction[6]);
    joint_state_msg.effort.push_back(msg->effort[4]);
    joint_state_msg.velocity.push_back(msg->velocity[4]);

    gripper_joint_state = joint_state_msg;
}

void cobottaJointSubCallback(const sensor_msgs::JointState::ConstPtr& msg) {
    sensor_msgs::JointState joint_state_msg;

    for (int i = 0; i < 6; i++) {
        joint_state_msg.name.push_back(msg->name[i]);
        joint_state_msg.position.push_back(msg->position[i]);
        joint_state_msg.effort.push_back(msg->effort[i]);
        joint_state_msg.velocity.push_back(msg->velocity[i]);
    }

    cobotta_joint_state = joint_state_msg;
}

void combineJointState() {
    sensor_msgs::JointState joint_state_msg;

    for (int i = 0; i < cobotta_joint_state.name.size(); i++) {
        joint_state_msg.name.push_back(cobotta_joint_state.name[i]);
        joint_state_msg.position.push_back(cobotta_joint_state.position[i]);
        joint_state_msg.effort.push_back(cobotta_joint_state.effort[i]);
        joint_state_msg.velocity.push_back(cobotta_joint_state.velocity[i]);
    }

    for (int i = 0; i < gripper_joint_state.name.size(); i++) {
        joint_state_msg.name.push_back(gripper_joint_state.name[i]);
        joint_state_msg.position.push_back(gripper_joint_state.position[i]);
        joint_state_msg.effort.push_back(gripper_joint_state.effort[i]);
        joint_state_msg.velocity.push_back(gripper_joint_state.velocity[i]);
    }
    joint_states = joint_state_msg;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "gripper_ntlab_controller");
    ros::NodeHandle n;

    ros::Publisher joint_pub = n.advertise<sensor_msgs::JointState>("cobotta/all_joint_states", 10);
    ros::Subscriber gripper_sub = n.subscribe("gripper_ntlab/joint_states", 10, gripperJointSubCallback);
    ros::Subscriber cobotta_sub = n.subscribe("cobotta/joint_states", 10, cobottaJointSubCallback);
    ros::Rate rate(100);
    while (ros::ok()) {
        combineJointState();
        joint_states.header.stamp = ros::Time::now();
        joint_pub.publish(joint_states);
        rate.sleep();
        ros::spinOnce();
    }
    return 0;
}