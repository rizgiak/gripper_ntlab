#include <errno.h>
#include <fcntl.h>
#include <gripper_ntlab_controller/CartesianPosition.h>
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

#define L0 0.016
#define L1 0.055
#define L2 0.081
#define L3 0.088
#define L4 0.013

using namespace std;

const int DOF = 5;
bool set_position_flag = false;
int message_buffer = 0;

sensor_msgs::JointState gripper_joint_state, cobotta_joint_state, joint_states;
gripper_ntlab_controller::JointPosition gripper_joint_position;  // to set each joint in microcontroller (arduino)
gripper_ntlab_controller::CartesianPosition gripper_cartesian_position;

double deg2rad(double degree) {
    return degree * (PI / 180);
}

double rad2deg(double radian) {
    return radian * 180 / PI;
}

double pulse2rad(double pulse) {
    return deg2rad((pulse / 4096) * 360);
}

double pulse2pos(double pulse) {
    double circle = 2 * PI * 12 / 1000;
    return (pulse / 4096) * circle;
}

double pos2pulse(double position) {
    double circle = 2 * PI * 12 / 1000;
    return position * 4096 / circle;
}

double rad2pulse(double radian) {
    return rad2deg(radian) * 4096 / 360;
}

const int direction[DOF] = {1, -1, 1, -1, 1};                             // direction of movement in rviz
const int xacro_offset[DOF] = {0, 0, -550, 550};                          // offset to adjust visualization in rviz
const int mimic_direction[7] = {1, -1, 1, -1, 1, -1, -1};                 // direction for mimic joint
const double kinematic_offset[DOF] = {0, 0, -0.381961, 0.4356498893, 0};  // offset of initial position regarding 0 degree is in home position
const string mimic_joint[7] = {
    "l_base_hand_r",
    "r_base_hand_r",
    "l_hand_rod_b",
    "l_rod_a_finger",
    "r_hand_rod_b",
    "r_rod_a_finger",
    "r_finger_roll"};

// forward Kinematics ------------------------------------------------------------------
double calculateX(double l1, double theta) {
    return L1 + l1 + L2 * cos(theta) + L3;
}

double calculateY(double theta, double dir) {
    return L2 * sin(theta) + L4 * dir - L0 * dir;
}

// Inverse Kinematics ------------------------------------------------------------------
double calculateTheta(double y, double dir) {
    return asin((y + L4 * dir - L0 * dir) / L2);
}

double calculateL1(double theta, double x) {
    return x - L3 - L2 * cos(theta) - L1;
}
// -------------------------------------------------------------------------------------

const double limit_x1[2] = {0.223, 0.27};
const double limit_y1[2] = {-0.075, 0.02};
const double limit_x2[2] = {0.223, 0.27};
const double limit_y2[2] = {-0.02, 0.075};

std::vector<double> motor_last_position = {0, 0, 0, 0, 0};
std::vector<double> calculateJointPulse(double x1, double y1, double x2, double y2, double rad) {
    std::vector<double> motor(5);
    ROS_INFO_STREAM("Position: " << x1 << ", " << y1 << ", " << x2 << ", " << y2 << ", " << rad);

    // Limit position to prevent collision
    if (x1 > limit_x1[0] && x1 < limit_x1[1] &&
        y1 > limit_y1[0] && y1 < limit_y1[1] &&
        x2 > limit_x2[0] && x2 < limit_x2[1] &&
        y2 > limit_y2[0] && y2 < limit_y2[1]) {
        double theta = calculateTheta(y1, -1);
        double l1 = calculateL1(theta, x1);
        if (l1 >= 0) {
            motor[0] = (int)pos2pulse(l1) * -1;         // motor direction
            motor[2] = (int)rad2pulse(theta + 1.228);  // offset homing position to straight down
        } else {
            motor[0] = motor_last_position[0];
            motor[2] = motor_last_position[2];
            ROS_INFO_STREAM("Left. Solution not found. Unreachable position.");
        }

        theta = calculateTheta(y2, 1);
        l1 = calculateL1(theta, x2);
        if (l1 >= 0) {
            motor[1] = (int)pos2pulse(l1) * 1;       // motor direction
            motor[3] = (int)rad2pulse(theta - 1.28);  // offset homing position to straight down
        } else {
            motor[1] = motor_last_position[1];
            motor[3] = motor_last_position[3];
            ROS_INFO_STREAM("Right. Solution not found. Unreachable position.");
        }

        motor[4] = (int)rad2pulse(rad);
        motor_last_position = motor;
    } else {
        motor = motor_last_position;
        ROS_INFO_STREAM("Solution not found. Cartesian out of the limit.");
    }

    return motor;
}

void gripperSetCartesianPositionSubCallback(const gripper_ntlab_controller::CartesianPosition::ConstPtr& msg) {
    gripper_ntlab_controller::JointPosition joint_position;
    joint_position.mode = 101;  // mode set position
    vector<double> position = calculateJointPulse(msg->x1, msg->y1, msg->x2, msg->y2, msg->rad);
    ROS_INFO_STREAM("Pulse: " << position[0] << ", " << position[1] << ", " << position[2] << ", " << position[3] << ", " << position[4]);
    joint_position.position = position;
    gripper_joint_position = joint_position;
    message_buffer = msg->buffer;
    set_position_flag = msg->torque;
}

void gripperSetJointPositionSubCallback(const gripper_ntlab_controller::JointPosition::ConstPtr& msg) {
    gripper_ntlab_controller::JointPosition joint_position;
    joint_position.mode = msg->mode;
    for (int i = 0; i < 2; i++) {
        joint_position.position.push_back((int)pos2pulse(msg->position[i]));
    }
    for (int i = 2; i < 5; i++) {
        joint_position.position.push_back((int)rad2pulse(msg->position[i]));
    }
    gripper_joint_position = joint_position;
    set_position_flag = true;
}

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

    // assign left roll
    joint_state_msg.name.push_back(msg->name[4]);
    joint_state_msg.position.push_back(pulse2rad(msg->position[4]));
    joint_state_msg.effort.push_back(msg->effort[4]);
    joint_state_msg.velocity.push_back(msg->velocity[4]);

    // mimic right roll
    joint_state_msg.name.push_back(mimic_joint[6]);
    joint_state_msg.position.push_back(pulse2rad(msg->position[4]) * mimic_direction[6]);
    joint_state_msg.effort.push_back(msg->effort[4]);
    joint_state_msg.velocity.push_back(msg->velocity[4]);

    gripper_joint_state = joint_state_msg;

    // Calculate forward kinematics --------------------------------------------------------------
    double x1, y1, theta1, x2, y2, theta2, rad;
    // left hand
    theta1 = (pulse2rad(msg->position[2] + xacro_offset[2]) + kinematic_offset[2]);
    y1 = calculateY(theta1, 1);
    x1 = calculateX(pulse2pos(msg->position[0] + xacro_offset[0]) * -direction[0], theta1);
    theta1 = rad2deg(theta1);

    // right hand
    theta2 = (pulse2rad(msg->position[3] + xacro_offset[3]) + kinematic_offset[3]);
    y2 = calculateY(theta2, -1);
    x2 = calculateX(pulse2pos(msg->position[1] + xacro_offset[1]) * -direction[1], theta2);
    theta2 = rad2deg(theta2);

    rad = pulse2rad(msg->position[4]);
    // -------------------------------------------------------------------------------------------

    // set gripper current position in cartesian space
    gripper_cartesian_position.x1 = x1;
    gripper_cartesian_position.y1 = y1;
    gripper_cartesian_position.x2 = x2;
    gripper_cartesian_position.y2 = y2;
    gripper_cartesian_position.rad = rad;
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
    ros::Publisher gripper_pub = n.advertise<gripper_ntlab_controller::JointPosition>("gripper_ntlab/set_position", 10);
    ros::Publisher gripper_cartesian_pub = n.advertise<gripper_ntlab_controller::CartesianPosition>("gripper_ntlab/cartesian_position", 10);

    ros::Subscriber hand_set_cartesian_sub = n.subscribe("cobotta/hand_set_cartesian", 10, gripperSetCartesianPositionSubCallback);
    ros::Subscriber hand_set_joint_sub = n.subscribe("cobotta/hand_set_joint", 10, gripperSetJointPositionSubCallback);
    ros::Subscriber gripper_sub = n.subscribe("gripper_ntlab/joint_states", 10, gripperJointSubCallback);
    ros::Subscriber cobotta_sub = n.subscribe("cobotta/joint_states", 10, cobottaJointSubCallback);

    ros::Rate rate(100);
    while (ros::ok()) {
        gripper_cartesian_pub.publish(gripper_cartesian_position);

        if (set_position_flag) {
            for (int i = 0; i < message_buffer + 1; i++) {
                gripper_pub.publish(gripper_joint_position);
            }
            set_position_flag = false;
        }
        combineJointState();
        joint_states.header.stamp = ros::Time::now();
        joint_pub.publish(joint_states);
        rate.sleep();
        ros::spinOnce();
    }
    return 0;
}