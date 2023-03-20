
#include <gripper_ntlab_msgs/JointPosition.h>
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
    ros::init(argc, argv, "move_gripper");

    ros::NodeHandle n;
    ros::Publisher fake_controller = n.advertise<gripper_ntlab_msgs::JointPosition>("gripper_ntlab/SetPosition", 10);
    ros::Subscriber joint_sub = n.subscribe("gripper_ntlab/JointState", 10, jointSubCallback);

    ros::Rate loop_rate(0.2);

    // Initialize msg
    float pos[DOF] = {110, -110, 110, -110, 0};

    while (ros::ok()) {
        gripper_ntlab_msgs::JointPosition msg;
        msg.mode = 112;
        //ROS_INFO("ros::ok");
        int counter = 0;
        if (sub_flag) {
            for (int i = 0; i < DOF; i++) {
                //if(last_position[i] == pos[i] + present_position[i])
                last_position[i] = pos[i] + present_position[i];
                msg.position.push_back(last_position[i]);
            }
            fake_controller.publish(msg);
            sub_flag = false;
        }
        loop_rate.sleep();
        ros::spinOnce();
    }

    return 0;
}
