cmake_minimum_required(VERSION 3.0.2)
project(gripper_ntlab_sensor)

find_package(catkin REQUIRED COMPONENTS
  rospy
)

catkin_python_setup()

catkin_package()

catkin_install_python(
	PROGRAMS
	  scripts/fs_read_node
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)