# -*- coding: utf-8 -*-
import rospy

class FSReadNode():
    def __init__(self, node_name):
        print (node_name)
        rospy.init_node(node_name)