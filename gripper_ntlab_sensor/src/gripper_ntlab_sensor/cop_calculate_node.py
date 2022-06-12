# -*- coding: utf-8 -*-
import rospy
import numpy as np
import matplotlib.pyplot as plt

from std_msgs.msg import Float32
from geometry_msgs.msg import Point32


class COPCalculateNode:
    def fs1_callback(self, data):
        self.fs1_data = data

    def fs2_callback(self, data):
        self.fs2_data = data

    def fs3_callback(self, data):
        self.fs3_data = data

    # TODO: fix cop calculation
    def cop_calculation(self):
        data = Point32()
        data.x = self.fs1_data
        data.y = self.fs2_data
        data.z = self.fs3_data
        return data, True

    def draw_plot(self):
        print("draw plot")

    def __init__(self, node_name):
        print(node_name)
        rospy.init_node(node_name)

        # Param load

        # Set publisher
        self.pub = rospy.Publisher("/cop", Point32, queue_size=10)

        # Set subscriber
        self.sub_fs1 = rospy.Subscriber("/fs1", Float32, self.fs1_callback)
        self.sub_fs2 = rospy.Subscriber("/fs2", Float32, self.fs2_callback)
        self.sub_fs3 = rospy.Subscriber("/fs3", Float32, self.fs3_callback)

        # Set init

    def run(self):
        """
        Run the COP Calculate Node
        """

        x = [0 for i in range(5)]  # x軸のサイズ
        y = [0 for k in range(5)]  # y軸のサイズ

        plt.ion()
        plt.figure()

        plt.title("Center of Pressure")
        plt.xlabel("x")
        plt.ylabel("y")
        plt.ylim(-10, 10)  # 固定：y軸の最小値，最大値
        plt.xlim(-10, 10)
        plt.grid(True)
        (li,) = plt.plot(x, y)

        while not rospy.is_shutdown():
            ret, success = self.cop_calculation()
            if success:
                x.append(ret.x.data)
                x.pop(0)
                y.append(ret.y.data)
                y.pop(0)
                li.set_xdata(x)
                li.set_ydata(y)

                plt.draw()
                plt.pause(0.1)
                self.pub.publish(ret)
