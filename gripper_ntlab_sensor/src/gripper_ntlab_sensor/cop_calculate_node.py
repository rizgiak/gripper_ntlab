# -*- coding: utf-8 -*-
import rospy
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches

from std_msgs.msg import Float32, Bool
from geometry_msgs.msg import Point32


class COPCalculateNode:
    def fs1_callback(self, data):
        self.fs1_data = data.data

    def fs2_callback(self, data):
        self.fs2_data = data.data

    def fs3_callback(self, data):
        self.fs3_data = data.data

    # TODO: fix cop calculation
    def cop_calculation(self):
        data = Point32()

        dx = 26
        dy = 31

        x_den = (self.fs3_data + self.fs2_data) * 2
        y_den = (self.fs1_data + self.fs2_data + self.fs3_data) * 2

        if x_den < 0.001 or y_den < 0.001:
            x_cop = y_cop = 0
        else:
            x_cop = (self.fs3_data - self.fs2_data) * dx / x_den
            y_cop = (self.fs1_data - self.fs2_data - self.fs3_data) * dy / y_den

        data.x = x_cop
        data.y = y_cop
        data.z = 0
        return data, True

    def draw_plot(self):
        print("draw plot")

    def reset_sensor(self):
        rospy.sleep(2)
        self.pub_fs1_reset.publish(data=True)
        self.pub_fs2_reset.publish(data=True)
        self.pub_fs3_reset.publish(data=True)
        rospy.sleep(0.5)
        self.pub_fs1_reset.publish(data=False)
        self.pub_fs2_reset.publish(data=False)
        self.pub_fs3_reset.publish(data=False)

    def __init__(self, node_name):
        print(node_name)
        rospy.init_node(node_name)

        # Param load

        # Set publisher
        self.pub = rospy.Publisher("/cop", Point32, queue_size=10)
        self.pub_fs1_reset = rospy.Publisher("/fs1_reset", Bool, queue_size=10)
        self.pub_fs2_reset = rospy.Publisher("/fs2_reset", Bool, queue_size=10)
        self.pub_fs3_reset = rospy.Publisher("/fs3_reset", Bool, queue_size=10)

        # Set subscriber
        self.sub_fs1 = rospy.Subscriber("/fs1", Float32, self.fs1_callback)
        self.sub_fs2 = rospy.Subscriber("/fs2", Float32, self.fs2_callback)
        self.sub_fs3 = rospy.Subscriber("/fs3", Float32, self.fs3_callback)

        # Set init

    def run(self):
        """
        Run the COP Calculate Node
        """

        buffer_data = 5

        x = [0 for i in range(buffer_data)]  # x軸のサイズ
        y = [0 for k in range(buffer_data)]  # y軸のサイズ

        # create loadcell visual
        points = [[13, -15.5], [0, 15.5], [-13, -15.5], [13, -15.5]]
        patch = patches.Polygon(xy=points, closed=True, alpha=0.35)
        sensor_l1 = plt.Circle((0.0, 15.5), 3.0, color="#000000", alpha=0.2)
        sensor_l2 = plt.Circle((-13.0, -15.5), 3.0, color="#000000", alpha=0.2)
        sensor_l3 = plt.Circle((13.0, -15.5), 3.0, color="#000000", alpha=0.2)

        plt.ion()
        plt.figure()

        plt.title("Center of Pressure")
        plt.xlabel("x")
        plt.ylabel("y")
        plt.ylim(-19.5, 19.5)  # 固定：y軸の最小値，最大値
        plt.xlim(-17, 17)
        plt.grid(True)
        (li,) = plt.plot(x, y)
        plt.show()
        plt.gca().add_patch(sensor_l1)
        plt.gca().add_patch(sensor_l2)
        plt.gca().add_patch(sensor_l3)
        plt.gca().add_patch(patch)

        self.reset_sensor()

        while not rospy.is_shutdown():
            ret, success = self.cop_calculation()
            if success:
                x.append(ret.x)
                x.pop(0)
                y.append(ret.y)
                y.pop(0)
                li.set_xdata(x)
                li.set_ydata(y)
                plt.draw()
                plt.pause(0.1)
                self.pub.publish(ret)
