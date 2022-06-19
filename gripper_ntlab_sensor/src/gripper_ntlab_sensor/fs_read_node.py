# -*- coding: utf-8 -*-
import rospy
import serial

from std_msgs.msg import Float32, Bool


class FSReadNode:
    def init_fs_sensor(self, _serial):
        _packet = bytearray()
        _packet.append(0x43)
        _packet.append(0x58)
        _packet.append(0xD)
        _serial.write(_packet)
    
    def fs_reset_callback(self, data):
         self.reset_flag = data.data

    def read_sensor(self, _serial, _filtered):
        ret = 0
        success = False
        if _serial.read() == b"G": # Header validation
            if _serial.read() == b"S": # Header validation
                _serial.read()  # Read ","
                _status = _serial.read(4)  # Read status
                _serial.read()  # Read ","
                raw_notation = _serial.read()  # Read notation
                raw_data = _serial.read(6)  # Read data
                _serial.read()  # Read ","
                filtered_notation = _serial.read()  # Read notation
                filtered_data = _serial.read(6)  # Read data
                success = True
                if(_filtered):
                    ret = filtered_notation + filtered_data
                    if float(ret) < float(self.hpf_threshold): # apply high pass filter
                        ret = 0
                else:
                    ret = raw_notation + raw_data
                
                if self.reset_flag: # set offset data to create initial offset
                    self.offset = float(ret)

        return float(ret), success

    def __init__(self, node_name):
        print(node_name)
        rospy.init_node(node_name)

        # Param load
        port = rospy.get_param("~port", "/dev/ttyUSB0")
        baud = rospy.get_param("~baud", 460800)
        topic_name = rospy.get_param("~topic_name", "/fs1")
        topic_reset_name = rospy.get_param("~topic_reset_name", "/fs1_reset")
        self.filtered = rospy.get_param("~filtered", True)
        self.hpf_threshold = rospy.get_param("~hpf_threshold", 0.001)

        # Set publisher 
        self.pub = rospy.Publisher(topic_name, Float32, queue_size=10)

        # Set subscriber
        self.sub = rospy.Subscriber(topic_reset_name, Bool, self.fs_reset_callback)

        self.offset = 0
        self.reset_flag = False
        try:
            self.ser = serial.Serial(port, baud)
            self.init_fs_sensor(self.ser)
        except:
            print("Failed to connect")
            exit()

    def run(self):
        """
        Run the FSRead Node
        """

        while not rospy.is_shutdown():
            ret, success = self.read_sensor(self.ser, self.filtered)
            ret = ret - self.offset
            if success:
                self.pub.publish(Float32(data=ret))
