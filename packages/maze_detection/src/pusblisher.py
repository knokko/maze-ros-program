#!/usr/bin/env python3

import os
import rospy
from ros import rostopic, rosgraph
from duckietown.dtros import DTROS, NodeType
from std_msgs.msg import String


if __name__ == "__main__":
    print("Hello Python")
    rospy.init_node('camera_control_node')