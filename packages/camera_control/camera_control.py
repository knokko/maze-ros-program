#!/usr/bin/env python3

import os
import rospy
from ros import rostopic, rosgraph
from duckietown.dtros import DTROS, NodeType
from std_msgs.msg import String

class PublisherNode(DTROS):
    def __init__(self, node_name):
        super(PublisherNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
        self.pub = rospy.Publisher('chatter', String, queue_size=10)

    def run(self):
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            message = "Hello python"
            rospy.loginfo("pub message: '%s'" % message)
            self.pub.publish(message)
            rate.sleep()

if __name__ == "__main__":
    print("Hello Python")
    node = PublisherNode(node_name='camera_control')
    node.run()
    rospy.spin()