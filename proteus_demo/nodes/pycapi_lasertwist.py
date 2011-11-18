#!/usr/bin/env python
import roslib
roslib.load_manifest('rospy')
roslib.load_manifest('geometry_msgs')
roslib.load_manifest('sensor_msgs')
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

import rawlasertwist # Python/C-API (docs.python.org/c-api)

"""
This is an example on how to interface C code without any link to ROS to via the
Python/C-API (docs.python.org/c-api).
usage:
rosrun proteus_demo pycapi_lasertwist.py cmd:=/ATRV/Motion_Controller laser:=/ATRV/Sick
"""

def handle_laser(msg):
    cmd = Twist()
    velocity = rawlasertwist.rawlasertwist(msg.ranges)
    cmd.linear.x = velocity[0]
    cmd.angular.z = velocity[1]
    topic.publish(cmd)

"""
http://www.ros.org/doc/api/sensor_msgs/html/msg/LaserScan.html
"""
if __name__ == '__main__':
    rospy.init_node('lasertwist')
    topic = rospy.Publisher('/cmd', Twist)
    rospy.Subscriber('/laser', LaserScan, handle_laser)
    rospy.spin()

