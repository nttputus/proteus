#!/usr/bin/env python
import roslib
roslib.load_manifest('rospy')
roslib.load_manifest('geometry_msgs')
roslib.load_manifest('sensor_msgs')
import rospy
from sensor_msgs.msg import Range
from geometry_msgs.msg import Twist

data = {
  'left': 100.0,
  'right': 100.0,
  'update': 2
}

def handle_ir_left(msg):
    data['left'] = msg.range
    #data['update'] += 1
    #print("left: %f"%msg.range)

def handle_ir_right(msg):
    data['right'] = msg.range
    #data['update'] += 1
    #print("right: %f"%msg.range)

"""
http://www.ros.org/doc/api/sensor_msgs/html/msg/LaserScan.html
"""
if __name__ == '__main__':
    rospy.init_node('AvoidObstacleIR')
    topic=rospy.Publisher('/ATRV/Motion_Controller', Twist)
    rospy.Subscriber('/ATRV/InfraredR', Range, handle_ir_right)
    rospy.Subscriber('/ATRV/InfraredL', Range, handle_ir_left)
    while not rospy.is_shutdown():
        #print("update: %i"%data['update'])
        #if data['update'] > 1:
        #data['update'] = 0
        cmd = Twist()
        # halt if an object is less than 2m in a 30deg angle
        # TODO cf Range.max_range
        if data['left'] < 6 or data['right'] < 6:
            print(" l:%f ; r:%f "%(data['left'], data['right']))
            # we go to the highest-range side scanned
            if data['left'] < data['right']:
                cmd.angular.z = -1
            else:
                cmd.angular.z = 1
        else:
            cmd.linear.x = 1
        # publish twist
        topic.publish(cmd)
        rospy.sleep(1.0)

