#!/usr/bin/env python
"""
usage:
rosrun proteus_demo AvoidObstacleKB.py cmd:=/ATRV/Motion_Controller

http://www.ros.org/wiki/Remapping%20Arguments
"""

import roslib
roslib.load_manifest('rospy')
roslib.load_manifest('geometry_msgs')
import rospy
from geometry_msgs.msg import Twist
import sys
import curses
import threading

class EventCurses(threading.Thread):
    def __init__(self):
        threading.Thread.__init__(self)
        self._map = {}
        self._listen = False
    def run(self):
        self._listen = True
        stdscr = curses.initscr()
        curses.noecho()
        stdscr.keypad(1)
        while self._listen:
            c = stdscr.getch() # read key
            self._new_event(c)
        # terminate curses
        curses.endwin()

    def stop(self):
        self._listen = False
    def connect(self, key, function):
        self._map[key] = function
    def _new_event(self, key):
        if key in self._map.keys():
            self._map[key]()

class TwistKB(object):
    def __init__(self):
        self._cmd = Twist()
        self._kb = EventCurses()
        self.topic = '/cmd'
        self.connect()
    def connect(self):
        self._kb.connect(ord('x'), self.cmd_quit)
        self._kb.connect(ord('z'), self.cmd_forward)
        self._kb.connect(ord('s'), self.cmd_backward)
        self._kb.connect(ord('q'), self.cmd_left)
        self._kb.connect(ord('d'), self.cmd_right)
        #self._kb.connect(ord('w'), self.cmd_forward)
        #self._kb.connect(ord('a'), self.cmd_left)
        self._kb.connect(curses.KEY_UP,    self.cmd_forward)
        self._kb.connect(curses.KEY_DOWN,  self.cmd_backward)
        self._kb.connect(curses.KEY_LEFT,  self.cmd_left)
        self._kb.connect(curses.KEY_RIGHT, self.cmd_right)
        self._kb.connect(curses.KEY_EXIT,  self.cmd_quit)
        self._kb.connect(27,  self.cmd_quit) # escape

    def cmd_quit(self):
        rospy.signal_shutdown("cmd_quit")
    def cmd_forward(self):
        self._cmd.linear.x = 1 # forward
    def cmd_backward(self):
        self._cmd.linear.x = -1 # backward
    def cmd_left(self):
        self._cmd.angular.z = 1 # turn left
    def cmd_right(self):
        self._cmd.angular.z = -1 # turn right

    def run(self):
        rospy.init_node('AvoidObstacleKB')
        publisher = rospy.Publisher(self.topic, Twist)
        self._kb.start()
        while not rospy.is_shutdown():
            publisher.publish(self._cmd)
            self._cmd.linear.x *= .9
            self._cmd.angular.z *= .9
            rospy.sleep(.1)
        # node is shutdown, stop listenning
        self._kb.stop()

def main(argv):
    tkb = TwistKB()
    tkb.run()
    return 0

if __name__ == "__main__":
    sys.exit(main(sys.argv))
