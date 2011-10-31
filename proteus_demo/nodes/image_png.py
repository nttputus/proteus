#!/usr/bin/env python
"""
Save ROS Image to PNG
(template is 'image.%09i.png'%image.header.seq)
usage:
rosrun proteus_demo image_png.py image:=/ATRV/CameraMain

TIPS: use MEncoder to build a video:
mencoder mf://image.*.png -mf fps=20:type=png -sws 6 -o imageROS.avi -ovc x264 -nosound
"""
import roslib
roslib.load_manifest('rospy')
roslib.load_manifest('sensor_msgs')
import rospy
from sensor_msgs.msg import Image

import wx
import sys

def handle_image(image):
    bmp = None
    if image.encoding == 'rgba8':
        bmp = wx.BitmapFromBufferRGBA(image.width, image.height, image.data)
    elif image.encoding == 'rgb8':
        bmp = wx.BitmapFromBuffer(image.width, image.height, image.data)
    else:
        print("encoding not supported: %s"%image.encoding)
        return
    bmp.SaveFile("image.%09i.png"%image.header.seq, wx.BITMAP_TYPE_PNG)

def main(argv):
    app = wx.App() # prevent from: You forgot to call g_type_init()
    rospy.init_node('ImagePNG')
    rospy.Subscriber('/image', Image, handle_image)
    print(__doc__)
    rospy.spin()
    return 0

if __name__ == "__main__":
    sys.exit(main(sys.argv))
