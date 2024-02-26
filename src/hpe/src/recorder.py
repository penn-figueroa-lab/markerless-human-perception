import rospy
from sensor_msgs.msg import Image as msg_Image
from sensor_msgs.msg import CameraInfo
from geometry_msgs.msg import PointStamped
from std_msgs.msg import String
import cv2
from cv_bridge import CvBridge, CvBridgeError
import message_filters
import numpy as np
import os
import sys
import time

import keyboard
keyboard.on_press_key("s", lambda _:save_files())

images = []

def save_files():
    rospy.signal_shutdown("Ended the iterations")


def callback(d435_sub, d455_sub):
    global images
    cv_d435 = bridge.imgmsg_to_cv2(d435_sub, d435_sub.encoding)
    cv_d455 = bridge.imgmsg_to_cv2(d455_sub, d455_sub.encoding)
    
    images.append(cv2.hconcat([cv_d435,cv_d455]))
    
    # cv2.imwrite("/home/rmhri/markerless-human-perception/src/depth.png",cv2.cvtColor(cv_depth,cv2.COLOR_BGR2RGB))
    # cv2.imwrite("/home/rmhri/markerless-human-perception/src/color.png",cv2.cvtColor(cv_color,cv2.COLOR_BGR2RGB))

def main():
    global bridge
    rospy.init_node("recorder")  
    bridge = CvBridge()
    
    # Initiate listeners
    d435_sub = message_filters.Subscriber('/435/color/image_raw', msg_Image)
    d455_sub = message_filters.Subscriber('/455/color/image_raw', msg_Image)
    # Synchronize
    ts = message_filters.ApproximateTimeSynchronizer([d435_sub, d455_sub], 100,0.1, allow_headerless=True)
    ts.registerCallback(callback)
    print("Setup completed, start recording. Press \"s\" to stop")
    rospy.spin()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        rospy.logwarn("Node Not Executed")
    