import rospy
from sensor_msgs.msg import Image as msg_Image
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import String
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

import cv2
from cv_bridge import CvBridge, CvBridgeError
import message_filters
import numpy as np
import os
from numpy.linalg import inv
import json
import sys
sys.path.append('/usr/local/python')
from openpose import pyopenpose as op
import time


width     = 848
height    = 480
framerate = 60
depth_width = 848
depth_height = 480
depth_framerate = 60

depth_array = None

def callback(data_color,data_depth, camera_info):
    global K, depth_array, counter
    cv_color = bridge.imgmsg_to_cv2(data_color, data_color.encoding)
    cv_depth = bridge.imgmsg_to_cv2(data_depth, data_depth.encoding)
    
    if depth_array is None:
        depth_array = np.array(cv_depth, dtype=np.float32)
        counter = 1
    
    elif counter > 300:
        np.save("/home/rmhri/catkin_ws/src/depth",depth_array/counter)
        np.save("/home/rmhri/catkin_ws/src/K",K)
        rospy.signal_shutdown("Ended the iterations")
    else:
        depth_array += np.array(cv_depth, dtype=np.float32)
        counter += 1
    
    
    cv2.imwrite("/home/rmhri/catkin_ws/src/depth.png",cv2.cvtColor(cv_depth,cv2.COLOR_BGR2RGB))
    cv2.imwrite("/home/rmhri/catkin_ws/src/color.png",cv2.cvtColor(cv_color,cv2.COLOR_BGR2RGB))
    K = np.array(camera_info.K).reshape(3,3)
    
    print(counter)
    
def main():
    global bridge,pub
    rospy.init_node("RGBD_extrinsics_calibration")  
    bridge = CvBridge()
    
    # Initiate listeners
    color_sub = message_filters.Subscriber('/camera/color/image_raw', msg_Image)
    depth_sub = message_filters.Subscriber('/camera/aligned_depth_to_color/image_raw', msg_Image)
    info_sub = message_filters.Subscriber('/camera/color/camera_info', CameraInfo)
    
    
    
    # Setup publisher
    pub = rospy.Publisher('camera2world', String, queue_size=10)
    #rospy.init_node('3Dhpe', anonymous=False)
    
    # Synchronize
    ts = message_filters.TimeSynchronizer([color_sub, depth_sub, info_sub], 10)
    ts.registerCallback(callback)
    rospy.spin()

if __name__ == main():
    try:
        main()
    except rospy.ROSInterruptException:
        rospy.logwarn("Node Not Executed")
    