import rospy
from sensor_msgs.msg import Image as msg_Image
from sensor_msgs.msg import CameraInfo
from geometry_msgs.msg import PointStamped
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

SIZE = 1000

def callback(data_color,data_depth, camera_info, m0, m1, m2, m3, m4):
    global K, depth_array, counter, point_array
    cv_color = bridge.imgmsg_to_cv2(data_color, data_color.encoding)
    cv_depth = bridge.imgmsg_to_cv2(data_depth, data_depth.encoding)
    
    if depth_array is None:
        depth_array = np.array(cv_depth, dtype=np.float32)
        point_array = np.empty((5,3))
        counter = 1    
        point_array[0,:] = [m0.point.x, m0.point.y, m0.point.z]
        point_array[1,:] = [m1.point.x, m1.point.y, m1.point.z]
        point_array[2,:] = [m2.point.x, m2.point.y, m2.point.z]
        point_array[3,:] = [m3.point.x, m3.point.y, m3.point.z]
        point_array[4,:] = [m4.point.x, m4.point.y, m4.point.z]
        
        # print(m0,m1,m2,m3,m4)
        # rospy.signal_shutdown("Ended the iterations")
    elif counter > SIZE:
        np.save("/home/rmhri/markerless-human-perception/src/depth",depth_array/counter)
        np.save("/home/rmhri/markerless-human-perception/src/K",K)
        np.save("/home/rmhri/markerless-human-perception/src/points",point_array/counter)
        rospy.signal_shutdown("Ended the iterations")
    else:
        point_array[0,:] += [m0.point.x, m0.point.y, m0.point.z]
        point_array[1,:] += [m1.point.x, m1.point.y, m1.point.z]
        point_array[2,:] += [m2.point.x, m2.point.y, m2.point.z]
        point_array[3,:] += [m3.point.x, m3.point.y, m3.point.z]
        point_array[4,:] += [m4.point.x, m4.point.y, m4.point.z]
        depth_array += np.array(cv_depth, dtype=np.float32)
        counter += 1

    
    cv2.imwrite("/home/rmhri/markerless-human-perception/src/depth.png",cv2.cvtColor(cv_depth,cv2.COLOR_BGR2RGB))
    cv2.imwrite("/home/rmhri/markerless-human-perception/src/color.png",cv2.cvtColor(cv_color,cv2.COLOR_BGR2RGB))
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
    m0 = message_filters.Subscriber('/natnet_ros/calib_board/marker0/pose', PointStamped)
    m1 = message_filters.Subscriber('/natnet_ros/calib_board/marker1/pose', PointStamped)
    m2 = message_filters.Subscriber('/natnet_ros/calib_board/marker2/pose', PointStamped)
    m3 = message_filters.Subscriber('/natnet_ros/calib_board/marker3/pose', PointStamped)
    m4 = message_filters.Subscriber('/natnet_ros/calib_board/marker4/pose', PointStamped)
    
    
    
    # Setup publisher
    pub = rospy.Publisher('camera2world', String, queue_size=10)
    #rospy.init_node('3Dhpe', anonymous=False)
    
    # Synchronize
    # ts = message_filters.TimeSynchronizer([color_sub, depth_sub, info_sub, m0, m1, m2, m3, m4], 1000)
    ts = message_filters.ApproximateTimeSynchronizer([color_sub, depth_sub, info_sub, m0, m1, m2, m3, m4], 100,0.1, allow_headerless=True)
    ts.registerCallback(callback)
    print("Setup completed, collecting calibration data:")
    rospy.spin()

if __name__ == main():
    try:
        main()
    except rospy.ROSInterruptException:
        rospy.logwarn("Node Not Executed")
    