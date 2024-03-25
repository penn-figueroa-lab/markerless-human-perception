import rospy
from sensor_msgs.msg import Image as msg_Image
from sensor_msgs.msg import CameraInfo
from geometry_msgs.msg import PoseStamped, Pose
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

SIZE = 10
f = []
w = []

def rigid_transform_3D(A, B):
    # Transformation matrix that goes from A to B, such as TR*A = B
    # It works using N x 3 or N x 4 points

    centroid_A = np.mean(A, axis=0)
    centroid_B = np.mean(B, axis=0)

    H = np.dot((A - centroid_A).T, (B - centroid_B))

    U, S, V = np.linalg.svd(H, full_matrices=False)

    _, log_det = np.linalg.slogdet(np.dot(V, U.T))
    d = np.sign(log_det)    
    
    
    diagonal = np.array([1, 1, d])
    if A.shape[1] == 4:
        diagonal[3] = 1
    TR = np.dot(np.dot(V.transpose(), np.diag(diagonal)), U.T)   
    
    t_vec = centroid_B - np.dot(TR, centroid_A)

    # Could be a 3x3 matrix, enlarge to 4x4 if not to hold transformation
    if TR.shape[0] == 3:
        TR = np.pad(TR, ((0, 1), (0, 1)), mode='constant', constant_values=0)
        TR[3, 3] = 1
    TR[:3, 3] = t_vec[:3]

    return TR

def callback(franka, world):
    global f, w
    # Get the world position of the EE
    
    p_f = np.array([franka.position.x,franka.position.y,franka.position.z])
    p_w = np.array([world.pose.position.x,world.pose.position.y,world.pose.position.z])
    if not np.any(np.round(p_f,3) == np.array([np.round(p,3) for p in f])) and \
       not np.any(np.round(p_w,3) == np.array([np.round(p,3) for p in w])):
        f.append(p_f)
        w.append(p_w)
    
    
    
    if len(f) % SIZE == 0:
        RT = rigid_transform_3D(np.array(w),np.array(f))
        print(np.linalg.inv(RT))
        np.savetxt("RT_world2franka.csv",RT)
        
    

    
def main():
    global bridge
    rospy.init_node("franka_extrinsics_calibration")  
    bridge = CvBridge()
    
    # Initiate listeners
    ee_franka = message_filters.Subscriber('/franka_state_controller/ee_pose', Pose)
    ee_world  = message_filters.Subscriber('/natnet_ros/frankaEE/pose', PoseStamped)
        
    # Synchronize
    ts = message_filters.ApproximateTimeSynchronizer([ee_franka, ee_world], 100,0.1, allow_headerless=True)
    ts.registerCallback(callback)
    print("Setup completed, collecting calibration data:")
    rospy.spin()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        rospy.logwarn("Node Not Executed")