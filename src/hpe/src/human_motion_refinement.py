import rospy
from sensor_msgs.msg import Image as msg_Image
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import String

import numpy as np
import os
from numpy.linalg import inv
import json
import sys
sys.path.append('/usr/local/python')
import time

from Biomechanical.Skeleton import Skeleton,ConstrainedSkeleton


parts_25 = ["Nose","Neck", "RShoulder", "RElbow", "RWrist", "LShoulder", "LElbow","LWrist", "MidHip",\
            "RHip", "RKnee", "RAnkle", "LHip", "LKnee", "LAnkle", "REye", "LEye", "REar", "LEar", "LBigToe",\
            "LSmallToe", "LHeel", "RBigToe", "RSmallToe", "RHeel", "Background"]

from io import StringIO

def callback(pose):
    pose = json.loads(pose.data)
    print(pose["timestamp"])
    out_msg = "TBD" # json.dumps({camera_info.header.stamp.to_sec() : res})
    pub.publish(out_msg)
    
def main():
    global pub
    rospy.init_node("human_pose_refinement")  
    # Initiate listener
    rospy.Subscriber("poses", String, callback)
    
    # Setup publisher
    pub = rospy.Publisher('refined_poses', String, queue_size=10)
    
    # Synchronize
    rospy.spin()

if __name__ == main():
    try:
        main()
    except rospy.ROSInterruptException:
        rospy.logwarn("Node Not Executed")
