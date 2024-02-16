import rospy
from sensor_msgs.msg import Image as msg_Image
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import String

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

H135 = 25
F135 = H135 + 40

parts_25 = ["Nose","Neck", "RShoulder", "RElbow", "RWrist", "LShoulder", "LElbow","LWrist", "MidHip",\
            "RHip", "RKnee", "RAnkle", "LHip", "LKnee", "LAnkle", "REye", "LEye", "REar", "LEar", "LBigToe",\
            "LSmallToe", "LHeel", "RBigToe", "RSmallToe", "RHeel", "Background"]
    

import argparse
opWrapper = None
params = dict()

width     = 848
height    = 480
framerate = 60
depth_width = 848
depth_height = 480
depth_framerate = 60

def to_3d(u,v,d, acc):
    m = np.array([u,v,1])
    m = np.transpose(m)
    RES = (inv(K) @ m ) * d
    kp_3d =  {"x" : float(RES[0]), "y": float(RES[1]), "z" : float(RES[2]), "u" : float(u), "v":float(v), "d" : float(d), "acc": float(acc)}

    return kp_3d

def get_3D_keypoints(depth,rgb):
    keypoints_3d = {}
    for key in rgb:
        #print(key)
    
        y = (rgb[key][1] / height) * depth_height
        x = (rgb[key][0] / width)  * depth_width
        
        if y >= depth_height:
            y = depth_height -1
        if x >= depth_width:
            x = depth_width -1
        
        d = depth[int(y)][int(x)]
        x = rgb[key][0]
        y = rgb[key][1]

        if y >= height:
            y = height -1
        if x >= width:
            x = width -1
        acc = rgb[key][2]
        kp3d = to_3d(y,x,d, acc)
        
        # kp3d["name"] = key
        keypoints_3d[key] = kp3d
        
    return keypoints_3d

def load_openpose():
    global opWrapper
    global params
    parser = argparse.ArgumentParser()
    parser.add_argument("--no-display", action="store_true", help="Disable display.")
    args = parser.parse_known_args()
    dir_path = os.path.dirname(os.path.realpath(__file__))    
    params["model_folder"] = "/home/rmhri/openpose/models/"
    params["hand"] = False
    params["face"] = False
    params["net_resolution"] = "-1x368"

    # Add others in path?
    for i in range(0, len(args[1])):
        
        curr_item = args[1][i]
        #print(curr_item)
        if i != len(args[1])-1: next_item = args[1][i+1]
        else: next_item = "1"
        if "--" in curr_item and "--" in next_item:
            key = curr_item.replace('-','')
            if key not in params:  params[key] = "1"
        elif "--" in curr_item and "--" not in next_item:
            key = curr_item.replace('-','')
            if key not in params: params[key] = next_item
    

    opWrapper = op.WrapperPython()
    opWrapper.configure(params)
    opWrapper.start()

def printKeypoints(datum):

    body = datum.poseKeypoints

    kp = []
    if not body is None:
        score = [0]*len(body)
        for person in range(0, len(body)):
            for body_parts in range(0, len(body[person])):
                score[person] += body[person][body_parts][2]
        score_tmp = score[:]
        
        score_tmp.sort(reverse=True)
        

        for i in range(0,len(body)):
            body_candidate = score.index(score_tmp[i])
            
            small_kp = {}
            for body_parts in range(0, len(body[body_candidate])):
                small_kp[parts_25[body_parts]] = [body[body_candidate][body_parts][0],  body[body_candidate][body_parts][1], body[body_candidate][body_parts][2]]
            kp.append(small_kp)
        
    return kp

def openpose_process(image):
    datum = op.Datum()
    datum.cvInputData = image
    opWrapper.emplaceAndPop(op.VectorDatum([datum]))
    return printKeypoints(datum)

def callback(data_color,data_depth, camera_info):
    #print("got data")
    global K
    cv_color = bridge.imgmsg_to_cv2(data_color, data_color.encoding)
    cv_depth = bridge.imgmsg_to_cv2(data_depth, data_depth.encoding)
    depth_array = np.array(cv_depth, dtype=np.float32)
    # cv2.imwrite("/home/rmhri/imgs/depth2.png",cv2.cvtColor(cv_depth,cv2.COLOR_BGR2RGB))
    # cv2.imwrite("/home/rmhri/imgs/depth2.png",cv2.cvtColor(cv_color,cv2.COLOR_BGR2RGB))

    res = openpose_process(cv_color)
    
    # print("Detected",len(res),"people")
    
    K = np.array(camera_info.K).reshape(3,3)
    
    if res:
        for i in range(0, len(res)):
            res[i] = get_3D_keypoints(depth_array,res[i])
        out_msg = { "timestamp" : camera_info.header.stamp.to_sec() , "value" : res}
        out_msg = json.dumps(out_msg)
        pub.publish(out_msg)
    
def main():
    global bridge,pub
    rospy.init_node("human_pose_estimator")  
    bridge = CvBridge()
    load_openpose()    
    # Initiate listeners
    color_sub = message_filters.Subscriber('/camera/color/image_raw', msg_Image)
    depth_sub = message_filters.Subscriber('/camera/aligned_depth_to_color/image_raw', msg_Image)
    info_sub = message_filters.Subscriber('/camera/color/camera_info', CameraInfo)
    
    # Setup publisher
    pub = rospy.Publisher('hpe/poses', String, queue_size=1000)
    #rospy.init_node('3Dhpe', anonymous=False)
    
    # Synchronize
    ts = message_filters.TimeSynchronizer([color_sub, depth_sub, info_sub], 10)
    ts.registerCallback(callback)
    # print("Openpose set up, collecting data.")
    rospy.spin()

if __name__ == main():
    try:
        main()
    except rospy.ROSInterruptException:
        rospy.logwarn("Node Not Executed")
    