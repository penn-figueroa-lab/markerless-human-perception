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
from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion,TransformStamped
import tf2_ros

H135 = 25
F135 = H135 + 40

parts_25 = ["Nose","Neck", "RShoulder", "RElbow", "RWrist", "LShoulder", "LElbow","LWrist", "MidHip",\
            "RHip", "RKnee", "RAnkle", "LHip", "LKnee", "LAnkle", "REye", "LEye", "REar", "LEar", "LBigToe",\
            "LSmallToe", "LHeel", "RBigToe", "RSmallToe", "RHeel", "Background"]
    
hand = ["WristDetailed","Thumb1CMC","Thumb2Knuckles","Thumb3IP","Thumb4FingerTip","Index1Knuckles","Index2PIP","Index3DIP",\
        "Index4FingerTip","Middle1Knuckles","Middle2PIP","Middle3DIP","Middle4FingerTip","Ring1Knuckles",\
        "Ring2PIP","Ring3DIP","Ring4FingerTip","Pinky1Knuckles","Pinky2PIP","Pinky3DIP","Pinky4FingerTip"]


color = [(255,0,0),(0,255,0),(0,0,255),(255,255,0),(255,0,255),(0,0,255)]

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
    kp_3d =  {"x" : float(RES[0])/1000, "y": float(RES[1])/1000, "z" : float(RES[2])/1000, "u" : float(u), "v":float(v), "d" : float(d)/1000, "acc": float(acc)}

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
        kp3d = to_3d(x,y,d, acc)
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
    params["hand"] = True
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
    lhand = datum.handKeypoints[0]
    rhand = datum.handKeypoints[1]

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
            for lh_parts in range(0, len(lhand[body_candidate])):
                small_kp["L" + hand[lh_parts]] = [lhand[body_candidate][lh_parts][0],  lhand[body_candidate][lh_parts][1], lhand[body_candidate][lh_parts][2]]
            for rh_parts in range(0, len(rhand[body_candidate])):
                small_kp["R" + hand[rh_parts]] = [rhand[body_candidate][rh_parts][0],  rhand[body_candidate][rh_parts][1], rhand[body_candidate][rh_parts][2]]
            kp.append(small_kp)
    return kp

def openpose_process(image):
    datum = op.Datum()
    datum.cvInputData = image
    opWrapper.emplaceAndPop(op.VectorDatum([datum]))
    return printKeypoints(datum)

def broadcast_point(frame,name,x,y,z):
    point = Pose()
    point.position.x = x
    point.position.y = y
    point.position.z = z
    point.orientation.x = 1
    point.orientation.y = 0
    point.orientation.z = 0
    point.orientation.w = 0
    transform = TransformStamped()
    transform.header.stamp = rospy.Time.now()
    transform.header.frame_id = frame  # Fixed frame
    transform.child_frame_id = name
    transform.transform.translation = point.position
    transform.transform.rotation = point.orientation
    tf_broadcaster.sendTransform(transform)
    print(transform)

def callback(data_color,data_depth, camera_info):
    global K
    cv_color = bridge.imgmsg_to_cv2(data_color, data_color.encoding)
    cv_depth = bridge.imgmsg_to_cv2(data_depth, data_depth.encoding)
    depth_array = np.array(cv_depth, dtype=np.float32)

    res = openpose_process(cv_color)
    
    K = np.array(camera_info.K).reshape(3,3)
    
    # send pose msg
    if res:
        for i in range(0, len(res)):
            res[i] = get_3D_keypoints(depth_array,res[i])
        out_msg = { "timestamp" : camera_info.header.stamp.to_sec() , "value" : res}
    else:
        out_msg = { "timestamp" : camera_info.header.stamp.to_sec() , "value" : []}
    out_msg = json.dumps(out_msg)
    pose_pub.publish(out_msg)
        
    # send image labeled
    if res:
        for i in range(0, len(res)):
            for part in parts_25:
                if part != 'Background' and res[i][part]["acc"] > 0:
                    cv_color = cv2.circle(cv_color,(int(res[i][part]["u"]),int(res[i][part]["v"])), 4, color[i], -1)
        for i in range(0, len(res)):
            for part in hand:
                if res[i]["L" + part]["acc"] > 0:
                    cv_color = cv2.circle(cv_color,(int(res[i]["L" + part]["u"]),int(res[i]["L" + part]["v"])), 2, color[i], -1)
                if res[i]["R" + part]["acc"] > 0:
                    cv_color = cv2.circle(cv_color,(int(res[i]["R" + part]["u"]),int(res[i]["R" + part]["v"])), 2, color[i], -1)
                    
    image_message = bridge.cv2_to_imgmsg(cv_color, data_color.encoding)
    image_message.header = data_color.header
    image_pub.publish(image_message)
    
def main():
    global bridge,pose_pub, tf_broadcaster, image_pub
    rospy.init_node("human_pose_estimator")  
    bridge = CvBridge()
    load_openpose()    
    # Initiate listeners
    color_sub = message_filters.Subscriber('/455/color/image_raw', msg_Image)
    depth_sub = message_filters.Subscriber('/455/aligned_depth_to_color/image_raw', msg_Image)
    info_sub = message_filters.Subscriber('/455/color/camera_info', CameraInfo)
    
    tf_broadcaster = tf2_ros.TransformBroadcaster()
    
    # Setup publisher
    pose_pub = rospy.Publisher('hpe/poses', String, queue_size=1000)
    image_pub = rospy.Publisher('hpe/image_labeled', msg_Image, queue_size=1000)
    
    # Synchronize
    ts = message_filters.TimeSynchronizer([color_sub, depth_sub, info_sub], 10)
    ts.registerCallback(callback)
    rospy.spin()

if __name__ == main():
    try:
        main()
    except rospy.ROSInterruptException:
        rospy.logwarn("Node Not Executed")
    