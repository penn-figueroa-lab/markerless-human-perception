import rospy
from sensor_msgs.msg import Image as msg_Image
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import String
from visualization_msgs.msg import Marker,MarkerArray
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
    
color = [(255,0,0),(0,255,0),(0,0,255),(255,255,0),(255,0,255),(0,0,255),(0,255,255)]

Rt_c2w = np.load("/home/rmhri/markerless-human-perception/src/RT_camera2world.npy")

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
    params["hand"] = False
    params["face"] = False
    params["net_resolution"] = "-1x368"

    # Add others in path?
    for i in range(0, len(args[1])):
        
        curr_item = args[1][i]
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

 

# Display point as transform
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
    
# Display keypoint as sphere
def build_marker(id,p,color):
    marker = Marker()
    marker.header.frame_id = "world"
    marker.header.stamp = rospy.Time.now()
    marker.type = 2
    marker.id = id
    marker.action = 0
    
    # Set the scale of the marker
    marker.scale.x = 0.1
    marker.scale.y = 0.1
    marker.scale.z = 0.1
    
    # Set the color
    marker.color.r = color[0]/255
    marker.color.g = color[1]/255
    marker.color.b = color[2]/255
    marker.color.a = 1.0
    
    # Set the pose of the marker
    p = np.dot(p,Rt_c2w[:3,:3].transpose())+Rt_c2w[:3,3].transpose()
    marker.pose.position.x = p[0]
    marker.pose.position.y = p[1]
    marker.pose.position.z = p[2]
    marker.pose.orientation.x = 0.0
    marker.pose.orientation.y = 0.0
    marker.pose.orientation.z = 0.0
    marker.pose.orientation.w = 1.0
    return marker

def callback(data_color,data_depth, camera_info):
    #print("got data")
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
                    try:
                        cv_color = cv2.circle(cv_color,(int(res[i][part]["u"]),int(res[i][part]["v"])), 4, color[i], -1)
                    except:
                        print("Rendering error")

    image_message = bridge.cv2_to_imgmsg(cv_color, data_color.encoding)
    image_message.header = data_color.header
    image_pub.publish(image_message)
    
    marker_array = MarkerArray()
    markers = []
    for j,dic in enumerate(res):
        for i,k in enumerate(list(dic.keys())):
            markers.append(build_marker(100*j+i,np.array([dic[k]["x"],dic[k]["y"],dic[k]["z"]]),color[j]))
    marker_array.markers = markers
    marker_pub.publish(marker_array)
    
def main():
    global bridge,pose_pub, tf_broadcaster, image_pub, marker_pub 
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
    marker_pub = rospy.Publisher("hpe/visualization_marker", MarkerArray, queue_size = 2)
    
    # Synchronize
    ts = message_filters.TimeSynchronizer([color_sub, depth_sub, info_sub], 10)
    ts.registerCallback(callback)
    rospy.spin()

if __name__ == main():
    try:
        main()
    except rospy.ROSInterruptException:
        rospy.logwarn("Node Not Executed")
    