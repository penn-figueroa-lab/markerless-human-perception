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
import signal
from datetime import datetime
images = []

abs_path = "/home/rmhri/"

def save_images_as_video(output_path, fps=30):
    # Get the shape of the first image to determine the video dimensions
    height, width, _ = images[0].shape

    # Define the codec and create a VideoWriter object for MP4
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    video_writer = cv2.VideoWriter(output_path, fourcc, fps, (width, height))

    try:
        # Write each frame to the video
        for image in images:
            rgb_image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
            video_writer.write(rgb_image)
    finally:
        # Release the VideoWriter object
        video_writer.release()

def signal_handler(sig, frame):
    global stop
    stop = time.time()
    print("\n",len(images)/(stop-start),"hz")
    save_files(fps=round(len(images)/(stop-start)))
    print("\nCTRL+C pressed. Saving files...")

signal.signal(signal.SIGINT, signal_handler)

def save_files(fps=30):
    if len(images) > 0:
        save_images_as_video(abs_path+"markerless-human-perception/tmp/" + name + ".mp4",fps=fps)
        print("\nSaved",len(images))
    else:
        print("No frame")
    rospy.signal_shutdown("Ended the iterations")

def callback(d435_sub, d455_sub, pose_sub):
    global images
    cv_d435 = bridge.imgmsg_to_cv2(d435_sub, d435_sub.encoding)
    cv_d455 = bridge.imgmsg_to_cv2(d455_sub, d455_sub.encoding)
    pose_sub
    images.append(cv2.hconcat([cv_d435,cv_d455]))
    
def main():
    global bridge, name, start
    
    now = datetime.now()
    name = now.strftime("%Y_%m_%d_%H_%M_%S")
    if len(myargv) > 1:
        name += "_" + myargv[-1]
    rospy.init_node("recorder")  
    bridge = CvBridge()
    
    # Initiate listeners: /hpe/poses /hpe/ee_goal /franka_state_controller/joint_states /franka_state_controller/ee_pose
    d435_sub = message_filters.Subscriber('/435/color/image_raw', msg_Image)
    d455_sub = message_filters.Subscriber('/hpe/image_labeled', msg_Image)
    pose_sub = message_filters.Subscriber('/hpe/poses', String)
    # goal_sub = message_filters.Subscriber('/hpe/ee_goal', PointStamped)
        
    # Synchronize
    ts = message_filters.ApproximateTimeSynchronizer([d435_sub, d455_sub, pose_sub], 10, 0.1, allow_headerless=True)
    ts.registerCallback(callback)
    print("Setup completed, start recording. Press \"CTRL+C\" to stop")
    start = time.time()
    rospy.spin()


if __name__ == "__main__":
    global myargv
    myargv = rospy.myargv(argv=sys.argv)
    print(myargv)
    main()
    