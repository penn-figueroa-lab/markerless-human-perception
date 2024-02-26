import numpy as np
import quaternion
# ROS
import rospy
import message_filters
from std_msgs.msg import String
from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion,TransformStamped
import json
import tf2_ros
import quaternion

def callback(pose_sub, ee_sub):
    pose = json.loads(pose_sub.data)
    
    # For each person get both wrists
    candidates = []
    candidates_c = []
    for p in range(len(pose["value"])):
        # rw = np.array( [pose["value"][p]["RWrist"]["x"],
        #                 pose["value"][p]["RWrist"]["y"],
        #                 pose["value"][p]["RWrist"]["z"]])
        # rw = np.dot(rw,Rt_c2w[:3,:3].transpose())+Rt_c2w[:3,3].transpose()
        lw = np.array( [pose["value"][p]["LWrist"]["x"],
                        pose["value"][p]["LWrist"]["y"],
                        pose["value"][p]["LWrist"]["z"]])
        if lw[2] > 3.5 or lw[2] < 0.2:
            "no wrist"
            return
        candidates_c.append(lw)
        lw = np.dot(lw,Rt_c2w[:3,:3].transpose())+Rt_c2w[:3,3].transpose()
        # candidates.append(rw)
        candidates.append(lw)
    
    # Get the world position of the EE
    ee = np.array([ee_sub.position.x,ee_sub.position.y,ee_sub.position.z])
    ee = np.dot(ee,Rt_f2w[:3,:3].transpose())+Rt_f2w[:3,3].transpose()
    
    best = np.array([float('inf'),float('inf'),float('inf')])
    distance = float('inf')
    best_i = 0
    for i,c in enumerate(candidates):
        d = np.linalg.norm(c-ee)
        if d < distance and d > 0:
            distance = d
            best = c
            best_i = i
    print("ee_f",ee,"wrist_w",best,"wrist_c",candidates_c[best_i])
    
    
    # best = candidates_c[i]
    
    # print(candidates)
    # res = np.dot(np.array(p3d),Rt[:3,:3].transpose())+Rt[:3,3].transpose()

    out_msg = PoseStamped()
    
    # out_msg.pose.position.x = ee[0]
    # out_msg.pose.position.y = ee[1]
    # out_msg.pose.position.z = ee[2]
    out_msg.pose.position.x = best[0]
    out_msg.pose.position.y = best[1]
    out_msg.pose.position.z = best[2]
    out_msg.pose.orientation.x = 1
    out_msg.pose.orientation.y = 0
    out_msg.pose.orientation.z = 0
    out_msg.pose.orientation.w = 0
    
    pub.publish(out_msg)
    
    # Only for debug
    transform = TransformStamped()
    transform.header.stamp = rospy.Time.now()
    transform.header.frame_id = "world"  # Fixed frame
    transform.child_frame_id = "closest_wrist"
    transform.transform.translation = out_msg.pose.position
    transform.transform.rotation = out_msg.pose.orientation
    tf_broadcaster.sendTransform(transform)

    # Only for debug
    # out_msg.pose.position.x = candidates_c[best_i][0]
    # out_msg.pose.position.y = candidates_c[best_i][1]
    # out_msg.pose.position.z = candidates_c[best_i][2]

    # transform = TransformStamped()
    # transform.header.stamp = rospy.Time.now()
    # transform.header.frame_id = "realsense"  # Fixed frame
    # transform.child_frame_id = "closest_wrist"
    # transform.transform.translation = out_msg.pose.position
    # transform.transform.rotation = out_msg.pose.orientation
    # tf_broadcaster.sendTransform(transform)
    
    
    
    camera = Pose()
    camera.position.x = Rt_c2w[0,3]
    camera.position.y = Rt_c2w[1,3]
    camera.position.z = Rt_c2w[2,3]
    q = quaternion.from_rotation_matrix(Rt_c2w[0:3,0:3])
    camera.orientation.x = q.x
    camera.orientation.y = q.y
    camera.orientation.z = q.z
    camera.orientation.w = q.w
    transform = TransformStamped()
    transform.header.stamp = rospy.Time.now()
    transform.header.frame_id = "world"  # Fixed frame
    transform.child_frame_id = "realsense"
    transform.transform.translation = camera.position
    transform.transform.rotation = out_msg.pose.orientation
    tf_broadcaster.sendTransform(transform)
    
def main():
    global pub, Rt_c2w, Rt_f2w,Rt_w2f, tf_broadcaster, Rt_w2c
    rospy.init_node("wrist_tracker")
    
    # Initiate listener
    pose_sub = message_filters.Subscriber('/hpe/poses', String)
    ee_sub = message_filters.Subscriber('/franka_state_controller/ee_pose', Pose)
    
    # Setup publisher
    pub = rospy.Publisher('hpe/ee_goal', PoseStamped, queue_size=10)
    tf_broadcaster = tf2_ros.TransformBroadcaster()
    
    # Rt_c2w = np.loadtxt("/home/rmhri/markerless-human-perception/src/RT_camera2world.csv",delimiter=",")
    Rt_c2w = np.load("/home/rmhri/markerless-human-perception/src/RT_camera2world.npy")
    Rt_f2w = np.load("/home/rmhri/markerless-human-perception/src/RT_world2franka.npy")
    Rt_w2f = np.linalg.inv(Rt_f2w)
    Rt_w2c = np.linalg.inv(Rt_c2w)
    
    # Synchronize
    ts = message_filters.ApproximateTimeSynchronizer([pose_sub, ee_sub], 10, 0.1, allow_headerless=True)
    ts.registerCallback(callback)
    print("Waiting for wrist...")
    rospy.spin()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        rospy.logwarn("Node Not Executed")
