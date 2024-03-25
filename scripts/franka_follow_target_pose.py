#!/usr/bin/env python

import rospy
import geometry_msgs.msg
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion, Twist
import tf2_ros
import tf.transformations as tf
import numpy as np

abs_path = "/home/rmhri/catkin_ws/src/rmhri_franka/"

class PoseController:
    def __init__(self):
        rospy.init_node('pose_controller', anonymous=True)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        self.target_pose = None
        self.ee_pose = None
        # Subscribers
        rospy.Subscriber("/franka_state_controller/ee_pose", Pose, self.ee_pose_callback)

        # Uncomment body to track
        rospy.Subscriber("/hpe/ee_goal", PoseStamped, self.target_pose_callback)

        # Publisher, uncomment controller to use
        self.pub_desired_ee_vel = rospy.Publisher('/passiveDS/desired_twist', Twist, queue_size=10)


        # Define approach distance
        self.x_approach_distance = 0.3
        self.z_approach_distance = 0.15
        self.A = [[1, 0, 0],
                  [0, 1.1, 0],
                  [0, 0, 1]]

        self.world_to_base = np.loadtxt(abs_path + "RT_franka2world.csv", delimiter=',')

        self.base_to_world = np.linalg.inv(self.world_to_base)
        # print("base2World", self.base_to_world)

    def target_pose_callback(self, data):
        target_pose_temp = [data.pose.position.x, data.pose.position.y, data.pose.position.z, data.pose.orientation.x, data.pose.orientation.y, data.pose.orientation.z, data.pose.orientation.w]
        #Convert target pose from world frame to robot pose frame
        pose_matrix = tf.quaternion_matrix(target_pose_temp[3:])
        pose_matrix[:3, 3] = target_pose_temp[:3]
        pose_base_matrix = np.dot(self.base_to_world, pose_matrix)
        pose_base_translation = pose_base_matrix[:3, 3]
        pose_base_quaternion = tf.quaternion_from_matrix(pose_base_matrix)
        self.target_pose = np.concatenate((pose_base_translation, pose_base_quaternion))

    def ee_pose_callback(self, data):
        self.ee_pose = [data.position.x, data.position.y, data.position.z, data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w]
       
    def run_passiveDS(self):
        while True:
            if (self.ee_pose is None) or (self.target_pose is None):
                continue
            else:
                current_position = np.array([self.ee_pose[0], self.ee_pose[1], self.ee_pose[2]])
                desired_position = np.array([self.target_pose[0] - self.x_approach_distance , 
                                         self.target_pose[1], 
                                         self.target_pose[2] + self.z_approach_distance])
                
                desired_cmd_vel = np.dot(self.A, desired_position - current_position)

                if np.isnan(desired_cmd_vel).any() == True:
                    # Create a Twist message instance
                    twist_msg = Twist()
                    
                    # Set linear attribute of the Twist message
                    twist_msg.linear.x = 0
                    twist_msg.linear.y = 0
                    twist_msg.linear.z = 0
                    twist_msg.angular.x = 0
                    twist_msg.angular.y = 0
                    twist_msg.angular.z = 0
                    
                    # Publish the Twist message
                    print("desired_cmd_vel",twist_msg)
                    self.pub_desired_ee_vel.publish(twist_msg)

                print("desired_cmd_vel", desired_cmd_vel)
                # Create a Twist message instance
                twist_msg = Twist()
                
                # Set linear attribute of the Twist message
                twist_msg.linear.x = desired_cmd_vel[0]
                twist_msg.linear.y = desired_cmd_vel[1]
                twist_msg.linear.z = desired_cmd_vel[2]

                # Publish the Twist message
                self.pub_desired_ee_vel.publish(twist_msg)


    def run_pose_impedance(self):

        while True:
            if (self.ee_pose is None) or (self.target_pose is None):
                continue
            else:
                # Define the desired position based on the offset
                desired_position = Point()
                desired_position.x = self.target_pose[0] - self.approach_distance
                desired_position.y = self.target_pose[1] 
                desired_position.z = self.target_pose[2] + self.approach_distance

                # Define the desired orientation (maintaining the same orientation)
                desired_orientation = Quaternion()
                desired_orientation.x = self.ee_pose[3]
                desired_orientation.y = self.ee_pose[4]
                desired_orientation.z = self.ee_pose[5]
                desired_orientation.w = self.ee_pose[6]

                # Create a new Pose message with the desired pose
                desired_pose = PoseStamped()
                desired_pose.pose.position = desired_position
                desired_pose.pose.orientation = desired_orientation

                # Publish the desired pose
                print("desired_pose", desired_pose)
                self.pub_desired_pose.publish(desired_pose)

                # Publish the desired pose as a TF transform
                transform = geometry_msgs.msg.TransformStamped()
                transform.header.stamp = rospy.Time.now()
                transform.header.frame_id = "panda_link0"  # Fixed frame
                transform.child_frame_id = "desired_pose"
                transform.transform.translation = desired_position
                transform.transform.rotation = desired_orientation
                self.tf_broadcaster.sendTransform(transform)

if __name__ == '__main__':
    try:
        pose_controller = PoseController()
        pose_controller.run_passiveDS()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
