#!/usr/bin/python

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped

import numpy as np
import os
import json
import sys
import time

from COMETH import Skeleton, ConstrainedSkeleton
from scipy.optimize import linear_sum_assignment

abs_path = "/home/rmhri/"

labels =  ['LShoulder','RShoulder','LElbow','RElbow','LWrist','RWrist','LHip','RHip','LKnee','RKnee','LAnkle','RAnkle']

s12 = Skeleton(abs_path+'markerless-human-perception/src/hpe/src/Biomechanical-Model/BODY12.xml',3)
s15 = ConstrainedSkeleton(abs_path+'markerless-human-perception/src/hpe/src/Biomechanical-Model/BODY15_constrained_3D.xml')

def compute_distance(A,B):
    distances = np.zeros(A.shape[0])
    for i in range(A.shape[0]):
        distances[i] = np.linalg.norm(A[i,:]-B[i,:])
    return np.nanmean(distances), distances

# Global variables useful for hungarian matching
lost_ext = {}
max_id = -1
prev = None

# Global variables for HPF
from OPFdir.OPF import OPF_3d
acc_thresh = 0.4
filters = {}
p0 = {}

def hugarian_tracking(prev_ext,now_ext):
    global lost_ext,max_id
    
    n_prev = list(prev_ext.keys())
    n_now = list(now_ext.keys())
    
    prev = np.array(list(prev_ext.values()))
    now = np.array(list(now_ext.values()))
        
    if lost_ext:
        n_prev += list(lost_ext.keys())
        lost = np.array(list(lost_ext.values()))
        if prev.shape[0] == 0:
            prev = lost
        else:
            prev = np.vstack((prev,lost))
        for key, value in lost_ext.items():
            prev_ext[key] = value
        lost_ext = {}
    
    max_id = max([int(n) for n in n_prev]+[int(n) for n in n_prev])
    cost_matrix = np.full((np.max([len(n_prev),len(n_now)]),np.max([len(n_prev),len(n_now)])),  np.nan)
    cost_matrix_c = np.full((np.max([len(n_prev),len(n_now)]),np.max([len(n_prev),len(n_now)])),  np.nan)
    for i in range(len(n_prev)):
        for j in range(len(n_now)):
                A = prev[i,:,:3]
                B = now[j,:,:3]
                cost_matrix[i,j], _ = compute_distance(A,B)
                cost_matrix_c[i,j] = np.nanmean(np.abs(prev[i,:,3]-now[j,:,3]))
    if not np.all(np.isnan(cost_matrix)):
        cost_matrix = np.nan_to_num(cost_matrix,nan=np.nanmax(cost_matrix))
    else:
        cost_matrix = np.nan_to_num(cost_matrix,nan=10)
    
    cost_matrix[cost_matrix > 0.5] += 1
    row_indices, col_indices = linear_sum_assignment(cost_matrix)    
    
    for j in row_indices:
        # it means it's fake so it's lost
        if col_indices[j] >= len(n_now):
            # lost.append(prev[j,:]) 
            lost_ext[n_prev[j]] = prev_ext[n_prev[j]]
        elif j >= len(n_prev):
            now_ext[str(max_id + 1)] = now_ext.pop(n_now[col_indices[j]])
        else:    
            now_ext[n_prev[j]] = now_ext.pop(n_now[col_indices[j]])
        
    return now_ext



def callback(pose):
    global prev, filters
    
    pose = json.loads(pose.data)
    refined_poses = {}
    refined_poses["timestamp"] = pose["timestamp"]
    refined_poses["value"] = []

    # Remove skeletons which centroids that are more than 3 meters
    for p in range(len(pose["value"])-1,-1,-1):
        z = []
        for l in labels:
            if pose["value"][p][l]["z"] > 0 :
                z.append(pose["value"][p][l]["z"])
        z = np.array(z)
        if z.shape[0] < 1 or np.mean(z) > 3: #  z.shape[0] < 3 ??
            pose["value"].pop(p)

    # Biomechanical Constraints
    for p in range(len(pose["value"])):
        # From dict to numpy
        X = np.array([ [pose["value"][p][l]["x"],pose["value"][p][l]["y"],pose["value"][p][l]["z"]] for l in labels ])
        X[X == 0] = np.nan
        s12.load_from_numpy(X.reshape(-1,3),labels)
        s15.load_from_BODY12(s12)
        h = s15.estimate_height()
        for b in s15.bones_list:
            min_l = (s15.proportions[b.name][0]-s15.proportions[b.name][1])*h
            max_l = (s15.proportions[b.name][0]+s15.proportions[b.name][1])*h
            # If they are in range, increase confidence
            if b.length > min_l and b.length < max_l:
                if b.src.name in labels:
                    pose["value"][p][b.src.name]["acc"]  = min( pose["value"][p][b.src.name]["acc"]+0.1,1) 
                if b.dest.name in labels:
                    pose["value"][p][b.dest.name]["acc"]  = min( pose["value"][p][b.dest.name]["acc"]+0.1,1) 
            else:
                if b.src.name in labels:
                    pose["value"][p][b.src.name]["acc"]  = max( pose["value"][p][b.src.name]["acc"]-0.1,0) 
                if b.dest.name in labels:
                    pose["value"][p][b.dest.name]["acc"]  = max( pose["value"][p][b.dest.name]["acc"]-0.1,0)

    # Hungarian: build the skeleton
    now = []
    for p in range(len(pose["value"])):
        X = [ [pose["value"][p][l]["x"],pose["value"][p][l]["y"],pose["value"][p][l]["z"],pose["value"][p][l]["acc"]] for l in labels ]
        now.append(X)
    now = np.array(now)
    now[now == 0] = np.nan
    now_ext = {}
    for p in range(len(pose["value"])):
        now_ext[str(p+1)]= now[p,:]
    now = now_ext
    # If nobody is in there
    if now == {}:
        return
    if prev is not None:
        now = hugarian_tracking(prev,now)
    
    # HPF
    for body in list(now.keys()):
        # If there is a new label, add it
        if not body in filters.keys() and not np.any(np.isnan(now[body][labels.index("LWrist"),:])):
            filters[body] = OPF_3d(num_particles=5000, name=body)
            p0[body] = now[body][labels.index("LWrist"),:3]
        elif not np.isnan(now[body][labels.index("LWrist"),3]):
            p = now[body][labels.index("LWrist"),:3]
            c = now[body][labels.index("LWrist"),3]
            filters[body].predict()
            if np.isnan(p[0]) or np.isnan(c) or c < acc_thresh:
                try:
                    print("hidden")
                    filters[body].OP_update(None, 50)
                except:
                    filters[body].trajectory.append(p)
            else:
                print("visible",p,p0[body],c)
                filters[body].update(p-p0[body],confidence=c)
            filters[body].systematic_resample()
            filters[body].resample_from_index()
            if np.any(np.isnan(filters[body].trajectory[-1])):
                print(p,filters[body].trajectory[-1])
                while True:
                    pass
                del filters[body]
            else:
                now[body][labels.index("LWrist"),:3] = filters[body].trajectory[-1] + p0[body]
                print(body,len(filters[body].trajectory))
    prev = now
    
    l = list(now.keys())
    
    r_pose = {}
    r_pose["timestamp"] = pose
    r_pose["value"] = []
    for body in list(now.keys()):
        X = {}
        X["body"] = body
        for i,l in enumerate(labels):
            X[l] = {'x':now[body][i,0],'y':now[body][i,1],'z':now[body][i,2],'acc':now[body][i,3]}
        r_pose["value"].append(X)

    
    out_msg = json.dumps(r_pose)
    pose_pub.publish(out_msg)
    

def main():
    global pose_pub
    rospy.init_node("human_pose_refinement")  
    # Initiate listener
    rospy.Subscriber("hpe/poses", String, callback)
    
    # Setup publisher
    pose_pub = rospy.Publisher('hpe/refined_poses', String, queue_size=1000)
    
    # Synchronize
    rospy.spin()

if __name__ == main():
    try:
        main()
    except rospy.ROSInterruptException:
        rospy.logwarn("Node Not Executed")
