# importing the module 
import cv2 
import numpy as np
from numpy.linalg import inv
import itertools 

width     = 848
height    = 480
framerate = 60
depth_width = 848
depth_height = 480
abs_path = "/home/rmhri/markerless-human-perception/"
K = np.load(abs_path + "src/K.npy")
depth = np.load(abs_path + "src/depth.npy")
P3D = np.load(abs_path + "src/points.npy") # Optitrack send in 'm'

import numpy as np

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

def to_3d(u,v,d):
    m = np.array([u,v,1])
    m = np.transpose(m)
    RES = (inv(K) @ m ) * d
    return RES[0:3]
 
# Get Nx3 matrix, output a NxN adjacence matrix with the distance between each i and j points
def get_adjancence_matrix(points):
    assert points.shape == (5,3)
    adj = np.zeros((points.shape[0],points.shape[0]))
    for i in range(len(points)):
        for j in range(len(points)):
            adj[i,j] = np.linalg.norm(points[i,:] - points[j,:])
    return adj
    
def export_RT(p3d):
    permutations = list(itertools.permutations(P3D))
    score = np.empty(len(permutations))
    
    calib_board = np.array([
        [0,     0,      0],
        [0,     100,     0],
        [200,   100,     0],
        [200,   0,      0],
        [170,   30,      0]
    ])/1000
    
    gt = get_adjancence_matrix(calib_board)
    win = np.nan
    w_score = np.inf
    for i in range(len(permutations)):
        adj = get_adjancence_matrix(np.array(permutations[i]))
        score = np.mean(np.abs(adj-gt))
        if score < w_score:
            win = i
            w_score = score
    print("Average error of optitrack:",1000*np.round(np.mean(np.abs(get_adjancence_matrix(np.array(permutations[win]))-gt)),2),"mm")
    Rt = rigid_transform_3D(p3d, np.array(permutations[win]))
    res = np.dot(p3d, Rt[:3,:3].transpose())+Rt[:3,3].transpose()
    print("camera points projected to world frame",res)
    print("optitrack points in world frame",permutations[win])
    
    print("Average error of realsense:",1000*np.round(np.mean(np.abs(get_adjancence_matrix(np.array(res))-gt)),2),"mm")
    print("Average error w.r.t. of optitrack for each axis:", 1000*np.round(np.mean(np.abs(res-np.array(permutations[win])),axis=0),2),"mm")
    np.save(abs_path + "src/RT_camera2world",Rt)
    print(Rt)
    exit()

def click_event(event, x, y, flags, params): 
    global points2d
    
    # checking for left mouse clicks 
    if event == cv2.EVENT_LBUTTONDOWN: 
        print(x, ' ', y) 
        points2d.append([x,y])
        
        # displaying the coordinates  
        font = cv2.FONT_HERSHEY_SIMPLEX 
        cv2.putText(img, str(len(points2d)), (x,y), font, 
                    0.4, (0, 200, 0), 1) 
        cv2.imshow('image', img) 
        if len(points2d)==5:
            p3d = []
            
            # de-project the point in 3d
            for p2d in points2d:
                y = (p2d[1] / height) * depth_height
                x = (p2d[0] / width)  * depth_width
                
                if y >= depth_height:
                    y = depth_height -1
                if x >= depth_width:
                    x = depth_width -1
                d = depth[int(y)][int(x)]
                print("depth:",d)
                x = p2d[0]
                y = p2d[1]
                if y >= height:
                    y = height -1
                if x >= width:
                    x = width -1
                print(y,x,d)
                p3d.append(to_3d(x,y,d))
            try:
                export_RT(np.array(p3d)/1000)
            except Exception as error:
                print(error)
                cv2.destroyAllWindows() 
            cv2.destroyAllWindows() 

points2d = []

# driver function 
if __name__=="__main__": 

    img = cv2.imread(abs_path + "src/color.png", 1) 
  
    cv2.imshow('image', img) 

    cv2.setMouseCallback('image', click_event) 
  
    cv2.waitKey(0)
    
    print("hey")