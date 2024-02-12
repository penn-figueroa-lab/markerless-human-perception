# importing the module 
import cv2 
import numpy as np
from numpy.linalg import inv

width     = 848
height    = 480
framerate = 60
depth_width = 848
depth_height = 480

K = np.load("/home/rmhri/catkin_ws/src/K.npy")
depth = np.load("/home/rmhri/catkin_ws/src/depth.npy")

import numpy as np
# Input: expects 3xN matrix of points
# Returns R,t
# R = 3x3 rotation matrix
# t = 3x1 column vector
def rigid_transform_3D(A, B):
    assert A.shape == B.shape

    num_rows, num_cols = A.shape
    if num_rows != 3:
        raise Exception(f"matrix A is not 3xN, it is {num_rows}x{num_cols}")

    num_rows, num_cols = B.shape
    if num_rows != 3:
        raise Exception(f"matrix B is not 3xN, it is {num_rows}x{num_cols}")

    # find mean column wise
    centroid_A = np.mean(A, axis=1)
    centroid_B = np.mean(B, axis=1)

    # ensure centroids are 3x1
    centroid_A = centroid_A.reshape(-1, 1)
    centroid_B = centroid_B.reshape(-1, 1)

    # subtract mean
    Am = A - centroid_A
    Bm = B - centroid_B

    H = Am @ np.transpose(Bm)

    # sanity check
    #if linalg.matrix_rank(H) < 3:
    #    raise ValueError("rank of H = {}, expecting 3".format(linalg.matrix_rank(H)))

    # find rotation
    U, S, Vt = np.linalg.svd(H)
    R = Vt.T @ U.T

    # special reflection case
    if np.linalg.det(R) < 0:
        print("det(R) < R, reflection detected!, correcting for it ...")
        Vt[2,:] *= -1
        R = Vt.T @ U.T

    t = -R @ centroid_A + centroid_B

    return R, t



def to_3d(u,v,d):
    m = np.array([u,v,1])
    m = np.transpose(m)
    RES = (inv(K) @ m ) * d
    return RES[0:3]
 
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
            print(points2d)
            
            # de-project the point in 3d
            for p2d in points2d:
                y = (p2d[1] / height) * depth_height
                x = (p2d[0] / width)  * depth_width
                
                if y >= depth_height:
                    y = depth_height -1
                if x >= depth_width:
                    x = depth_width -1
                d = depth[int(y)][int(x)]
                x = p2d[0]
                y = p2d[1]
                if y >= height:
                    y = height -1
                if x >= width:
                    x = width -1
                p3d = to_3d(y,x,d)
                print(p3d)
            
            cv2.destroyAllWindows() 

points2d = []

# driver function 
if __name__=="__main__": 

    img = cv2.imread("/home/rmhri/catkin_ws/src/color.png", 1) 
  
    cv2.imshow('image', img) 

    cv2.setMouseCallback('image', click_event) 
  
    cv2.waitKey(0)     