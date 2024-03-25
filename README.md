# markerless-human-perception
Collection of ROS nodes for 3D Markerless Human Pose Estimation and Marker-less Wrist Following

## Node General Installation

```
git clone https://github.com/penn-figueroa-lab/markerless-human-perception.git
cd markerless-human-perception
source /opt/ros/noetic/setup.bash
catkin_make
source devel/setup.bash
```

## 3D Human Pose Estimation node
TBD

## Human Motion Refinement node
TBD

## Wrist Tracker
TBD

## Passive Velocity Controller
TBD

## Robot Extrinsics
TBD

## Camera Extrinsics
TBD

### Camera stream
Connect the Realsense camera to the PC and run the correct launcher:
```
roslaunch realsense2_camera data/rs_d455_rmhri.launch
```

### Optitrack calibration board
Turn on the OptiTrack device. Turn on the Motive app, load the calibration file, and check if the asset "calib_board" is selected.

Then, run the modified version of natnet_ros_cpp:
```
roslaunch natnet_ros_cpp natnet_ros.launch
```

## Run demo #0: real-time motion tracking

### Camera stream
On the PC connected to the robot, launch the two camera streams:
```
roslaunch realsense2_camera rs_d435_rmhri.launch
roslaunch realsense2_camera rs_d455_rmhri.launch
```

On PC1 run the real-time 3D pose estimator (i.e., OpenPose):
```
cd scritps
python3 human_pose_estimator.py
```