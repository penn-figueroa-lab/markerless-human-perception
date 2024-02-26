# markerless-human-perception
Collection of ROS nodes for 3D Markerless Human Pose Estimation 

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

## Run Calibration

### Camera stream
On PC0 (10.102.224.77) connect the Realsense camera and the correct launcher in a tab:
```
roslaunch realsense2_camera rs_d435_rmhri.launch
```

### Optitrack calibration board
Turn on the OptiTrack device. Turn on the Motive app, load the calibration file, and check if the asset "calib_board" is selected.

Then, on PC0 (10.102.224.77), run natnet_ros_cpp:
```
roslaunch natnet_ros_cpp natnet_ros.launch
```

## Run demo #0: real-time motion tracking

### Camera stream
On PC0 (10.102.224.77) run realsense with the correct launcher in a tab:
```
roslaunch realsense2_camera rs_d435_rmhri.launch
roslaunch realsense2_camera rs_d455_rmhri.launch json_file_path:=/home/rmhri/d455_config_v1.json
```

On PC1 (10.103.142.166) run the real-time 3D pose estimator (i.e., OpenPose):
```
```