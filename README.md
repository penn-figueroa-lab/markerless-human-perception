# A Robust Filter for Marker-less Multi-person Tracking in Human-Robot Interaction Scenarios

Please check out our
[paper](https://arxiv.org/pdf/2406.01832) and [project page](https://penn-figueroa-lab.github.io/markerless-human-perception/).

Collection of ROS nodes for 3D marker-less human pose estimation from single RGB-D camera and marker-less wrist following.



<!-- <p align="center">
<iframe width="560" height="315" src="https://www.youtube.com/embed/8sS7gowsk3o?si=UgKJMbCoYi3J_Se9" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>
</p> -->

## Installation

```
git clone https://github.com/penn-figueroa-lab/markerless-human-perception.git
cd markerless-human-perception
source /opt/ros/noetic/setup.bash
catkin_make
source devel/setup.bash
```

### Dependencies
Please follow the installation of the following packages:
- [COMETH](https://github.com/PARCO-LAB/COMETH): biomechanical model of the human body
- [ROS Noetic Ninjemys](https://wiki.ros.org/noetic): inter-nodes communication 
- [Realsense-ROS](https://github.com/IntelRealSense/realsense-ros/tree/ros1-legacy): ROS wrapper for the communication with realsense cameras
-  [franka_interactive_controller](https://github.com/penn-figueroa-lab/franka_interactive_controllers): default PassiveDS Controller
- [OpenPose](https://github.com/CMU-Perceptual-Computing-Lab/openpose): as the HPE backbone, we choose to use OpenPose

## Nodes description

### Camera stream
Connect the realsense camera to the PC and run with the correct launchfile:
```
roslaunch realsense2_camera data/rs_d455_rmhri.launch
```

### Human Pose Estimation
We developed two different ROS nodes for the HPE task:
- [OpenPose](scripts/human_pose_estimator.py): lighter version for fast inference
- [OpenPose with hands](scripts/human_pose_estimator_hands.py): heavier but with better precision and more keypoints

### Wrist tracker
[This node](scripts/wrist_tracker.py) reads the poses from the HPE backbone and publishes the target to follow at each frame.

### Filter
[This node](scripts/filter.py) is the implementation of our filter. It reads the poses from the HPE backbone and after filtering publishes the target to follow at each frame. When the filter is enables, it doesn't need the wrist tracker node.

### Passive Velocity Controller
[This nose](scripts/franka_follow_target_pose.py) sends the desired end-effector velocity to the passive controller.
To enable passive controller, launch:
```
roslaunch franka_interactive_controllers franka_interactive_bringup.launch
```

## Calibration 
We prepared a set of scripts usefull for retrieving 4x4 RT matrices to transform a point from the franka coordinate system and the camera coordinate system to the Optitrack one.
<!-- ![plot](./static/calib_board.jpg) -->
<p align="center">
<img src="./static/calib_board.jpg" width="320"/>
</p>

The .dwg file of calibration board can be found [here](data/calib_board.dwg).

### Optitrack calibration board (ground truth)
Turn on the OptiTrack device. Turn on the Motive app, load the calibration file.
Then, run the [modified version](https://github.com/hparekh15/natnet_ros_cpp) of natnet_ros_cpp:

```
roslaunch natnet_ros_cpp natnet_ros.launch
```

### Camera calibration
[This node](scripts/calibration_realsense.py) records the RGB-D stream, along with the intrinsics matrix. Please note that the calibration board must be seen by both the camera and Optitrack. Then, [this tool](scripts/offline_calibration_tool.py) extract the 5 points of the calibration board from the camera recordings.

### Franka calibration
[This node](scripts/calibration_franka.py) collects the end-effector positions in both robot coordinate system and Optitrack. This can be used later to obtain the 4x4 RT matrix using the [Kabsch-Umeyama algorithm](https://en.wikipedia.org/wiki/Kabsch_algorithm#:~:text=The%20Kabsch%20algorithm%2C%20also%20known,two%20paired%20sets%20of%20points.) .

<!-- ### Camera stream
On the PC connected to the robot, launch the two camera streams:
```
roslaunch realsense2_camera rs_d435_rmhri.launch
roslaunch realsense2_camera rs_d455_rmhri.launch
```

On PC1 run the real-time 3D pose estimator (i.e., OpenPose):
```
cd scritps
python3 human_pose_estimator.py
``` -->

## Citation
If you are going to use some of these nodes for a scientific research, please cite the work below:
```
@inproceedings{Martini2024,
  title={A Robust Filter for Marker-less Multi-person Tracking in Human-Robot Interaction Scenarios},
  author={Martini, Enrico and Parekh, Harshil and Peng, Shaoting and Bombieri, Nicola and Figueroa, Nadia},
  booktitle={2024 33rd IEEE International Conference on Robot and Human Interactive Communication (RO-MAN)},
  pages={1-6},
  year={2024},
  organization={IEEE}
}
```
