# A Robust Filter for Marker-less Multi-person Tracking in Human-Robot Interaction Scenarios
Collection of ROS nodes for 3D marker-less human pose estimation from single RGB-D camera and marker-less wrist following.

## Nodes Installation

```
git clone https://github.com/penn-figueroa-lab/markerless-human-perception.git
cd markerless-human-perception
source /opt/ros/noetic/setup.bash
catkin_make
source devel/setup.bash
```

## Passive Velocity Controller
We use the default PassiveDS Controller from the [franka_interactive_controller](https://github.com/penn-figueroa-lab/franka_interactive_controllers) repository. Follow the steps in the link to complete the setup. Then launch:

```
roslaunch franka_interactive_controllers franka_interactive_bringup.launch
```

### Camera stream
Connect the Realsense camera to the PC and run the correct launcher:
```
roslaunch realsense2_camera data/rs_d455_rmhri.launch
```

### Optitrack calibration board
Turn on the OptiTrack device. Turn on the Motive app, load the calibration file.

Then, run the modified version of natnet_ros_cpp:
```
roslaunch natnet_ros_cpp natnet_ros.launch
```

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
If you are going to use part of these nodes for a scientific paper, please cite the work below:
```
@inproceedings{Martini2024,
  title={A Robust Filter for Marker-less Multi-person Tracking in Human-Robot Interaction Scenarios},
  author={Martini, Enrico and Parekh, Harshil and Peng, Shaoting and Bombieri, Nicola and Figueroa, Nadia},
  booktitle={2024 33nd IEEE International Conference on Robot and Human Interactive Communication (RO-MAN)},
  pages={1-6},
  year={2024},
  organization={IEEE}
}
```