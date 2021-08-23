# workspace-pollination

## Launch

Once fully functional, launch the following, in the following order:

```

roslaunch kinova_bringup kinova_robot.launch kinova_robotType:=j2n6s300 use_urdf:=true ## launch the robot and the realsense camera
roslaunch j2n6s300_moveit_config bramblebee_arm.launch
roslaunch manipulation_mapping flower_mapper.launch
rosrun manipulation_control ee_go_to_pose_action_node
rosrun manipulation_mapping pre_pose_mapping_ros.py
rosrun manipulation_state_machine planning_ga_ros.py
rosrun manipulation_state_machine state_machine_pollinating.py
rosrun manipulation_state_machine visual_servoing_action_server.py 
rosrun manipulation_pollinator pollinator_control_node
```

Run on a new system to generate lookup table for segmentation service:

```
rosrun manipulation_vision generate_lookup 
```

If not running on Bramblebee (Husky), to start the pollination procedure:

```
rostopic pub -1 /start_pollination_procedures std_msgs/Bool '{data: True}'
```

## Development
- For all manipulation packages, use the `manipulation::` namespace

## Requirements

This software has been tested on Ubuntu 18.04 with ROS Kinetic with OpenCV 3.4.13 as well as OpenCV modules. Please use [these instructions to install OpenCV 3 with the extra modules](https://github.com/wvu-irl/guides-and-resources/wiki/Core-OpenCV-and-Extra-Modules).

This uses submodules, please refer to the [wiki page](https://github.com/wvu-irl/guides-and-resources/wiki/Git-Submodules) on how to work with submodules.

 This software also requires Intel RealSense SDK, please follow these [instructions on how to install](https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md) and make sure to install `librealsense2-dkms`, `librealsense2-utils`, and `librealsense2-dev` using the following command:

```
sudo apt-get install -y librealsense2-dkms=1.3.4-0ubuntu1 librealsense2=2.17.1-0~realsense0.372 librealsense2-utils=2.17.1-0~realsense0.372 librealsense2-dev=2.17.1-0~realsense0.372
```

Intel RealSense working with:
* SDK 1.17.1
* Firmware 5.11.1.0
* intel-ros development branch commit: bc31f2c358037adfa6c3a4e19ccf5845e96ca57a

Also, MoveIt must be installed:
```
sudo apt-get install -y ros-melodic-moveit ros-melodic-trac-ik ros-melodic-moveit-visual-tools
```
If ros-melodic-desktop or ros-melodic-ros-base was installed, then pcl_ros and tf2-geometry_msgs most likely need to be separately installed:
```
sudo apt-get install -y ros-melodic-pcl-ros ros-melodic-tf2-geometry-msgs
```

#Pandas
sudo pip install pandas

## Tensorflow       
Install tensorflow (CPU version) via pip:

      sudo apt-get install -y python-pip python-dev
      sudo pip install -U pip
      pip install -U tensorflow==1.4.0 --user
      python -c "import tensorflow as tf; print(tf.__version__)"

## GTSAM
This software requires [GTSAM 4.0.3](https://github.com/borglab/gtsam/releases/tag/4.0.3). 

Make sure GTSAM is built with correct version of Eigen see https://github.com/erik-nelson/blam/issues/48

## PCL
PCL 1.8 is installed with ROS from the `ros-melodic-pcl-ros` package. An issue may be encountered that is an interaction between a conflicting data type between PCL and OpenCV. The best known fix at the moment is described in this [pull request](https://github.com/PointCloudLibrary/pcl/pull/4266). This issue does not occur on some computers, however, so this fix is likely not the best solution. It has to do with some interaction between particular versions of PCL and OpenCV, when both used in the same project.
