# mocap4ros2_vicon

[![rolling](https://github.com/MOCAP4ROS2-Project/mocap4ros2_vicon/actions/workflows/rolling.yaml/badge.svg)](https://github.com/MOCAP4ROS2-Project/mocap4ros2_vicon/actions/workflows/rolling.yaml)

This project provides support for ROS2 integration with Vicon cameras (MOCAP systems based on vision) as part of the project [MOCAP4ROS2](https://rosin-project.eu/ftp/mocap4ros2).

# Installation

## Dependencies:
Vicon drivers for ROS2 are based on Vicon DataStream SDK 1.11.0. When compiling the `mocap4r2_vicon_driver`, SDK is downloaded and installed, requiring packages `wget` and `p7zip-full` for this.
Also, our package depends on other two repositories from MOCAP4ROS2 Project:
- [mocap_interfaces](https://github.com/MOCAP4ROS2-Project/mocap_interfaces)
- [mocap4r2_control](https://github.com/MOCAP4ROS2-Project/mocap)

A rosinstall file is provided to automatically manage them. 

## Installation steps:
The following commands cover all the necessary steps to install DataStream SDK, dependencies and Vicon driver:

```bash
## dependencies
sudo apt-get install wget git p7zip-full 

## create workspace
mkdir -p ~/mocap4r2_ws/src
cd ~/mocap4r2_ws/src

## clone vicon repository
git clone https://github.com/MOCAP4ROS2-Project/mocap4ros2_vicon.git -b rolling
## clone necessary mocap repositories
cp mocap4ros2_vicon/mocap4ros2.rosinstall .rosinstall
wstool update

## manage ros2 dependencies for our workspace
# rosdep init may not be needed
sudo rosdep init
rosdep update
rosdep install --from-paths . --ignore-packages-from-source --rosdistro humble -y

## build all
cd ~/mocap4r2_ws/
colcon build --symlink-install --packages-up-to mocap4r2_vicon_driver
```

# Configuration

Check `config/mocap4r2_vicon_driver_params.yaml` for the available parameters. Note that `host_name` should be the IP address and port of the machine running VICON tracker. 

# Launching
After sourcing the workspace, the Vicon driver can be started using the provided launcher:

`ros2 launch mocap4r2_vicon_driver mocap4r2_vicon_driver_launch.py`

Remember that the vicon driver is a lifecycle node, so it needs to be signaled to start:

`ros2 lifecycle set /mocap4r2_vicon_driver_node activate`

# About MOCAP4ROS2

The project [MOCAP4ROS2](https://rosin-project.eu/ftp/mocap4ros2) is funded as a Focused Technical Project by [ROSIN](http://rosin-project.eu/). 
Visit [mocap4ros2-project github.io](https://mocap4ros2-project.github.io/) for additional info and documentation. 


<a href="http://rosin-project.eu">
  <img src="http://rosin-project.eu/wp-content/uploads/rosin_ack_logo_wide.png"
       alt="rosin_logo" height="60" >
</a>

Supported by ROSIN - ROS-Industrial Quality-Assured Robot Software Components.  
More information: <a href="http://rosin-project.eu">rosin-project.eu</a>

<img src="http://rosin-project.eu/wp-content/uploads/rosin_eu_flag.jpg"
     alt="eu_flag" height="45" align="left" >  

This project has received funding from the European Union’s Horizon 2020  
research and innovation programme under grant agreement no. 732287.

***

<p align="center"> 
<img align="center" src="https://github.com/MOCAP4ROS2-Project/mocap4ros2_exp_and_resources/blob/master/resources/mocap4ros_arch.png" 
    alt="mocap4ros_arch" width="100%">
</p>
