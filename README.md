# mocap4ros2_vicon

[![Build Status](https://travis-ci.com/MOCAP4ROS2-Project/mocap4ros2_vicon.svg?branch=master)](https://travis-ci.com/MOCAP4ROS2-Project/mocap4ros2_vicon)

This project provides support for ROS2 integration with Vicon cameras (MOCAP systems based on vision) and Technaid TechMCS IMUs (MOCAP systems based on motion sensors).


# Install 

Before installing the project you will have to install Vicon DataStream SDK.

## Vicon DataStream SDK

Get the official binaries released in the official download page [here](https://www.vicon.com/software/datastream-sdk/?section=downloads). 
Last version used in this project was 1.11.0.

You can check the documentation [here](https://docs.vicon.com/spaces/viewspace.action?key=DSSDK19).

### Installing on Linux (version 1.11.0)

Uncompress the official binaries and then uncompress the corresponding linux drivers and sources:
```bash
cd ~/Downloads
unzip ViconDataStreamSDK_1.11.0_128037.zip
unzip ViconDataStreamSDK_1.11.0.128037h__Linux64.zip
cd ~/Downloads/Linux64
sudo aptitude install p7zip-full
mkdir ~/Downloads/Linux64/sources
cd sources
7za x ../ViconDataStreamSDKSourceLinux64-source.7z 
```

### Installing on Windows

Uncompress the official binaries, uncompress the corresponding Windows binaries (typically Win64) and run the MSI installer `ViconDataStreamSDK_1.11.0.128037h__x64.msi`.

SDK files are placed to `C:\Program Files\Vicon\DataStream SDK\Win64\CPP` by default, so update the `PATH` environment variable with this SDK path:

`set PATH=%PATH%;C:\Program Files\Vicon\DataStream SDK\Win64\CPP`



## ROS2 Nodes

### ROS2 Humble install under Linux
Once the SDK is installed, we will install this repo and [mocap](https://github.com/MOCAP4ROS2-Project/mocap.git) repository, which is also a dependency. 

First, we clone the repositories:
```bash
cd ~/workspace
mkdir -p mocap/src
cd mocap/src
git clone https://github.com/MOCAP4ROS2-Project/mocap_msgs.git
git clone https://github.com/MOCAP4ROS2-Project/mocap.git
git clone https://github.com/MOCAP4ROS2-Project/mocap4ros2_vicon.git
```

Next, we move Vicon SDK sources (and cpp test) to our repo:
```bash
cd ~/workspace/mocap/src/mocap4ros2_vicon/vicon2_driver
mkdir vicon_sdk
cd vicon_sdk
cp -rf ~/Downloads/Linux64/sources/Vicon/CrossMarket/DataStream .
cp -rf ~/Downloads/Linux64/sources/Vicon/CrossMarket/StreamCommon .
cp ~/Downloads/Linux64/ViconDataStreamSDK_CPPTest.cpp ~/workspace/mocap/src/mocap4ros2_vicon/vicon2_driver/src/ViconDataStreamSDK_CPPTest.cpp
cd ~/workspace/mocap/src/mocap4ros2_vicon/vicon2_driver/vicon_sdk/DataStream/
rm -rf *Test
rm -rf ~/Downloads/Linux64
rm -rf ~/Downloads/ViconDataStreamSDK_1.11.0.*
```

We need to change one include line in `ViconDataStreamSDK_CPPTest.cpp` in order to find the header in our package:
```bash
#include "ViconDataStreamSDK_CPP/DataStreamClient.h"
```

Now, we install any missing dependency:
```bash
cd ~/workspace/mocap/src
rosdep install --from-paths . --ignore-src --rosdistro humble -r -y 
```

And compile:
```bash
cd ~/workspace/mocap/
colcon build --symlink-install
```

Now you can launch MOCAP4ROS2 nodes. Remember to "source" this Vicon workspace before with one of these commands:
```bash
# Linux
source ~/workspace/mocap/install.setup.bash

# Windows
call local_setup.bat
```


# Running MOCAP4ROS2

## Set-up
- Connect your ROS2 computer/laptop to the same network as Vicon system is connected.
- Run Nexus (Vicon software) and calibrate cameras (if required).
- Edit some parameters on configuration file (`mocap4ros2_drivers/vicon2_driver/config/vicon2_driver_params.yaml`) to match your setup: e.g. `host_name` parameter.

## Starting  Vicon-ROS2 driver
- `vicon2_driver` is a lifecycle node. It can be started using the provided launcher:
```bash
ros2 launch vicon2_driver vicon2.launch.py
```
Once ready, we can activate it using different states. See next section for details about it. 

- `vicon2_driver` node works at the frequency established by the Vicon System.
- It will provide two publishers:
    - One with the markers at  `[tracked_frame_suffix_]/markers`
    - One TransformBroadcaster sending TFs to `/tf`

## Lifecyclenode state management
- `vicon2_driver` is a lifecycle node with different states, to know the different states you can run the next command in a terminal: 
```bash
ros2 lifecycle list vicon2_driver_node
- cleanup [2]
	Start: inactive
	Goal: cleaningup
- activate [3]
	Start: inactive
	Goal: activating
- shutdown [6]
	Start: inactive
	Goal: shuttingdown
```
- Once you have launched the `vicon2_driver` you can activate it by running in other terminal:
```bash
ros2 lifecycle set vicon2_driver_node activate
```

- If you want to stop the vicon2_driver you have to run in other terminal:
```bash
ros2 lifecycle set vicon2_driver_node shutdown
```

# About MOCAP4ROS2


The project [MOCAP4ROS2](https://rosin-project.eu/ftp/mocap4ros2) is funded as a Focused Technical Project by [ROSIN](http://rosin-project.eu/).


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
