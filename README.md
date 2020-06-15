# Vicon DataStream SDK

Get the official binaries released in the official download page [here](https://www.vicon.com/software/datastream-sdk/?section=downloads).

You can check the documentation [here](https://docs.vicon.com/spaces/viewspace.action?key=DSSDK19).

## Installing on Linux

Copy all the libraries to /usr/local/lib and move the headers to /usr/local/include/ViconDataStreamSDK/.

```
sudo mv {YOUR_ViconDataStreamSDK}/Linux64/Release/* /usr/local/lib/
cd /usr/local/include/
sudo mkdir ViconDataStreamSDK
sudo mv ../lib/*.h ViconDataStreamSDK/
```

Update the `LD_LIBRARY_PATH` environment variable:

`export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib/`

It's convenient to automatically update this environment variable in your bash session every time a new shell is launched, so type it into your .bashrc file:

`echo "export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib/"`

## Installing on Windows

Execute the installer of your distribution (typically Win64).

SDK files are placed to `C:\Program Files\Vicon\DataStream SDK\Win64\CPP` by default, so update the `PATH` environment variable with this SDK path:

`set PATH=%PATH%;C:\Program Files\Vicon\DataStream SDK\Win64\CPP`

## Guide

- Connect your computer/laptop to the same network as Vicon system is connected.

- Run Nexus (Vicon software) and calibrate cameras (if required).

- Edit some parameters on configuration file (mocap4ros_drivers/vicon2_driver/config/vicon2_driver_params.yaml). \
e.g. `host_name` parameter.

- Launch the Vicon-ROS2 driver launcher: `ros2 launch vicon2_driver vicon2.launch.py`

- Check new topics where Vicon info is received in custom message format and TFs format.
