# DroneArm control algorithms

> Environment was created on Debian Buster for Raspberry Pi 4 (Raspbian64)
> ROS Noetic

## Installation

### 0.Packet dependencies

- [mavros](https://docs.px4.io/master/en/ros/mavros_installation.html)
- [rosbridge-server](https://github.com/RobotWebTools/rosbridge_suite/tree/ros1)
- [vicon_bridge](https://github.com/ethz-asl/vicon_bridge)
- [usb_cam](https://github.com/ros-drivers/usb_cam)

> NOTE: vicon_bridge is available on GitHub. Please, build it from source.

```bash
sudo apt install ros-noetic-rosbridge-server ros-noetic-mavros ros-noetic-mavros-extras ros-noetic-usb-cam
```

### 1.Load PX4 configuration parameters

Using QGroundControl load `drone_arms.params` if PixhawkAP version < 1.12 or `drone_arms_new_fw.params` otherwise. This parameters includes PID calibration and necessary mappings for MoCap and mavros,
such as enabled TELEM2 port with 92100 baud rate and parameters described in [PX4 MoCap article](https://docs.px4.io/master/en/ros/external_position_estimation.html).

### 2.Verify your Network parameters, Vicon IP & host IP in `drone_arm.launch`

### 3.Build ROS package in your workspace

```bash
catkin_make
```

## Script Execution

### For the actual testing environment

```bash
roslaunch px4_control drone_arm.launch # Drone + Vicon + Camera + ROS Bridge + Controls 
```

### For simulation

Firstly, you should run the simulation environment (jmavsim or gazebo, see [PX4 Docs](https://docs.px4.io/master/en/simulation/) or root [README.md](../../README.md)). Then to launch controls, you may pass this command:

```bash
roslaunch px4_control px4_sim.launch # Drone + ROS Bridge + Controls 
```

### After, you could run Unity with ROS# stack

or check control with a simple keyboard teleop script:

```bash
rosrun px4_control keyboard_pose.py
```

---

## Legacy ReadMe

### Example usage

Prerequisites (tested on Ubuntu 16.04 LTS operating system):

1. Install HTC Vive SteamVR.
2. ```pip2 install openvr``` OR [download](https://github.com/cmbruns/pyopenvr/releases) the installer.
3. Setup VR environment. You can follow an [example](https://wiki.bitcraze.io/doc:lighthouse:setup) from Bitcraze.

Launch jMAVsim simulator:

```bash
make px4_sitl_default jmavsim
```

Establish connection with the drone via mavlink:

```bash
roslaunch px4_control px4_sim.launch
```

Take the VR controller you are going to use to control the drone.
And make sure it is tracked by HTC Vive base stations. You can use the script
[controller_test.py](https://github.com/RuslanAgishev/px4_control/blob/master/scripts/drone_arm/controller_test.py)
to find controller's position relative to the origin.

Start interactive drone control:

```bash
rosrun px4_control angular_interactive_control_vr.py
```
