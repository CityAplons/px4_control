# DroneArm control algorithms

> Environment was created on Debian Buster for Raspberry Pi 4 (Raspbian64)
> ROS Noetic

## Installation

### Packet dependencies

- [mavros](https://docs.px4.io/master/en/ros/mavros_installation.html)
- rosbridge-server

    ```bash
        sudo apt install ros-noetic-rosbridge-server
    ```

- [vicon_bridge](https://github.com/ethz-asl/vicon_bridge)
- [robot_upstart](https://github.com/clearpathrobotics/robot_upstart)

### Load PX4 configuration parameters

Using QGroundControl load `drone_arms.params`. This parameters includes PID calibration and necessary mappings for MoCap and mavros,
such as enabled TELEM2 port with 92100 baud rate and parameters described in [PX4 MoCap article](https://docs.px4.io/master/en/ros/external_position_estimation.html).

### Verify your Network parameters, Vicon IP & host IP

### Build ROS package in your workspace

```bash
catkin_make
```

## Script Execution

Main launch file should start automatically during computer boot.
You may check if it's running by `rostopic list` command. Mavros topics will be presented.

Otherwise, you might manually run:

```bash
roslaunch px4_control drone_arm.launch
```

After, you could run control script.

```bash
rosrun px4_control interactive_control.py
```

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
