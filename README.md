# ROS2 Control Hoverboard Hardware Interface

ROS2 Control hardware interface for Hoverboard motors. Implementation in C++ with Boost libraries. All steering and feedback messages are transmitted by serial port communication.

**Work is still in progress, we will be adding new features.**

### Compatible distros

- jazzy
- humble
- foxy (tested just a bit, separate branch, more tests on request)

### Requirements

Hoverboard driver firmware: [ https://github.com/hoverboard-robotics/hoverboard-firmware-hack-FOC]( https://github.com/hoverboard-robotics/hoverboard-firmware-hack-FOC)

All ROS2 and system dependencies are installable by the rosdep command-line tool. Just run the following command:

```sh
rosdep install --from-paths src -y --ignore-src
```

You can learn more about the rosdep command line tool here: [https://docs.ros.org/en/humble/Tutorials/Intermediate/Rosdep.html](https://docs.ros.org/en/humble/Tutorials/Intermediate/Rosdep.html)

### How to run it?

The simplest way to run the whole package is to use the demo launch file. The launch file can be an inspiration for your custom usage of the package.

To use the launch file, just run the following command:

```sh
ros2 launch hoverboard_demo_bringup hoverboard.launch.py
```

### Inspirations

* [https://github.com/Factor-Robotics/odrive_ros2_control](https://github.com/Factor-Robotics/odrive_ros2_control/tree/humble-fw-v0.5.3)
* [https://github.com/joshnewans/diffdrive_arduino](https://github.com/joshnewans/diffdrive_arduino/tree/humble)

**Thank you for your support and inspiration!**

### Tests

The code works well with USB to TTL adapter with a 3.3V voltage level and with a UART1 port on the Jetson Xavier NX Development Kit. The code was tested with ROS2 Humble distribution with a standalone and containerized (Docker) setup.

### Cooperation and contact

If you want to add or change something, just fork this repository and push changes by the pull request. If you need something, but you can't do it yourself, please write an issue. We will do our best to help.

### Team

* [Bart Van der Haagen](https://www.linkedin.com/in/bart-van-der-haagen-b91b10bb/)
* [Robert Gruberski](https://www.linkedin.com/in/rgruberski/)

