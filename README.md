# ball-chaser-bot

## Overview

These are two packages containing a Gazebo environment with a skid-steer drive robot equipped with a camera and a Hokuyo lidar. There is one node for commanding robot joints and a second one for processing the camera image, detecting round objects, and sending the appropriate velocities commands.

### License

The source code is released under an [MIT license](LICENSE).

**Author/Maintainer: George Sotirchos**

The ball-chaser-bot package has been tested under [ROS] Noetic on Ubuntu 20.04.
This is experimental, personal project code, and possibly subject to frequent changes with any need for explanation disclaimed.

![Example image](media/recording.gif)

## Installation

### Building from Source

#### Dependencies

- [Robot Operating System (ROS)](http://wiki.ros.org) (middleware for robotics),
- [opencv-python](https://github.com/opencv/opencv-python) (Open Source Computer Vision Library)

#### Building

To build from source, clone the latest version from this repository into your catkin workspace and compile the package using

``` bash
cd catkin_ws/src
git clone https://github.com/7555G/ball-chaser-bot
cd ../
rosdep install --from-paths . --ignore-src
catkin_make
```

## Usage

1. Start the Gazebo environment containing the robot:

    ``` bash
    roslaunch my_robot world.launch
    ```

2. Start the `ball_chaser` and the `process_image_opencv` nodes:

    ``` bash
    roslaunch ball_chaser ball_chaser.launch
    ```

    Or to use the "dumb" `process_image` node (without OpenCV):

    ``` bash
    roslaunch ball_chaser ball_chaser.launch use_opencv:=false
    ```

## Launch files

* **my_robot/launch/world.launch:** A Gazebo simulation is opened with 4-wheeled skid-steer drive robot equipped with a camera and a Hokuyo lidar spawned in an office environment containing some balls, along with an RViz window showing the camera image and the lidar measurements.

* **ball_chaser/launch/ball_chase.launch:**

     Arguments:

     - **`use_opencv`**: Whether to use the OpenCV image processing node or the "dumb" one.<br/>
        Default: `true` (Optional)

## Nodes

### drive_bot

Provides a service for publishing drive commands to the robots actuators.

#### Published Topics

* **`/cmd_vel`** ([geometry_msgs/Twist](http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/Twist.html))

    The requested `linear_x` and `angular_z` velocities for the robot wheel joints.

#### Services

* **`/ball_chaser/command_robot`** ([ball_chaser/DriveToTarget](ball_chaser/srv/DriveToTarget.srv))

    Publishes the requested the requested `linear_x` and `angular_z` velocities to `/cmd_vel` topic.

    ```
    rosservice call /ball_chaser/command_robot "linear_x: 0.0
    angular_z: 0.0"  # with the newline
    ```

### process_image_opencv

Processes the camera image published at `/camera/rgb/image_raw` and requests the appropriate drive commands via the `/ball_chaser/command_robot` service.

#### Subscribed Topics

* **`/camera/rgb/image_raw`** ([sensor_msgs/Image](http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/Image.html))

    The robot-mount camera image input.

#### Published Topics

* **`/cmd_vel`** ([geometry_msgs/Twist](http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/Twist.html))

    The requested `linear_x` and `angular_z` velocities for the robot wheel joints.

## Bugs & Feature Requests

Please report bugs and request features using the [Issue Tracker](https://github.com/7555G/ball-chaser-bot/issues).

[ROS]: http://www.ros.org
[Gazebo plugin]: http://gazebosim.org/tutorials?tut=ros_gzplugins#SkidSteeringDrive
[OpenCV]: https://docs.opencv.org/master/

