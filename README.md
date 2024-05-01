RemoteRobotics {#title}
==============

This is a [ROS](https://www.ros.org/)-package developed for controlling
and examining a servo motor farm. It is designed 
to be controlled remotely over an IP-Network.
The testing and development environment for this package was ROS Melodic.

## Usage {#usage}

### Core functionality

The core nodes for controlling the servo motors and reading their angular
positions are started with launch files. These commands have to be executed in
seperate terminals to make node output to stdout clearer.



```
roslaunch remote_robotics hardware.launch
roslaunch remote_robotics segments.launch
roslaunch remote_robotics brain.launch
```

### Launching nodes by hand

Every ROS node outputs launch instructions when trying to launch with wrong/missing
parameters.

The used topics and services can be viewed [`rosservice`](http://wiki.ros.org/rosservice)
and [rostopic](http://wiki.ros.org/rostopic)

### Node overview {#nodes}

#### Image processing

The following nodes
##### remro_cam

Reads camera image and publishes image data.

    rosrun remro_vision remro_cam { <int video source> }

The video source parameter is optional. In case you ignore it 
the program trys '0'(corresponding to /dev/video0) as the image source.

##### img_view

Displays video transferred as `sensor_msgs::ConstImgPtr` or `remote_robotics::MotorImg`.

    rosrun remro_vision img_view <char* msg type> <char* subscriber topic>

##### binary_converter

Converts image to binary image and publishes said image.

    rosrun remro_vision binary_converter 

**Topics**
* /image/binary
##### crop_node

Receives image messages and service requests containing motor index.
It sends image messages of type `MotorImg`.

    rosrun remro_vision crop_node <char* subscriber topic> <char* publisher topic>

##### crop_dims_test

Sends request to image crop node containing motor index obtained from user input.

    rosrun remro_vision crop_dims_test

##### angle_detector

Computes angle from **binary image** of a servo that is assigned the servo number.
It keeps the current angles saved and provides them with a [ROS service](http://wiki.ros.org/Services).

    rosrun remro_vision angle_detector <char* subscriber topic>

**Services**
* /rotor_pos/angle
##### servo_hub

Provides service to set the angular position of the servo motors

    rosrun remote_robotics servo_hub <int device_number>

`device_number` corresponds to pololu device that controls the servos are connected to.

**Services**
* /set_abs_servo_motor

##### brain

Designed to be the "processor" that controls the servo motors and evaluates their positions.

    rosrun remote_robotics brain

