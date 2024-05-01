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
roslaunch remote_robotics angle_detection.launch
```
```
roslaunch remote_robotics brain.launch
```

### Launching nodes by hand

Every ROS node outputs launch instructions when trying to launch with wrong/missing
parameters.

The used topics and services can be viewed [`rosservice`](http://wiki.ros.org/rosservice)
and [rostopic](http://wiki.ros.org/rostopic)

### Node overview {#nodes}

#### Image processing

##### remro_cam

Reads camera image and publishes image data.

    rosrun remro_vision remro_cam { <int video source> }

The video source parameter is optional. In case you ignore it 
the program trys '0'(corresponding to /dev/video0) as the image source.

**Publishing topics**
* /image/from_cam

##### img_view

Displays video transferred as `sensor_msgs::ConstImgPtr` or `remote_robotics::MotorImg`.

    rosrun remro_vision img_view <char* msg type> <char* subscriber topic>

##### binary_converter

Converts image to binary image and publishes said image.

    rosrun remro_vision binary_converter 

**Publishing topics**
* /image/binary
**Subscribed topics**
* /image/from_cam

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

#### Servo control

##### servo_hub

Provides service to set the angular position of the servo motors

    rosrun remote_robotics servo_hub <int device_number>

`device_number` corresponds to pololu device that controls the servos are connected to.

**Services**
* /set_abs_servo_motor

##### brain

Designed to be the "processor" that controls the servo motors and evaluates their positions.

    rosrun remote_robotics brain

## Launching two nodes of the same type

Trying to have two crop_nodes active at the same time (to monitor two servos at the same time) will result into an error message and the termiation of one of the launched nodes.
To launch the same node two times one has to change the node name when executing the node. In the aforementioned case:

    rosrun remote_robotics crop_node image/from_cam image/motor0 
    rosrun remote_robotics crop_node image/from_cam image/motor1 __name:=crop2

Changing the topic names of a node requires an argument with the pattern `<topic_name>:=<new_topic_name>`.
Changing the publisher topic of the binary_converter node would look like this:

    rosrun remote_robotics binary_converter image/from_cam image/binary:=image/binary2

Instead of trying to find the default topic names in the source code, one should try
`rostopic list` or use the graphical interface `rqt`.

See http://wiki.ros.org/Remapping%20Arguments for further information.

# 3. Communication between two machines

To setup a ROS-communication between two machines both 
IP addresses have to be known and set the Shell variables `ROS_MASTER_URI` and `ROS_IP`
to match the corresponding addresses.

It is necessary to disable the firewall on the machine running roscore. 
<!--- **both** machines. --> 

A verbose description of this process can be found at http://wiki.ros.org/ROS/Tutorials/MultipleMachines.
