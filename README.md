# Unoptimized dubins path

This package is used to apply dubins path trajectories to a robot. It takes as argument the different waypoint of the trajectory, the direction of rotation around these waypoints and publish the necessaries velocities to follow these trajectories. 



**Table of Contents**
---------------------

1. [Installation](#Installation)

2. [Dependencies](#Dependencies)

3. [Usage](#Usage)


<a name="Installation"/>
## Installation

To install it, clone the Dubins_path_test remote repository into your computer.

        git clone https://github.com/uf-reef-avl/dubins_path

<a name="Dependencies"/>
## Dependencies

This package depends mainly upon **reef_msgs** to retrieve the mocap position data from the robot. 

        git clone https://github.com/uf-reef-avl/reef_msgs
 
As this pack can be used either on quad or on turtlebot by remapping the topics, the user can also install the [kobuki packages](http://wiki.ros.org/kobuki/Tutorials/Installation)
 if he wants to use it directly on turtlebot. If he wants to use it on quad, this pack depends upon the [reef_teleop package](http://192.168.1.101/AVL-Summer-18/reef_teleop) or the [relative nav packages](http://192.168.1.101/AVL-Summer-18/relative_nav). 
 

 <a name="Usage"/>
## Usage

Dubins Path should be executed as a node using a ROS launchfile and paired with a rosvrpn or Gazebo node. Then the **rosvrpn position topic** and the **dubins path published velocity topic** should be remapped to match the dubins path's subscriber "pose" and the dubins path's publisher "velocity publisher". For example,
        
```xml
 <node pkg="ros_vrpn_client" name="vrpn_node" type="ros_vrpn_client" args="_vrpn_server_ip:=192.168.1.104" required="true" >
        <remap from="<!--robot mocap name-->/ned/pose" to="pose"/>
 </node>

 <node name="setpoint_publisher" pkg="dubins_path_test" type="get_setpoint_node.py" clear_params="true" output = "screen">
        <rosparam file="$(arg waypoints_file)" />
        <remap from="velocity_publisher" to="<!--turtlebot/quad command topic-->"/>
 </node>
```

The file ***basic.launch*** can also be used on turtlebot as an example. The robot name should be changed to match the mocap rigidbody's name. As this package is written in python, don't forget to compile also the file ***get_setpoint_node.py*** with the command **chmod +x get_setpoint_node.py**.



The basic idea of this package is to make the robot follow a moving point along a dubins path. A PD controller compute the velocity applied to the robot by using the difference between the robot's position/orientation and the moving point.
Because of this, the user has to specify some customs parameters in the yaml file named ***basic_waypoints.yaml*** in order to define: the trajectories, the velocity, the frame's type ... Example of this file and descriptions of these parameters are included:


```xml
#!!!!! Make sure all the value are float value !!!!!!!!!!!

#setpoint dictionary
# a setpoint dictionary consists of a setpoint position [x,y,z] in meters (NED frame),
# a yaw angle in degrees and a sens of rotation "clockwise" (clockwise = 0; counterclockwise = 1) around the setpoint
waypoint_list:
  [
    {position: [0., -2.,-1.], yaw_degrees: -90, clockwise: 1},
    {position: [0., 2.,-1.], yaw_degrees: -90, clockwise: 1}
    #{position: [0.8, 1. , 0], yaw_degrees: -90, clockwise: 0},
    #{position: [-0.8,-1., 0], yaw_degrees: -90, clockwise: 0}
  ]

# how close is "close enough" for the setpoint position
# imagine a circle around a setpoint with a radius equal to this value
# if the origin of the vehicle's coordinate frame enters this circle,
# the vehicle is considered to have hit the setpoint
# unit is meters
setpoint_radius_tolerance: 0.6

#this attribute varie from 0 to 1 and set the radius tolerance of the tangent point (entry and end) respectively to the setpoint radius size previously specified
proportional_tangent_point_radius_tolerance: 0.6

# fly to each setpoint in waypoint_list this many times
# e.g. if this value is set to 2, the vehicle will fly to each setpoint in
# waypoint_list and then do it again
number_of_cycles: 2

use_mocap: true

#Depending on the type of robot, the command can be sent in the NED (quad) or NWU (turtlebot) frame
#set this value to NED or NWU and make sure that the mocap PoseStamped message are set to NED
robot_command_orientation: NED

#if the frame is in NED, write a negative altitude
altitude: -1.0

#if this parameter is set to true, the dubins path won't be launched until this parameter is set to false. In the other hand, the dubin's path will be launched directly
starting_activation: false


#As the moving point is moving along the computed dubins path, the following parameters are used to control the velocity of this point and therefore the robot's velocity
#Set the distance in meters between each point of the dubins path trajectory.
distance_between_point: 0.05
#Set the rate update of the moving point in Hz. So if the distance between each point = 0.05 , and the update rate is 8.5 Hw then the maximum velocity of the robot will be 0.05*8.5 = 0.425 m/s
path_rate: 8.5
#the parameter below multiply the number of point on the circular trajectories around the setpoint in order to slow down the robot during them
circular_velocity_ratio: 1.5

#Specify the radius in meters of the robot's projected area
robot_radius: 0.25

#Maximale linear velocity possibly applied to the robot
VMax: 1.
#Minimale linear velocity applied to the robot
VMin: 0.1
#Maximale angular velocity possibly applied
PhiMax: 0.8

#Gains of the controller on x
Kx: 1.
#Gains of the controlller on y
Ky: 1.
#Gains of the controller on  theta
Ktheta: 1.

```

<a name="ROS Topics and Messages"/>
## ROS Topics and Messages

### Subscribed Topics
|Topic Name|Message Type|Description|
|--|--|--|
|pose|geometry_msgs::PoseStamped|Position of the robot|
|pose_stamped|geometry_msgs::TransformStamped|Position of the robot|

### Published Topics
|Topic Name|Message Type|Description|
|--|--|--|
|gains_publisher|geometry_msgs::Twist|Gains on x, y, yaw velocities|
|path_publisher|geometry_msgs::PoseStamped|Positions of the followed path|
|velocity_stamped_publisher|geometry_msgs::TwistStamped|Linear and angular velocities applied to the robot|
|velocity_publisher|geometry_msgs::Twist|Linear and angular velocities applied to the robot|
|quad_altitude_publisher|reef_teleop::AltitudeCommand|Altitude applied to the quad|
|quad_velocity_publisher|reef_teleop::VelocityCommand|Linear and angular velocities applied to the quad|
|quad_desired_state_publisher|relative_nav::VelocityCommand|Linear velocitie, angular velocitie and altitude applied to the quad|
