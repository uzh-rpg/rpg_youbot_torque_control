Torque-Control
==============

How to use the torque controller
--------------

For the torque controller, the `youbot_oodl` with enabled torque messages and disabled gripper sensor readouts has to be running. 
It can be downloaded from [https://github.com/ailab/youbot-ros-pkg](https://github.com/ailab/youbot-ros-pkg). The torque controller can then be started with 

    rosrun  torque_control  torque_control

if executed in the torque_control folder. In a launch file the following syntax has to be used:

    <launch>
  
    <node name="torque_control" pkg="torque_control" type="torque_control" cwd="node" output="screen"/>
  
    </launch>

This launches an action server. The action server goal msg must be of type `trajectory_msgs/JointTrajectory`. The message can either 
be created by your node or by calling the `trajectory_generator` service. The trajectory has to consist of values for joint 
positions, velocities and acceleration for all 5 arm joints. An example of how to call the `torque_controller` is found in 
the torque_example package.

How to use the trajectory generator
--------------

The `trajectory_generator` offers four different services:

- `From_JS_2_JS`: generates a joint space trajectory to move all the joints from the start joint space position to the desired end joint space position.
- `From_JS_2_CS`: generates a joint space trajectory to move the gripper in a straight line from the start joint space position to the Cartesian end position. This is helpful to reach a Cartesian end-effector position from the current manipulator configuration.
- `From_CS_2_CS`: generates a joint space trajectory to move the gripper in a straight line from the Cartesian start to the Cartesian end position.
- `Circular_Trajectory`: generates a circular joint space trajectory around a center point.

You can start the service with

    rosrun trajectory_generator trajectory_service

or

    <launch>

    <node name="trajectory" pkg="trajectory_generator" type="trajectory_service" output="screen" />

    </launch>

from a launch file. If a feasible trajectory for the input parameters are found, it is returned in a `trajectory_msgs/JointTrajectory`
message. "JS" in the service name stands for joint space. It expects a fully defined `brics_actuator::JointPositions` message. 
The "CS" stands for Cartesian space and is a `geometry_msgs::Pose` message. Examples for how the different services are called,
can be found in the `trajectory_generator/src/tester.cpp` file.
