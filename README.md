Torque-Control
==============

How to use the torque controller
--------------

For the torque controller, the youbot_oodl with enable torque messages and disable gripper sensor readouts has to be running. The torque controller can then be started with 

rosrun  torque_control  torque_control

if executed from the torque_control folder. In a launch file the following syntax has to be used:

<launch>

<node name="torque_control" pkg="torque_control" 
      type="torque_control" cwd="node" output="screen"/>

</launch>

This launches an action server without feedback. As the goal, a trajectory\_msgs/JointTrajectory needs to be defined. This can either be done manually or by calling the trajectory\_generator service. What is important is that the trajectory consists of values for joint positions, velocities and acceleration for all 5 arm joints. An example of how the torque\_controller is called can be found in the torque\_example.

How to use the trajectory generator
--------------

The trajectory_generator offers 4 different services:

- From_JS_2_JS: generates a joint space trajectory to move all the joints from the start joint space position to the desired end joint space position in the same amount of time.
- From_JS_2_CS: generates a joint space trajectory to move the gripper in a straight line from the start joint space position to the Cartesian end position.
- From_CS_2_CS: generates a joint space trajectory to move the gripper in a straight line from the Cartesian start to the Cartesian end position.
- Circular_Trajectory: generates a circular joint space trajectory around the center point.


If feasible trajectories for the input parameters are found, the trajectory is returned in a trajectory_msgs/JointTrajector message. "JS" in the service name stands for joint space. It expects a fully defined "brics\_actuator::JointPositions" message. The "CS" stands for Cartesian space. It is a "geometry\_msgs::Pose" message. Examples for how the different services are called, can be found in the trajectory\_generator/tester.cpp file.
