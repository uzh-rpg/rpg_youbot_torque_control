Torque-Control
==============

Video: [http://www.youtube.com/watch?v=OMZ1XVXErKY](http://www.youtube.com/watch?v=OMZ1XVXErKY)  
More information: [Master Thesis](http://rpg.ifi.uzh.ch/docs/theses/Benjamin_Keiser_Torque_Control_2013.pdf), [More software from the Robotics and Perception Group](http://rpg.ifi.uzh.ch/software_datasets.html)

This repository provides a torque controller for the KUKA youBot arm as well as a Service to generate trajectories which can then be executed by the torque controller. The Torque Controller has been tested under ROS-Groovy and Ubuntu 12.04.

Installation
------------

### Dependencies

The Torque Controller makes use of the trajectory_msgs of the pr2_controllers package. You can install it by running

    sudo apt-get install ros-groovy-pr2-controllers
    
Additionally, the following packages are required which you can download by running

    git clone https://github.com/ipa320/cob_common.git
    git clone https://github.com/ailab/youbot-ros-pkg.git

The latter is an adapted version of the [youbot/youbot-ros-pkg](https://github.com/youbot/youbot-ros-pkg). The adapted version enables sending torque messages to the `youbot_oodl` and disables gripper sensor readouts. This was necessary because the gripper position readout is blocking the `youbot_oodl`.

### Main Installation

You can download the actual Torque Controller by running

    git clone https://github.com/ailab/rpg_youbot_torque_control.git
    
Then, you can simply `rosmake` it

    cd rpg_youbot_torque_control/torque_control/
    rosmake

Basic Usage
-----------

### Torque Controller

The Torque Controller node is organized as an Action Server and can be launched as follows:

    <launch>
  
    <node name="torque_control" pkg="torque_control" type="torque_control" cwd="node" output="screen"/>
  
    </launch>
    
#### Subscribed Topics

* joint_states ([sensor_msgs::JointState](http://docs.ros.org/api/sensor_msgs/html/msg/JointState.html))

  The joint state message as published by the `youbot_oodl`.

#### Published Topics

* arm_1/arm_controller/position_command ([brics_actuator/JointPositions](https://github.com/ipa320/cob_common/blob/groovy_dev/brics_actuator/msg/JointPositions.msg))

  The joint positions commands to be sent to the `youbot_oodl`.

* arm_1/arm_controller/torques_command ([brics_actuator/JointTorques](https://github.com/ipa320/cob_common/blob/groovy_dev/brics_actuator/msg/JointTorques.msg))

  The joint torque commands to be sent to the `youbot_oodl`.

#### Action Subscribed Topics

* torque_control/goal ([trajectory_msgs/JointTrajectory](http://docs.ros.org/api/trajectory_msgs/html/msg/JointTrajectory.html))
* 
  This message is created by calling the `trajectory_generator` service. The trajectory must consist of values for joint positions, velocities and acceleration for all 5 arm joints.

* torque_control/cancel ([actionlib_msgs/GoalID](http://docs.ros.org/api/actionlib_msgs/html/msg/GoalID.html))

  Cancels an action with a specific ID.

#### Action Subscribed Topics

* torque_control/feedback (torque_control/torque_trajectoryActionFeedback)
* torque_control/status ([actionlib_msgs/GoalStatusArray](http://docs.ros.org/api/actionlib_msgs/html/msg/GoalStatusArray.html))
* torque_control/result (torque_control/torque_trajectoryActionResult)

#### Services

* turn_gravity_compensation_on
* turn_gravity_compensation_off

### Trajectory Generator

The trajectory generator node can be launched as follows:

    <launch>

    <node name="trajectory" pkg="trajectory_generator" type="trajectory_service" output="screen" />

    </launch>
    
I provides four different services to generate trajectories for the KUKA youBot arm. In the following, `JS` denotes Joint Space and `CS` denotes the Cartesion Space. If a feasible trajectory for the input parameters are found, it is returned in a `trajectory_msgs/JointTrajectory` message.

#### Services

* From_JS_2_JS ([trajectory_generator/JStoJS](https://github.com/ailab/rpg_youbot_torque_control/blob/master/trajectory_generator/srv/JStoJS.srv))

  Generates a joint space trajectory to move all the joints from the start joint space position to the desired end joint space position. The joints will perform a quadratic profile in position.
  
* From_JS_2_CS ([trajectory_generator/JStoCS](https://github.com/ailab/rpg_youbot_torque_control/blob/master/trajectory_generator/srv/JStoCS.srv))

  Generates a joint space trajectory to move the gripper in a straight line from the start joint space position to the Cartesian end position. This is helpful to reach a Cartesian end-effector position from the current manipulator configuration.
  
* From_CS_2_CS ([trajectory_generator/CStoCS](https://github.com/ailab/rpg_youbot_torque_control/blob/master/trajectory_generator/srv/CStoCS.srv))

  Generates a joint space trajectory to move the gripper in a straight line from the Cartesian start to the Cartesian end position.

* Circular_Trajectory ([trajectory_generator/Circle](https://github.com/ailab/rpg_youbot_torque_control/blob/master/trajectory_generator/srv/Circle.srv))

  Generates a circular joint space trajectory around a center point.

Examples on how to use the different services can be found in the [tester.cpp](https://github.com/ailab/rpg_youbot_torque_control/blob/master/trajectory_generator/src/tester.cpp) file.

### Example for using the Torque Controller with the Trajectory Generator

An axample on how to use the torque controller is provided in the [torque_example](https://github.com/ailab/rpg_youbot_torque_control/tree/master/torque_example) package. Make it as

    rosmake torque_example

Then, you can launch the provided launch file

    roslaunch torque_example torque_example.launch

which starts the Torque Controller and a Trajectory Generator Service. Then you can start the exapmple by running

    rosrun torque_example gwd_traj

