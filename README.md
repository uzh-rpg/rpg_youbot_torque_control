Torque-Control
==============

This repository provides a torque controller for the KUKA youBot arm as well as a Service to generate trajectories which can then be executed by the torque controller. The Torque Controller has been tested under the following setup:

* Ubuntu 12.04 with ROS-Hydro

Watch the [video](http://www.youtube.com/watch?v=8Ui3MoOxcPQ) demonstrating the RPG youBot Torque Controller:   
[![ RPG youBot Torque Controller Video](http://img.youtube.com/vi/OMZ1XVXErKY/0.jpg)](http://www.youtube.com/watch?v=OMZ1XVXErKY)

More information: [Master Thesis](http://rpg.ifi.uzh.ch/docs/theses/Benjamin_Keiser_Torque_Control_2013.pdf), [More software from the Robotics and Perception Group](http://rpg.ifi.uzh.ch/software_datasets.html)

Before You Use The Controller!
------------------------------

In order to use the torque controller, you will have to **change the controller gains of the motor controllers** of the KUKA youBot arm. Otherwise, the controller will **not** work. To change these gains, you can use the [joint configurator application](https://github.com/youbot/youbot_applications/tree/master/joint_configurator). Please read Section 4.3 of the [Master Thesis](http://rpg.ifi.uzh.ch/docs/theses/Benjamin_Keiser_Torque_Control_2013.pdf) to find out how to tune the gains. Our values are listed in Table 4.1--however, these values are most likely different for every robot. 


Installation
------------

### Dependencies

To compile everything you need to install the youBot driver.

    sudo apt-get install ros-hydro-youbot-driver

The Torque Controller makes use of the pr2_msgs and the brics_actuator packages. You can install them by running

    sudo apt-get install ros-hydro-pr2-msgs
    sudo apt-get install ros-hydro-brics-actuator
    
Additionally, you need an adapted version of the ros wrapper of the driver which you have to clone into the src folder of your catkin workspace

    cd catkin_ws/src
    git clone https://github.com/uzh-rpg/youbot_driver_ros_interface.git

The latter is an adapted version of the [mas-group/youbot_driver_ros_interface](https://github.com/mas-group/youbot_driver_ros_interface.git). **It is important to use the adapted package instead of the original one since the torque controller will not work properly otherwise!** The adapted version enables sending torque messages to the `youbot_oodl` and disables gripper sensor readouts. This was necessary because the gripper position readout is blocking the `youbot_oodl`.

### Main Installation

You can download the actual Torque Controller by running
    
    cd catkin_ws/src
    git clone https://github.com/uzh-rpg/rpg_youbot_torque_control.git
    
Then, you can simply `catkin_make` it

    cd catkin_ws
    catkin_make

### Test the Torque Controller with the Trajectory Generator

An axample on how to use the torque controller is provided in the [torque_example](https://github.com/uzh-rpg/rpg_youbot_torque_control/tree/master/torque_example) package. To test it, you first have to launch the youbot_ros_driver_interface by

    roslaunch youbot_driver_ros_interface youbot_driver.launch

Then, you can launch the provided launch file

    roslaunch torque_example torque_example.launch

which starts the Torque Controller and a Trajectory Generator Service. Then you can start the exapmple by running

    rosrun torque_example circle_traj
    
This will place the gripper to a start position using the existing position control. The node then asks you if you are ready to execute a trajectory with the torque controller. If you are type `yes` in the console where you started the `circle_traj` node. The gripper should then follow a circular trajectory.
    
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

* From_JS_2_JS ([trajectory_generator/JStoJS](https://github.com/uzh-rpg/rpg_youbot_torque_control/blob/master/trajectory_generator/srv/JStoJS.srv))

  Generates a joint space trajectory to move all the joints from the start joint space position to the desired end joint space position. The joints will perform a quadratic profile in position.
  
* From_JS_2_CS ([trajectory_generator/JStoCS](https://github.com/uzh-rpg/rpg_youbot_torque_control/blob/master/trajectory_generator/srv/JStoCS.srv))

  Generates a joint space trajectory to move the gripper in a straight line from the start joint space position to the Cartesian end position. This is helpful to reach a Cartesian end-effector position from the current manipulator configuration.
  
* From_CS_2_CS ([trajectory_generator/CStoCS](https://github.com/uzh-rpg/rpg_youbot_torque_control/blob/master/trajectory_generator/srv/CStoCS.srv))

  Generates a joint space trajectory to move the gripper in a straight line from the Cartesian start to the Cartesian end position.

* Circular_Trajectory ([trajectory_generator/Circle](https://github.com/uzh-rpg/rpg_youbot_torque_control/blob/master/trajectory_generator/srv/Circle.srv))

  Generates a circular joint space trajectory around a center point.

Examples on how to use the different services can be found in the [tester.cpp](https://github.com/uzh-rpg/rpg_youbot_torque_control/blob/master/trajectory_generator/src/tester.cpp) file.


