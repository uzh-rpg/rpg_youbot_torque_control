/*
// This file is part of RPG-YTC- the RPG youBot Torque Controller
//
// RPG-YTC is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// RPG-YTC is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with RPG-YTC.  If not, see <http://www.gnu.org/licenses/>.

 * ltr.cpp
 *
 *  Created on: Sep 11, 2013
 *      Author: keiserb
 */

#include "ros/ros.h"
#include "trajectory_generator/CStoCS.h"
#include "trajectory_generator/Circle.h"
#include "trajectory_msgs/JointTrajectoryPoint.h"
#include "Eigen/Dense"
#include <brics_actuator/JointPositions.h>
#include <brics_actuator/JointVelocities.h>
#include <geometry_msgs/Pose.h>
#include <iostream>
#include <boost/units/systems/si.hpp>
#include <torque_control/torque_trajectoryAction.h>
#include <torque_control/step.h>
#include <actionlib/client/simple_action_client.h>
#include "rpg_youbot_common/rpg_youbot_common.h"

using namespace std;
int main(int argc, char **argv)
{
  ros::init(argc, argv, "traj_tester");
  ros::NodeHandle nh;
  ros::Rate lr(10);
  ros::ServiceClient cs2cs_client = nh.serviceClient<trajectory_generator::CStoCS>("From_CS_to_CS");
  ros::Publisher arm_pub_pos = nh.advertise<brics_actuator::JointPositions>("/arm_1/arm_controller/position_command",
                                                                            1);
  actionlib::SimpleActionClient<torque_control::torque_trajectoryAction> ac("torque_control", true);

  double max_vel = 0.05;
  double max_acc = 0.5;
  ROS_INFO("max_vel: %f \t max_acc :%f", max_vel, max_acc);
  double first_pt[5];
  bool feasible = false;
  trajectory_msgs::JointTrajectory traj, temp;
  trajectory_msgs::JointTrajectoryPoint point;
  geometry_msgs::Pose start_p, low_p1, low_p2, end_p;
  trajectory_generator::CStoCS cs2cs;
  start_p.position.x = 0.12;
  start_p.position.y = 0.25;
  start_p.position.z = 0.00;
  Eigen::Quaterniond grip(0.6851, 0.1749, 0.6851, -0.1749);
  start_p.orientation.x = grip.x();
  start_p.orientation.y = grip.y();
  start_p.orientation.z = grip.z();
  start_p.orientation.w = grip.w();
  low_p1.position.x = 0.05;
  low_p1.position.y = 0.25;
  low_p1.position.z = -0.1;
  low_p1.orientation = start_p.orientation;
  low_p2.position.x = -0.05;
  low_p2.position.y = 0.25;
  low_p2.position.z = -0.1;
  low_p2.orientation = start_p.orientation;
  end_p.position.x = -0.12;
  end_p.position.y = 0.25;
  end_p.position.z = 0.00;
  end_p.orientation = start_p.orientation;

  cs2cs.request.start_pos = start_p;
  cs2cs.request.end_pos = low_p1;
  cs2cs.request.start_vel = 0.0;
  cs2cs.request.end_vel = max_vel;
  cs2cs.request.max_vel = max_vel;
  cs2cs.request.max_acc = max_acc;

  if (cs2cs_client.call(cs2cs))
  {
    if (cs2cs.response.feasible)
    {
      cout << "feasible" << endl;
      temp = cs2cs.response.trajectory;
      while (!temp.points.empty())
      {
        point = temp.points.back();
        temp.points.pop_back();
        traj.points.insert(traj.points.begin(), point);
      }
      traj.joint_names = temp.joint_names;
    }
    else
    {
      cout << "First Half Not Feasible" << endl;
      feasible = false;
    }
  }
  else
  {
    ROS_ERROR("Could not call service.");
    return 1;
  }

  cs2cs.request.start_pos = low_p1;
  cs2cs.request.end_pos = low_p2;
  cs2cs.request.start_vel = max_vel;
  cs2cs.request.end_vel = max_vel;
  cs2cs.request.max_vel = max_vel;
  cs2cs.request.max_acc = max_acc;

  if (cs2cs_client.call(cs2cs))
  {
    if (cs2cs.response.feasible)
    {
      cout << "feasible" << endl;
      temp = cs2cs.response.trajectory;
      while (!temp.points.empty())
      {
        point = temp.points.back();
        temp.points.pop_back();
        traj.points.insert(traj.points.begin(), point);
      }
      traj.joint_names = temp.joint_names;
    }
    else
    {
      cout << "First Half Not Feasible" << endl;
      feasible = false;
    }
  }
  else
  {
    ROS_ERROR("Could not call service.");
    return 1;
  }

  cs2cs.request.start_pos = low_p2;
  cs2cs.request.end_pos = end_p;
  cs2cs.request.start_vel = max_vel;
  cs2cs.request.end_vel = 0.0;
  cs2cs.request.max_vel = max_vel;
  cs2cs.request.max_acc = max_acc;

  if (cs2cs_client.call(cs2cs))
  {
    if (cs2cs.response.feasible)
    {
      cout << "feasible" << endl;
      temp = cs2cs.response.trajectory;
      while (!temp.points.empty())
      {
        point = temp.points.back();
        temp.points.pop_back();
        traj.points.insert(traj.points.begin(), point);
      }
      traj.joint_names = temp.joint_names;
    }
    else
    {
      cout << "Second Half Not Feasible" << endl;
      feasible = false;
    }
  }
  else
  {
    ROS_ERROR("Could not call service.");
    return 1;
  }

  if (feasible)
  {
    point = traj.points.back();
    int i = 0;
    while (!point.positions.empty())
    {
      first_pt[i] = point.positions.back();
      i++;
      point.positions.pop_back();
    }
    cout << "Publishing arm cmd" << endl;
    arm_pub_pos.publish(rpg_youbot_common::generateJointPositionMsg(first_pt));
    ros::spinOnce();
    sleep(0.5);
    arm_pub_pos.publish(rpg_youbot_common::generateJointPositionMsg(first_pt));
    ros::spinOnce();
    sleep(0.5);
    arm_pub_pos.publish(rpg_youbot_common::generateJointPositionMsg(first_pt));
    ros::spinOnce();
    sleep(1);

    ROS_INFO("READY FOR TORQUE?");
    int x;
    cin >> x;
    ac.waitForServer(); //will wait for infinite time
    ROS_INFO("Action server started, sending goal.");

    torque_control::torque_trajectoryGoal goal;
    goal.trajectory = traj;

    ac.sendGoal(goal);

    //wait for the action to return
    bool finished_before_timeout = ac.waitForResult(ros::Duration(60.0));

    if (finished_before_timeout)
    {
      actionlib::SimpleClientGoalState state = ac.getState();
      ROS_INFO("Action finished: %s", state.toString().c_str());
    }
    else
    {
      ROS_INFO("Action did not finish before the time out.");
    }
  }

  return 0;
}

