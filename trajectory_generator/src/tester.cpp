/*
 * tester.cpp
 *
 *  Created on: Sep 11, 2013
 *      Author: keiserb
 */

#include "ros/ros.h"
#include "trajectory_generator/JStoJS.h"
#include "trajectory_generator/JStoCS.h"
#include "trajectory_generator/CStoCS.h"
#include "trajectory_generator/Circle.h"
#include "trajectory_msgs/JointTrajectoryPoint.h"
#include "Eigen/Dense"
#include <brics_actuator/JointPositions.h>
#include <brics_actuator/JointVelocities.h>
#include <geometry_msgs/Pose.h>
#include <iostream>
#include "youbot_arm_model/youbot_joints.h"
#include <boost/units/systems/si.hpp>
#include <boost/units/io.hpp>

using namespace std;
int main(int argc, char **argv)
{
  ros::init(argc, argv, "traj_tester");
  ros::NodeHandle nh;
  ros::ServiceClient js2jstester = nh.serviceClient<trajectory_generator::JStoJS>("From_JS_to_JS");
  ros::ServiceClient js2cstester = nh.serviceClient<trajectory_generator::JStoCS>("From_JS_to_CS");
  ros::ServiceClient cs2cstester = nh.serviceClient<trajectory_generator::CStoCS>("From_CS_to_CS");
  ros::ServiceClient circletester = nh.serviceClient<trajectory_generator::Circle>("Circular_Trajectory");

  const std::string rad = boost::units::to_string(boost::units::si::radian);
  const std::string rad_s = boost::units::to_string(boost::units::si::radian_per_second);

  brics_actuator::JointPositions start;
  brics_actuator::JointPositions end;
  brics_actuator::JointVelocities start_v;
  brics_actuator::JointVelocities end_v;
  trajectory_msgs::JointTrajectoryPoint point;
  start.positions.resize(5);
  end.positions.resize(5);
  start_v.velocities.resize(5);
  end_v.velocities.resize(5);
  start.positions[0].value = 4.5204;
  start.positions[1].value = 1.8366;
  start.positions[2].value = -1.1556;
  start.positions[3].value = 2.8358;
  start.positions[4].value = 4.4942;
  end.positions[0].value = 4.5204;
  end.positions[1].value = 2.1819;
  end.positions[2].value = -1.1138;
  end.positions[3].value = 2.4487;
  end.positions[4].value = 4.4942;
  for (int i = 0; i < 5; i++)
  {
    start.positions[i].joint_uri = joint_names[i];
    end.positions[i].joint_uri = joint_names[i];
    start_v.velocities[i].joint_uri = joint_names[i];
    end_v.velocities[i].joint_uri = joint_names[i];
    start.positions[i].unit = rad;
    end.positions[i].unit = rad;
    start_v.velocities[i].unit = rad_s;
    end_v.velocities[i].unit = rad_s;
    start_v.velocities[i].value = 0;
    end_v.velocities[i].value = 0;
  }
  trajectory_generator::JStoJS js2js;
  js2js.request.start_pos = start;
  js2js.request.end_pos = end;
  js2js.request.start_vel = start_v;
  js2js.request.end_vel = end_v;
  js2js.request.max_vel = 0.05;
  js2js.request.max_acc = 0.5;
  /*  if (js2jstester.call(js2js))
   {
   if (js2js.response.feasible)
   {
   cout << "JStoJS feasible trajectory" << endl;
   while (!js2js.response.trajectory.joint_names.empty())
   {
   cout << js2js.response.trajectory.joint_names.back() << "\t";
   js2js.response.trajectory.joint_names.pop_back();
   }
   cout << endl;
   while (!js2js.response.trajectory.points.empty())
   {
   point = js2js.response.trajectory.points.back();
   js2js.response.trajectory.points.pop_back();
   cout << point.positions.back() << "\t";
   point.positions.pop_back();
   cout << point.positions.back() << "\t";
   point.positions.pop_back();
   cout << point.positions.back() << "\t";
   point.positions.pop_back();
   cout << point.positions.back() << "\t";
   point.positions.pop_back();
   cout << point.positions.back() << endl;
   }
   }
   else
   {
   cout << "JStoJS non feasible trajectory" << endl;
   }
   }
   */
  geometry_msgs::Pose object;
  trajectory_generator::CStoCS cs2cs;
  trajectory_generator::JStoCS js2cs;
  object.position.x = 0.0;
  object.position.y = -0.25;
  object.position.z = -0.02;
  Eigen::Quaterniond grip(0.6851, -0.1749, 0.6851, 0.1749);
  object.orientation.x = grip.x();
  object.orientation.y = grip.y();
  object.orientation.z = grip.z();
  object.orientation.w = grip.w();
  cs2cs.request.start_pos = object;
  object.position.z = -0.1;
  cs2cs.request.end_pos = object;
  cs2cs.request.start_vel = 0.0;
  cs2cs.request.end_vel = 0.0;
  cs2cs.request.max_vel = 0.05;
  cs2cs.request.max_acc = 0.5;

  js2cs.request.start_pos = start;
  js2cs.request.end_pos = object;
  js2cs.request.start_vel = 0.0;
  js2cs.request.end_vel = 0.0;
  js2cs.request.max_vel = 0.05;
  js2cs.request.max_acc = 0.5;

  /*  if (js2cstester.call(js2cs))
   {
   if (js2cs.response.feasible)
   {
   cout << "JStoCS feasible trajectory" << endl;
   while (!js2cs.response.trajectory.joint_names.empty())
   {
   cout << js2cs.response.trajectory.joint_names.back() << "\t";
   js2cs.response.trajectory.joint_names.pop_back();
   }
   cout << endl;
   while (!js2cs.response.trajectory.points.empty())
   {
   point = js2cs.response.trajectory.points.back();
   js2cs.response.trajectory.points.pop_back();
   cout << point.positions.back() << "\t";
   point.positions.pop_back();
   cout << point.positions.back() << "\t";
   point.positions.pop_back();
   cout << point.positions.back() << "\t";
   point.positions.pop_back();
   cout << point.positions.back() << "\t";
   point.positions.pop_back();
   cout << point.positions.back() << endl;
   }
   }
   else
   {
   cout << "JS 2 CS not feasible" << endl;
   }
   }
   */
  /*  if (cs2cstester.call(cs2cs))
   {
   if (cs2cs.response.feasible)
   {
   cout << "CStoCS feasible trajectory" << endl;
   while (!cs2cs.response.trajectory.joint_names.empty())
   {
   cout << cs2cs.response.trajectory.joint_names.back() << "\t";
   cs2cs.response.trajectory.joint_names.pop_back();
   }
   cout << endl;
   while (!cs2cs.response.trajectory.points.empty())
   {
   point = cs2cs.response.trajectory.points.back();
   cs2cs.response.trajectory.points.pop_back();
   cout << point.positions.back() << "\t";
   point.positions.pop_back();
   cout << point.positions.back() << "\t";
   point.positions.pop_back();
   cout << point.positions.back() << "\t";
   point.positions.pop_back();
   cout << point.positions.back() << "\t";
   point.positions.pop_back();
   cout << point.positions.back() << endl;
   }
   }
   else
   {
   cout << "CS 2 CS not feasible" << endl;
   }

   }
   */
  trajectory_generator::Circle circle;
  circle.request.radius = 0.1;
  circle.request.omega = 2 * circle.request.radius * M_PI;
  /* circle.request.rpy[0] = M_PI/2;
   circle.request.rpy[1] = 0;
   circle.request.rpy[2] = 0;
   circle.request.center[0] = 0.35;
   circle.request.center[1] = 0;
   circle.request.center[2] = 0.3;*/
  circle.request.rpy[0] = M_PI / 2;
  circle.request.rpy[1] = -M_PI / 4;
  circle.request.rpy[2] = M_PI / 10;
  circle.request.center[0] = 0.35;
  circle.request.center[1] = 0.0;
  circle.request.center[2] = 0.5;

  if (circletester.call(circle))
  {
    if (circle.response.feasible)
    {
      cout << "Circle feasible trajectory" << endl;
      while (!circle.response.trajectory.joint_names.empty())
      {
        cout << circle.response.trajectory.joint_names.back() << "\t";
        circle.response.trajectory.joint_names.pop_back();
      }
      cout << endl;
      while (!circle.response.trajectory.points.empty())
      {
        point = circle.response.trajectory.points.back();
        circle.response.trajectory.points.pop_back();
        cout << point.positions.back() << "\t";
        point.positions.pop_back();
        cout << point.positions.back() << "\t";
        point.positions.pop_back();
        cout << point.positions.back() << "\t";
        point.positions.pop_back();
        cout << point.positions.back() << "\t";
        point.positions.pop_back();
        cout << point.positions.back() << endl;
      }
    }
    else
    {
      cout << "CS 2 CS not feasible" << endl;
    }

  }
  return 0;
}
