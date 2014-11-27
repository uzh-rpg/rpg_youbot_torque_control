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

 * trajtory_server.cpp
 *
 *  Created on: Sep 11, 2013
 *      Author: keiserb
 */

#include "trajectory_generator/trajectory_generator.h"
#include "trajectory_generator/JStoJS.h"
#include "trajectory_generator/JStoCS.h"
#include "trajectory_generator/CStoCS.h"
#include "trajectory_generator/Circle.h"
#include <iostream>

namespace trajectory_generator {

using namespace std;

double lr; //oodl loop rate
ros::NodeHandle * nh;

bool JStoJS(trajectory_generator::JStoJS::Request &req, trajectory_generator::JStoJS::Response &res)
{
  Eigen::VectorXd s_pos(5), e_pos(5), s_vel(5), e_vel(5);
  double max_vel, max_acc;
  max_vel = req.max_vel;
  max_acc = req.max_acc;
  if (max_vel <= 0 || max_vel > 0.2)
  {
    ROS_ERROR("INVALID MAX_VEL %f, VALUE HAS TO BE BETWEEN 0 AND 0.2", max_vel);
    res.feasible = false;
    return false;
  }
  if (max_acc <= 0 || max_acc > 1)
  {
    ROS_ERROR("INVALID MAX_ACC %f, VALUE HAS TO BE BETWEEN 0 AND 1", max_acc);
    res.feasible = false;
    return false;
  }
  TrajectoryGenerator * traj = new TrajectoryGenerator(*nh, max_vel, max_acc, 1.0 / lr);
  if (!traj->bricsPosToEigen(req.start_pos, s_pos))
  {
    res.feasible = false;
    return false;
  }
  if (!traj->bricsPosToEigen(req.end_pos, e_pos))
  {
    res.feasible = false;
    return false;
  }
  if (!traj->bricsVelToEigen(req.start_vel, s_vel))
  {
    res.feasible = false;
    return false;
  }
  if (!traj->bricsVelToEigen(req.end_vel, e_vel))
  {
    res.feasible = false;
    return false;
  }
  traj->getTrajectory(s_pos, e_pos, s_vel, e_vel);
  traj->genToTrajectory(traj, res.trajectory);
  res.feasible = !res.trajectory.points.empty();
  delete traj;
  return true;
}

bool JStoCS(trajectory_generator::JStoCS::Request &req, trajectory_generator::JStoCS::Response &res)
{
  Eigen::VectorXd start(5);
  Eigen::Affine3d p_start, p_end;
  double start_v = req.start_vel;
  double end_v = req.end_vel;
  double max_vel, max_acc;
  max_vel = req.max_vel;
  max_acc = req.max_acc;
  if (max_vel <= 0 || max_vel > 0.2)
  {
    ROS_ERROR("INVALID MAX_VEL %f, VALUE HAS TO BE BETWEEN 0 AND 0.2", max_vel);
    res.feasible = false;
    return false;
  }
  if (max_acc <= 0 || max_acc > 1)
  {
    ROS_ERROR("INVALID MAX_ACC %f, VALUE HAS TO BE BETWEEN 0 AND 1", max_acc);
    res.feasible = false;
    return false;
  }
  TrajectoryGenerator * traj = new TrajectoryGenerator(*nh, max_vel, max_acc, 1.0 / lr);
  if (!traj->bricsPosToEigen(req.start_pos, start))
  {
    res.feasible = false;
    return false;
  }
  start = youbot2matlab(start);
  getCartPos(start, p_start);
  traj->smoothAffine(p_start);
  traj->poseToEigen(req.end_pos, p_end);
  traj->compareRotation(p_start, p_end);
  traj->getTrajectory(p_start, p_end, start_v, end_v);
  traj->genToTrajectory(traj, res.trajectory);
  res.feasible = !res.trajectory.points.empty();
  delete traj;
  return true;
}

bool CStoCS(trajectory_generator::CStoCS::Request &req, trajectory_generator::CStoCS::Response &res)
{
  Eigen::Affine3d p_start, p_end;
  double start_v = req.start_vel;
  double end_v = req.end_vel;
  double max_vel, max_acc;
  max_vel = req.max_vel;
  max_acc = req.max_acc;
  if (max_vel <= 0 || max_vel > 0.2)
  {
    ROS_ERROR("INVALID MAX_VEL %f, VALUE HAS TO BE BETWEEN 0 AND 0.2", max_vel);
    res.feasible = false;
    return false;
  }
  if (max_acc <= 0 || max_acc > 1)
  {
    ROS_ERROR("INVALID MAX_ACC %f, VALUE HAS TO BE BETWEEN 0 AND 1", max_acc);
    res.feasible = false;
    return false;
  }
  TrajectoryGenerator * traj = new TrajectoryGenerator(*nh, max_vel, max_acc, 1.0 / lr);
  traj->poseToEigen(req.start_pos, p_start);
  traj->poseToEigen(req.end_pos, p_end);
  traj->getTrajectory(p_start, p_end, start_v, end_v);
  traj->genToTrajectory(traj, res.trajectory);
  res.feasible = !res.trajectory.points.empty();
  delete traj;
  return true;
}

bool Circle(trajectory_generator::Circle::Request &req, trajectory_generator::Circle::Response &res)
{
  Eigen::Vector3d rpy, center;
  for (int i = 0; i < 3; i++)
  {
    rpy(i) = req.rpy[i];
    center(i) = req.center[i];
  }
  TrajectoryGenerator * traj = new TrajectoryGenerator(*nh, 0.0, 0.0, 1.0 / lr);
  traj->getCircle(req.radius, req.omega, rpy, center);
  traj->genToTrajectory(traj, res.trajectory);
  res.feasible = !res.trajectory.points.empty();
  delete traj;
  return true;
}

} // namespace trajectory_generator

using namespace trajectory_generator;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "Trajectory_Generator");
  nh = new ros::NodeHandle;
  nh->param("youBotDriverCycleFrequencyInHz", lr, 50.0);
  ros::ServiceServer srv_js_js = nh->advertiseService("From_JS_to_JS", JStoJS);
  ros::ServiceServer srv_js_cs = nh->advertiseService("From_JS_to_CS", JStoCS);
  ros::ServiceServer srv_cs_cs = nh->advertiseService("From_CS_to_CS", CStoCS);
  ros::ServiceServer srv_circle = nh->advertiseService("Circular_Trajectory", Circle);
  ROS_INFO("Ready to process trajectory requests");
  ros::spin();

  return 0;
}
