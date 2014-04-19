/*
 * trajectory_generator.h
 *
 *  Created on: May 2, 2013
 *      Author: keiserb
 */

#ifndef TRAJ_HPP_
#define TRAJ_HPP_

#include <iostream>

#include <ros/ros.h>

#include <math.h>

#include "youbot_arm_model/youbot_joints.h"
#include "youbot_arm_model/youbot_arm_forward_kinematics.h"
#include "youbot_arm_model/youbot_arm_dynamics_symbolic.h"

#include "ik_solver_service/SolvePreferredPitchIK.h"
#include "ik_solver_service/SolveFullyConstrainedIKArray.h"
#include "ik_solver_service/FullyConstrainedReq.h"
#include "ik_solver_service/FullyConstrainedRes.h"

#include <brics_actuator/JointPositions.h>
#include <brics_actuator/JointVelocities.h>
#include <trajectory_msgs/JointTrajectory.h>

#include <tf_conversions/tf_eigen.h>

#include <Eigen/Dense>

#include <boost/units/systems/si.hpp>
#include <boost/units/io.hpp>

namespace trajectory_generator {

class TrajectoryGenerator
{
public:
  //Constructor
  TrajectoryGenerator(ros::NodeHandle& nh, double maxvel, double maxacc, double tstep);
  ~TrajectoryGenerator();

  //define functions
  bool initialize(double eps, unsigned int max_iter);
  bool bricsPosToEigen(const brics_actuator::JointPositions joint_pos, Eigen::VectorXd & out);
  bool bricsVelToEigen(const brics_actuator::JointVelocities joint_vel, Eigen::VectorXd & out);
  void smoothAffine(Eigen::Affine3d & out);
  void compareRotation(Eigen::Affine3d & start, Eigen::Affine3d & end);
  void poseToEigen(const geometry_msgs::Pose pose, Eigen::Affine3d & out);
  void genToTrajectory(TrajectoryGenerator * gen, trajectory_msgs::JointTrajectory &trajectory);
  TrajectoryGenerator* getTrajectory(Eigen::VectorXd & s_pos, Eigen::VectorXd & e_pos, Eigen::VectorXd & s_vel,
                                     Eigen::VectorXd & e_vel);
  TrajectoryGenerator* getTrajectory(Eigen::Affine3d & s_pos, Eigen::Affine3d & e_pos, double s_vel, double e_vel);
  TrajectoryGenerator* getCircle(double radius, double omega, Eigen::Vector3d rpy, Eigen::Vector3d center);

private:
  int generateEndPosition(const Eigen::VectorXd& q_in, const Eigen::VectorXd& p_in, const Eigen::VectorXd& startvel,
                          const Eigen::VectorXd& endvel);
  int calculateMaxVelsJS();
  int calculateMaxVelsCS(Eigen::Affine3d& p_start, Eigen::Affine3d& p_end, double start_v, double end_v);
  int generateTrajectoriesJS();
  int generateTrajectoriesCS(Eigen::Affine3d& p_start, Eigen::Affine3d& p_end, double start_v, double end_v);
  int transformTrajectoryCStoJS();
  int generateTorques();
  bool checkJointLimits(Eigen::VectorXd & pos);
  void targetPositionCallbackJS(const brics_actuator::JointPositions::ConstPtr& msg);
  void targetPositionCallbackCS(const geometry_msgs::Pose::ConstPtr& msg);
  void rotToRPY(Eigen::Matrix3d & rot, Eigen::Vector3d & rpy);
  //define variables
  int DOF;
  double max_vel, max_acc, max_dec, t_step, accuracy, time, t_const, t_acc, t_dec;
  Eigen::Vector3d rpy_v;
  Eigen::VectorXd start_pos, end_pos, direction, start_vel, end_vel, max_v, max_a, max_d, m_qdotdot;
  std::vector<Eigen::VectorXd> q_tra, qdot_tra, qdotdot_tra;
  std::vector<Eigen::Vector3d> pos_tra, vel_tra, acc_tra, rpy_tra;
  ros::ServiceClient solve_fully_constrained_ik_array_client;
  std::string prop_urdf_model;
  brics_actuator::JointPositions m_joint_pos;
};

} // namespace trajectory_generator

#endif /* TRAJ_HPP_ */
