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

#include "trajectory_generator/trajectory_generator.h"
#include <iostream>

namespace trajectory_generator {

using namespace std;
/**
 * Constructs a TrajectoryGenerator class object. Requires the desired max
 * velocity and acceleration as well as the rate at which the loop operates.
 */
TrajectoryGenerator::TrajectoryGenerator(ros::NodeHandle& nh, double maxvel, double maxacc, double tstep) :
    DOF(5), accuracy(0), time(0), t_const(0), t_acc(0), t_dec(0)
{
  max_vel = maxvel;
  max_acc = maxacc;
  max_dec = maxacc;
  t_step = tstep;
  solve_fully_constrained_ik_array_client = nh.serviceClient<ik_solver_service::SolveFullyConstrainedIKArray>(
      "solve_fully_constrained_ik_array");
}

TrajectoryGenerator::~TrajectoryGenerator()
{
}
/**
 * initializes the TrajectoryGenerator object. Resizes all the JointArrays and creates the KDL solvers
 */
bool TrajectoryGenerator::initialize(double eps, unsigned int max_iter)
{
  end_pos.setZero(DOF);
  start_pos.setZero(DOF);
  direction.setZero(DOF);
  start_vel.setZero(DOF);
  end_vel.setZero(DOF);
  m_qdotdot.setZero(DOF);
  max_v.setZero(DOF);
  max_a.setZero(DOF);
  max_d.setZero(DOF);

  rpy_v = rpy_v.Zero();

  accuracy = eps;
  return true;
}

TrajectoryGenerator* TrajectoryGenerator::getTrajectory(Eigen::Affine3d & s_pos, Eigen::Affine3d & e_pos, double s_vel,
                                                        double e_vel)
{
  cout << "start cartesion position" << endl;
  cout << s_pos.matrix();
  cout << endl;
  cout << "Trajectory Generator created" << endl;

  if (!initialize(0.001, 1000))
  {
    ROS_WARN("Trajectory Generator initialize failed");
  }
  else
  {
    generateTrajectoriesCS(s_pos, e_pos, s_vel, e_vel);
    transformTrajectoryCStoJS();
    ROS_INFO("Trajectory Generator trajectories generated");
  }
  return this;
}

TrajectoryGenerator* TrajectoryGenerator::getTrajectory(Eigen::VectorXd & s_pos, Eigen::VectorXd & e_pos,
                                                        Eigen::VectorXd & s_vel, Eigen::VectorXd & e_vel)
{
  if (!initialize(0.001, 1000))
  {
    ROS_WARN("Trajectory Generator initialize failed");
  }
  else if (generateEndPosition(s_pos, e_pos, s_vel, e_vel) < 0)
  {
    ROS_WARN("Trajectory Generator end position generation failed");
  }
  else
  {
    ROS_INFO("End Position Generated");
    calculateMaxVelsJS();
    ROS_INFO("Trajectory Generator max vels calculated");
    generateTrajectoriesJS();
    ROS_INFO("Trajectory Generator trajectories generated");
  }
  return this;
}

/**
 * Generates the end positions for the joints for a given start position and the desired end effector position in
 * joint space. Also sets the start and end velocities to the desired values.
 */
int TrajectoryGenerator::generateEndPosition(const Eigen::VectorXd& q_in, const Eigen::VectorXd& p_in,
                                             const Eigen::VectorXd& startvel, const Eigen::VectorXd& endvel)
{
  cout << "Start Pos: \t";
  for (int i = 0; i < DOF; i++)
  {
    cout << q_in(i) << "\t";
  }
  cout << endl;
  cout << "End Pos: \t";
  for (int i = 0; i < DOF; i++)
  {
    cout << p_in(i) << "\t";
  }
  cout << endl;
  cout << "Start Vel: \t";
  for (int i = 0; i < DOF; i++)
  {
    cout << startvel(i) << "\t";
  }
  cout << endl;
  cout << "End Vel: \t";
  for (int i = 0; i < DOF; i++)
  {
    cout << endvel(i) << "\t";
  }
  cout << endl;
  cout << "max vel: \t" << 10 * max_vel << "\t max acc: \t" << 10 * max_acc << endl;
  start_pos = q_in;
  start_vel = startvel;
  end_vel = endvel;
  end_pos = p_in;
  for (int i = 0; i < DOF; i++)
  {
    if (q_in(i) < end_pos(i))
      direction(i) = 1;
    else
      direction(i) = -1;
  }
  return 0;
}
/**
 * Calculates the time needed for the trajectory as well as the actual max velocities and accelerations for each joint.
 * All times are discrete with time step according to the loop rate.
 */
int TrajectoryGenerator::calculateMaxVelsJS()
{
  Eigen::VectorXd qdiff;
  qdiff.setZero(DOF);
  qdiff = end_pos - start_pos;
  int index = 0;
  double maxdiff = qdiff(0);
  double act_diff = 0;
  double act_vel = 0;
  double old_vel = 0;
  double T = 0;
  for (int i = 1; i < DOF; i++)
  {
    if (fabs((double)qdiff(i)) > fabs(maxdiff))
    {
      maxdiff = qdiff(i);
      index = i;
    }
  }
  act_vel = start_vel(index);
  while (act_diff < fabs(maxdiff))
  {
    while (direction(index) * act_vel < 10 * max_vel && act_diff < fabs(maxdiff / 2))
    {
      old_vel = act_vel;
      act_vel += direction(index) * t_step * 10 * max_acc;
      act_diff += fabs(t_step * (act_vel + old_vel) / 2);
      T += t_step;
    }
    t_acc = T;
    T = 0;
    while (direction(index) * act_vel > end_vel(index))
    {
      old_vel = act_vel;
      act_vel -= direction(index) * t_step * 10 * max_dec;
      act_diff += fabs(t_step * (act_vel + old_vel) / 2);
      T += t_step;
    }
    t_dec = T;
    T = 0;
    while (act_diff <= fabs(maxdiff))
    {
      act_diff += fabs(t_step * 10 * max_vel);
      T += t_step;
    }
    t_const = T;
    time = t_const + t_acc + t_dec;
  }
  cout << "t_acc: \t" << t_acc << "\t t_dec: \t" << t_dec << "\t t_c: \t" << t_const << endl;
  for (int i = 0; i < DOF; i++)
  {
    max_v(i) = fabs(
        (double)(qdiff(i) - 0.5 * t_acc * start_vel(i) - 0.5 * t_dec * end_vel(i))
            / (0.5 * t_acc + t_const + 0.5 * t_dec));
    if (t_acc != 0)
    {
      max_a(i) = fabs((double)(direction(i) * max_v(i) - start_vel(i)) / t_acc);
    }
    else
    {
      max_a(i) = 0;
    }
    if (t_dec != 0)
    {
      max_d(i) = fabs((double)(direction(i) * max_v(i) - end_vel(i)) / t_dec);
    }
    else
    {
      max_d(i) = 0;
    }
  }
  cout << "Max Vels: \t";
  for (int z = 0; z < DOF; z++)
  {
    cout << max_v(z) << "\t";
  }
  cout << endl;
  return 0;
}

/**
 * Calculates the time needed for the trajectory as well as the actual max velocities and accelerations for the trajectory in cartesian space.
 * All times are discrete with time step according to the loop rate.
 */
int TrajectoryGenerator::calculateMaxVelsCS(Eigen::Affine3d& p_start, Eigen::Affine3d& p_end, double start_v,
                                            double end_v)
{
  //cout << "Calculating Max Vel" << endl;
  double T;
  Eigen::Vector3d q_diff, act_diff, old_vel, act_vel, tar_vel, q_norm, rpy_0, rpy_t, rpy_diff, temp;
  Eigen::Matrix3d rot_s, rot_e;
  rot_s = p_start.rotation();
  rot_e = p_end.rotation();
  rotToRPY(rot_s, rpy_0);
  rotToRPY(rot_e, rpy_t);
  cout << "rpy_0: " << rpy_0.matrix() << endl;
  cout << "rpy_t: " << rpy_t.matrix() << endl;
  q_diff = p_end.translation() - p_start.translation();
  //cout << "q_diff: \t" << q_diff[0] << "\t" << q_diff[1] << "\t" << q_diff[2] << endl;
  rpy_diff = rpy_t - rpy_0;
  act_diff = act_diff.Zero();
  q_norm = q_diff;
  q_norm.normalize();
  //cout << "q_norm: \t" << q_norm[0] << "\t" << q_norm[1] << "\t" << q_norm[2] << endl;
  act_vel = start_v * q_norm;
  tar_vel = end_v * q_norm;
  T = 0;
  while (act_diff.norm() < q_diff.norm())
  {
    while (act_vel.norm() < max_vel && fabs((double)act_vel.norm() - max_vel) > DBL_EPSILON
        && act_diff.norm() < q_diff.norm() / 2)
    {
      old_vel = act_vel;
      act_vel += t_step * q_norm * max_acc;
      act_diff += t_step * 0.5 * (act_vel + old_vel);
      T += t_step;
      //cout << "act_diff: \t" << act_diff[0] <<"\t" << act_diff[1] <<"\t" << act_diff[2] << "\t" << act_vel.Norm()<<endl;
    }
    t_acc = T;
    T = 0;
    cout << "t_acc: \t" << t_acc << endl;
    while (fabs((double)act_vel.norm() - (double)tar_vel.norm()) > accuracy)
    {
      old_vel = act_vel;
      act_vel -= t_step * q_norm * max_dec;
      act_diff += t_step * 0.5 * (act_vel + old_vel);
      //cout << "act_diff: \t" << act_diff[0] <<"\t" << act_diff[1] <<"\t" << act_diff[2] << endl;
      T += t_step;
    }
    t_dec = T;
    T = 0;
    cout << "t_dec: \t" << t_dec << endl;
    while (act_diff.norm() < q_diff.norm())
    {
      act_diff += t_step * max_vel * q_norm;
      T += t_step;
      //cout << "act_diff: \t" << act_diff[0] <<"\t" << act_diff[1] <<"\t" << act_diff[2] << endl;
    }
    t_const = T;
    cout << "t_const: \t" << t_const << endl;
    time = t_acc + t_const + t_dec;
    T = 0;
  }
  temp = (q_diff - 0.5 * t_acc * start_v * q_norm - 0.5 * t_dec * end_v * q_norm)
      / (0.5 * t_acc + t_const + 0.5 * t_dec);
  max_vel = temp.norm();
  while (start_v > max_vel || end_v > max_vel)
  {
    if (start_v > max_vel)
    {
      start_v = max_vel;
      temp = (q_diff - 0.5 * t_acc * start_v * q_norm - 0.5 * t_dec * end_v * q_norm)
          / (0.5 * t_acc + t_const + 0.5 * t_dec);
      max_vel = temp.norm();
    }
    if (end_v > max_vel)
    {
      end_v = max_vel;
      temp = (q_diff - 0.5 * t_acc * start_v * q_norm - 0.5 * t_dec * end_v * q_norm)
          / (0.5 * t_acc + t_const + 0.5 * t_dec);
      max_vel = temp.norm();
    }
  }
  if (t_acc != 0)
  {
    max_acc = (max_vel - start_v) / t_acc;
  }
  else
  {
    max_acc = 0;
  }
  if (t_dec != 0)
  {
    max_dec = (max_vel - end_v) / t_dec;
  }
  else
  {
    max_dec = 0;
  }
  cout << "max_vel: \t" << max_vel << "\t max_acc: \t" << max_acc << "\t max_dec: \t" << max_dec << endl;
  if (rpy_diff(0) > 0.01 || rpy_diff(0) > 0.01 || rpy_diff(0) > 0.01)
  {
    rpy_v = rpy_diff / (t_acc + t_const + t_dec);
  }
  else
  {
    rpy_v.setZero(3);
  }
  cout << "rpy_diff: \t" << (double)rpy_diff(0) << "\t" << (double)rpy_diff(1) << "\t" << (double)rpy_diff(2) << endl;
  cout << "rpy_v: \t" << (double)rpy_v(0) << "\t" << (double)rpy_v(1) << "\t" << (double)rpy_v(2) << endl;
  //cout << "Max Vel calculated" << endl;
  return 0;
}

int TrajectoryGenerator::generateTrajectoriesJS()
{
  Eigen::VectorXd cur_pos, cur_vel, cur_acc, cur_tor;
  calculateMaxVelsJS();
  double old_vel;
  cur_pos = start_pos;
  cur_vel = start_vel;
  cur_acc.resize(DOF);
  cur_tor.resize(DOF);
  for (int i = 0; i < DOF; i++)
  {
    cur_acc(i) = direction(i) * max_a(i);
  }
  q_tra.insert(q_tra.begin(), cur_pos);
  qdot_tra.insert(qdot_tra.begin(), cur_vel);
  qdotdot_tra.insert(qdotdot_tra.begin(), cur_acc);

  double T = t_step;
  while (fabs(T - t_acc - t_step) > accuracy / 1000.0)
  {
    for (int i = 0; i < DOF; i++)
    {
      old_vel = cur_vel(i);
      cur_vel(i) += t_step * cur_acc(i);
      cur_pos(i) += t_step * (cur_vel(i) + old_vel) / 2;
      cur_acc(i) = direction(i) * max_a(i);
    }
    if (!checkJointLimits(cur_pos))
    {
      return -1;
    }
    q_tra.insert(q_tra.begin(), cur_pos);
    qdot_tra.insert(qdot_tra.begin(), cur_vel);
    qdotdot_tra.insert(qdotdot_tra.begin(), cur_acc);
    T += t_step;
  }
  //cout << "Acceleration Trajectory done" << endl;
  if (t_const)
  {
    qdotdot_tra.pop_back();
    cur_acc.setZero(DOF);
    qdotdot_tra.insert(qdotdot_tra.begin(), cur_acc);
    while (fabs(T - t_acc - t_const) > accuracy / 1000.0)
    {
      for (int i = 0; i < DOF; i++)
      {
        old_vel = cur_vel(i);
        cur_vel(i) += t_step * cur_acc(i);
        cur_pos(i) += t_step * (cur_vel(i) + old_vel) / 2;
      }
      if (!checkJointLimits(cur_pos))
      {
        return -1;
      }
      q_tra.insert(q_tra.begin(), cur_pos);
      qdot_tra.insert(qdot_tra.begin(), cur_vel);
      qdotdot_tra.insert(qdotdot_tra.begin(), cur_acc);
      T += t_step;
    }
  }
  else
  {
    T -= t_step;
  }
  //cout << "Constant Trajectory Done" << endl;
  while (fabs(T - time - t_step) > accuracy / 1000.0)
  {
    for (int i = 0; i < DOF; i++)
    {
      old_vel = cur_vel(i);
      cur_vel(i) += t_step * cur_acc(i);
      cur_pos(i) += t_step * (cur_vel(i) + old_vel) / 2;
      cur_acc(i) = -direction(i) * max_d(i);
    }
    if (!checkJointLimits(cur_pos))
    {
      return -1;
    }
    q_tra.insert(q_tra.begin(), cur_pos);
    qdot_tra.insert(qdot_tra.begin(), cur_vel);
    qdotdot_tra.insert(qdotdot_tra.begin(), cur_acc);
    T += t_step;
  }
  cur_acc.setZero(DOF);
  qdotdot_tra.erase(qdotdot_tra.begin());
  qdotdot_tra.insert(qdotdot_tra.begin(), cur_acc);
  return 0;
}
/**
 * Sets the trajectory vectors for position, velocity and acceleration in Cartesian Space
 */
int TrajectoryGenerator::generateTrajectoriesCS(Eigen::Affine3d& p_start, Eigen::Affine3d& p_end, double start_v,
                                                double end_v)
{
  cout << "Start Position" << endl;
  cout << p_start.matrix();
  cout << endl;
  cout << "End Position" << endl;
  cout << p_end.matrix();
  cout << endl;
  calculateMaxVelsCS(p_start, p_end, start_v, end_v);
  Eigen::Matrix3d cur_rot = p_start.rotation();
  //cout << "Generating Trajectory" << endl;
  double T;
  Eigen::Vector3d cur_pos, cur_vel, cur_acc, cur_rpy, q_diff, q_norm;
  q_diff = p_end.translation() - p_start.translation();
  q_norm = q_diff;
  q_norm.normalize();

  cur_pos = p_start.translation();
  cur_vel = start_v * q_norm;
  cur_acc = max_acc * q_norm;
  rotToRPY(cur_rot, cur_rpy);
  pos_tra.insert(pos_tra.begin(), cur_pos);
  vel_tra.insert(vel_tra.begin(), cur_vel);
  acc_tra.insert(acc_tra.begin(), cur_acc);
  rpy_tra.insert(rpy_tra.begin(), cur_rpy);
  T = t_step;
  while (fabs(T - t_acc - t_step) > accuracy / 1000.0)
  {
    cur_acc = acc_tra.front();
    cur_vel = vel_tra.front() + cur_acc * t_step;
    cur_pos = pos_tra.front() + t_step * 0.5 * (cur_vel + vel_tra.front());
    cur_rpy = rpy_tra.front() + rpy_v;
    pos_tra.insert(pos_tra.begin(), cur_pos);
    vel_tra.insert(vel_tra.begin(), cur_vel);
    acc_tra.insert(acc_tra.begin(), cur_acc);
    rpy_tra.insert(rpy_tra.begin(), cur_rpy);
    T += t_step;
  }
  //cout << "Acceleration Trajectory done" << endl;
  if (t_const)
  {
    acc_tra.pop_back();
    cur_acc = cur_acc.Zero();
    acc_tra.insert(acc_tra.begin(), cur_acc);
    while (fabs(T - t_acc - t_const) > accuracy / 1000.0)
    {
      cur_vel = vel_tra.front();
      cur_pos = pos_tra.front() + t_step * cur_vel;
      cur_rpy = rpy_tra.front() + rpy_v;
      pos_tra.insert(pos_tra.begin(), cur_pos);
      vel_tra.insert(vel_tra.begin(), cur_vel);
      acc_tra.insert(acc_tra.begin(), cur_acc);
      rpy_tra.insert(rpy_tra.begin(), cur_rpy);
      T += t_step;
    }
  }
  else
  {
    T -= t_step;
  }
  //cout << "Constant Trajectory Done" << endl;
  while (fabs(T - time - t_step) > accuracy / 1000.0)
  {
    cur_acc = -q_norm * max_dec;
    cur_vel = vel_tra.front() + t_step * acc_tra.front();
    cur_pos = pos_tra.front() + t_step * 0.5 * (cur_vel + vel_tra.front());
    cur_rpy = rpy_tra.front() + rpy_v;
    pos_tra.insert(pos_tra.begin(), cur_pos);
    vel_tra.insert(vel_tra.begin(), cur_vel);
    acc_tra.insert(acc_tra.begin(), cur_acc);
    rpy_tra.insert(rpy_tra.begin(), cur_rpy);
    T += t_step;
  }
  //cout << "current pos: \t" << (double)cur_pos(0) << "\t" << (double)cur_pos(1) << "\t" << (double)cur_pos(2) << endl;
  //cout << "Trajectory completely generated" << endl;
  return 0;
}

/**
 * Converts the trajectory vectors for position, velocity and acceleration in Cartesian Space to Joint Space and generates the Torque Trajectory for it
 */
int TrajectoryGenerator::transformTrajectoryCStoJS()
{
  cout << "Transforming CS to JS Trajectory, this might take a while" << endl;
  Eigen::VectorXd cur_pos(5), next_pos(5), prev_pos(5), cur_vel(5), next_vel(5), cur_acc(5), cur_tor(5);
  Eigen::Vector3d tra_pos, tra_rpy;
  Eigen::VectorXd joint_vel(5);
  double roll, pitch, yaw;
  ik_solver_service::SolveFullyConstrainedIKArray fc_srv;
  ik_solver_service::FullyConstrainedReq request;
  ik_solver_service::FullyConstrainedRes response;
  std::vector<Eigen::Vector3d>::iterator itrpy = rpy_tra.end() - 1;
  for (std::vector<Eigen::Vector3d>::iterator it = pos_tra.end() - 1; it != pos_tra.begin() - 1; it--)
  {
    tra_pos = *it;
    tra_rpy = *itrpy;
    roll = tra_rpy[0];
    pitch = tra_rpy[1];
    yaw = tra_rpy[2];
    request.id = 1;
    request.pitch = pitch;
    request.des_position[0] = tra_pos[0];
    request.des_position[1] = tra_pos[1];
    request.des_position[2] = tra_pos[2];
    request.des_normal[0] = cos(yaw) * sin(pitch) * sin(roll) - sin(yaw) * cos(roll);
    request.des_normal[1] = sin(yaw) * sin(pitch) * sin(roll) + cos(yaw) * cos(roll);
    request.des_normal[2] = cos(pitch) * sin(roll);
    fc_srv.request.ikarray.insert(fc_srv.request.ikarray.begin(), request);
    itrpy--;
  }
  if (solve_fully_constrained_ik_array_client.call(fc_srv))
  {
    while (!fc_srv.response.ikresp.empty())
    {
      response = fc_srv.response.ikresp.back();
      fc_srv.response.ikresp.pop_back();
      if (response.feasible)
      {
        cur_pos(0) = response.joint_angles[0];
        cur_pos(1) = response.joint_angles[1];
        cur_pos(2) = response.joint_angles[2];
        cur_pos(3) = response.joint_angles[3];
        cur_pos(4) = response.joint_angles[4];
        /*  printf("\nfeasible: %d arm_to_front: %d arm_bended_up: %d gripper_downwards: %d\n", response.feasible,
         response.arm_to_front, response.arm_bended_up, response.gripper_downwards);*/
        if (!checkJointLimits(cur_pos))
        {
          return -1;
        }
      }
      else
      {
        cur_pos.setZero(DOF);
        cout << "fully constrained not feasible" << endl;
        q_tra.clear();
        return -1;
      }
      q_tra.insert(q_tra.begin(), cur_pos);
    }
    //Velocity and Acceleration Profiles by numerical differentiation
    std::vector<Eigen::VectorXd>::iterator pp = q_tra.end();
    std::vector<Eigen::VectorXd>::iterator np = q_tra.end() - 2;
    for (std::vector<Eigen::VectorXd>::iterator cp = q_tra.end() - 1; cp != q_tra.begin() - 1; cp--)
    {
      if (pp == q_tra.end())
      {
        prev_pos = *cp;
      }
      else
      {
        prev_pos = *pp;
      }
      cur_pos = *cp;
      if (np == q_tra.begin() - 1)
      {
        next_pos = *cp;
      }
      else
      {
        next_pos = *np;
      }
      //forward difference for velocity
      cur_vel = (next_pos - cur_pos) / t_step;

      //second derivative for acceleration (f(x+h)-2*f(x)+f(x-h))/t_step^2
      cur_acc = (next_pos - 2 * cur_pos + prev_pos) / pow(t_step, 2);

      qdot_tra.insert(qdot_tra.begin(), cur_vel);
      qdotdot_tra.insert(qdotdot_tra.begin(), cur_acc);
      pp--;
      np--;
    }
    cur_acc.setZero(DOF);
    qdotdot_tra.erase(qdotdot_tra.begin());
    qdotdot_tra.erase(qdotdot_tra.end());
    qdotdot_tra.insert(qdotdot_tra.begin(), cur_acc);
    qdotdot_tra.insert(qdotdot_tra.end(), cur_acc);
  }
  else
  {
    ROS_WARN("IK NOT RUNNING");
  }
  return 0;
}

bool TrajectoryGenerator::checkJointLimits(Eigen::VectorXd & pos)
{
  for (int i = 0; i < 5; i++)
  {
    if (i == 2)
    {
      if (pos(i) > 0 || pos(i) < -(fabs(joint_max_angles[i]) + fabs(joint_min_angles[i])))
      {
        ROS_WARN("trajectory not feasible, joint position %d outside of range %f , %f", i + 1,
                 -(fabs(joint_max_angles[i]) + fabs(joint_min_angles[i])), 0.0);
        q_tra.clear();
        qdot_tra.clear();
        qdotdot_tra.clear();
        return false;
      }
    }
    else
    {
      if (pos(i) < 0 || pos(i) > fabs(joint_max_angles[i]) + fabs(joint_min_angles[i]))
      {
        ROS_WARN("trajectory not feasible, joint position %d outside of range %f , %f", i + 1, 0.0,
                 (fabs(joint_max_angles[i]) + fabs(joint_min_angles[i])));
        q_tra.clear();
        qdot_tra.clear();
        qdotdot_tra.clear();
        return false;
      }
    }
    // cout << cur_pos(i) << "\t";
  }
  return true;
}

bool TrajectoryGenerator::bricsPosToEigen(const brics_actuator::JointPositions joint_pos, Eigen::VectorXd & out)
{
  int length = joint_pos.positions.size();
  if (length != 5)
  {
    ROS_ERROR("Invalid Position Request, number of positions received is %d instead of 5",
              (int )joint_pos.positions.size());
    return false;
  }
  else
  {
    const std::string unit = boost::units::to_string(boost::units::si::radian);
    for (int i = 0; i < length; i++)
    {
      int j;
      for (j = 0; j < DOF; j++)
      {
        if (joint_pos.positions[i].joint_uri == joint_names[j])
        {
          out(j) = joint_pos.positions[i].value;
          // Check for correct Unit
          if (unit != joint_pos.positions[i].unit)
          {
            ROS_WARN("Unit incompatibility for %s position. Are you sure you want to command %s instead of %s ?",
                     joint_names[j].c_str(), joint_pos.positions[i].unit.c_str(), unit.c_str());
            return false;
          }
          break;
        }
      }
      if (j > 4)
      {
        ROS_WARN("%s is not a valid joint name.", joint_pos.positions[i].joint_uri.c_str());
        return false;
      }
    }
    // Check for correct value range
    return checkJointLimits(out);
    return true;
  }
}

bool TrajectoryGenerator::bricsVelToEigen(const brics_actuator::JointVelocities joint_vel, Eigen::VectorXd & out)
{
  int length = joint_vel.velocities.size();
  if (length != 5)
  {
    ROS_ERROR("Invalid Velocity Request, number of velocities received is %d instead of 5",
              (int )joint_vel.velocities.size());
    return false;
  }
  else
  {
    const std::string unit = boost::units::to_string(boost::units::si::radian_per_second);
    for (int i = 0; i < length; i++)
    {
      int j;
      for (j = 0; j < DOF; j++)
      {
        if (joint_vel.velocities[i].joint_uri == joint_names[j])
        {
          out(j) = joint_vel.velocities[i].value;
          // Check for correct Unit
          if (unit != joint_vel.velocities[i].unit)
          {
            ROS_WARN("Unit incompatibility for %s velocity. Are you sure you want to command %s instead of %s ?",
                     joint_names[j].c_str(), joint_vel.velocities[i].unit.c_str(), unit.c_str());
            return false;
          }
          break;
        }
      }
      if (j > 4)
      {
        ROS_WARN("%s is not a valid joint name.", joint_vel.velocities[i].joint_uri.c_str());
        return false;
      }
    }
    return true;
  }
}

void TrajectoryGenerator::smoothAffine(Eigen::Affine3d & out)
{
  Eigen::Matrix3d rot;
  Eigen::Vector3d vec;
  rot = out.rotation();
  vec = out.translation();
  for (int i = 0; i < 3; i++)
  {
    for (int j = 0; j < 3; j++)
    {
      rot(i, j) = round((double)rot(i, j) * 10000.0) / 10000.0;
    }
    vec(i) = round((double)vec(i) * 10000.0) / 10000.0;
  }
  out.matrix().col(3) << vec, 1;
  out.matrix().col(2) << rot.col(2), 0;
  out.matrix().col(1) << rot.col(1), 0;
  out.matrix().col(0) << rot.col(0), 0;
}

void TrajectoryGenerator::poseToEigen(const geometry_msgs::Pose pose, Eigen::Affine3d & out)
{
  Eigen::Vector3d lin;
  lin(0) = pose.position.x;
  lin(1) = pose.position.y;
  lin(2) = pose.position.z;
  Eigen::Quaterniond rot(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);
  Eigen::Matrix3d rotm = rot.toRotationMatrix();
  out.matrix().col(3) << lin, 1;
  out.matrix().col(0) << rotm.col(0), 0;
  out.matrix().col(1) << rotm.col(1), 0;
  out.matrix().col(2) << rotm.col(2), 0;
  smoothAffine(out);
}

void TrajectoryGenerator::genToTrajectory(TrajectoryGenerator * gen, trajectory_msgs::JointTrajectory &trajectory)
{
  trajectory_msgs::JointTrajectoryPoint point;
  while (!gen->q_tra.empty())
  {
    point.positions.clear();
    point.velocities.clear();
    point.accelerations.clear();
    Eigen::VectorXd pos(5), vel(5), acc(5);
    pos = gen->q_tra.back();
    vel = gen->qdot_tra.back();
    acc = gen->qdotdot_tra.back();
    for (int i = 0; i < 5; i++)
    {
      point.positions.insert(point.positions.begin(), ((double)pos(i)));
      point.velocities.insert(point.velocities.begin(), ((double)vel(i)));
      point.accelerations.insert(point.accelerations.begin(), ((double)acc(i)));
    }
    gen->q_tra.pop_back();
    gen->qdot_tra.pop_back();
    gen->qdotdot_tra.pop_back();
    trajectory.points.insert(trajectory.points.begin(), point);
  }
  for (int i = 0; i < 5; i++)
  {
    trajectory.joint_names.insert(trajectory.joint_names.begin(), joint_names[i]);
  }
}

void TrajectoryGenerator::rotToRPY(Eigen::Matrix3d & rot, Eigen::Vector3d & rpy)
{
  rpy(1) = atan2(-(double)rot(2, 0), sqrt(pow((double)rot(0, 0), 2)) + pow((double)rot(1, 0), 2));
  if (fabs((double)rpy(1) - M_PI / 2) < DBL_EPSILON)
  {
    rpy(2) = 0;
    rpy(0) = atan2((double)rot(0, 1), (double)rot(1, 1));
  }
  else if (fabs((double)rpy(1) + M_PI / 2) < DBL_EPSILON)
  {
    rpy(2) = 0;
    rpy(0) = -atan2((double)rot(0, 1), (double)rot(1, 1));
  }
  else
  {
    rpy(2) = atan2((double)rot(1, 0) / cos((double)rpy(1)), (double)rot(0, 0) / cos((double)rpy(1)));
    rpy(0) = atan2((double)rot(2, 1) / cos((double)rpy(1)), (double)rot(2, 2) / cos((double)rpy(1)));
  }
}

TrajectoryGenerator* TrajectoryGenerator::getCircle(double radius, double omega, Eigen::Vector3d rpy,
                                                    Eigen::Vector3d center)
{
  Eigen::Vector3d cur_pos, cur_rpy, temp;
  Eigen::Matrix3d rotm = Eigen::Matrix3d(
      Eigen::AngleAxisd(rpy(2), Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(rpy(1), Eigen::Vector3d::UnitY()));
  cout << "Rotation" << endl;
  cout << rotm << endl;
  double time;
  if (!initialize(0.001, 1000))
  {
    ROS_WARN("Trajectory Generator initialize failed");
  }
  else
  {
    for (int i = 0; i <= 2 * M_PI / (omega * t_step); i++)
    {
      time = (double)i * t_step;
      temp(0) = 0;
      temp(1) = radius * sin(omega * time);
      temp(2) = radius * cos(omega * time);
      cur_pos = center + rotm * temp;
      pos_tra.insert(pos_tra.begin(), cur_pos);
      cur_rpy(0) = rpy(0);
      cur_rpy(1) = rpy(1);
      cur_rpy(2) = atan2((double)cur_pos(1), (double)cur_pos(0));
      rpy_tra.insert(rpy_tra.begin(), cur_rpy);
    }
    transformTrajectoryCStoJS();
    ROS_INFO("Trajectory Generator trajectories generated");
  }
  return this;
}

void TrajectoryGenerator::compareRotation(Eigen::Affine3d & start, Eigen::Affine3d & end)
{
  Eigen::Matrix3d rot_s, rot_e;
  Eigen::Vector3d vec;
  rot_s = start.rotation();
  rot_e = end.rotation();
  int k = 0;
  for (int i = 0; i < 3; i++)
  {
    for (int j = 0; j < 3; j++)
    {
      if (fabs((double)rot_s(i, j) - (double)rot_e(i, j)) < 0.001)
      {
        k++;
      }
    }
  }
  if (k >= 5)
  {
    start.matrix().col(2) << rot_e.col(2), 0;
    start.matrix().col(1) << rot_e.col(1), 0;
    start.matrix().col(0) << rot_e.col(0), 0;
  }
}

} // namespace trajectory_generator
