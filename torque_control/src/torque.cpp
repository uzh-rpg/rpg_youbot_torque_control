/*
 * torque.cpp
 *
 *  Created on: May 2, 2013
 *      Author: keiserb
 */

#include "torque.hpp"
#include <time.h>
#include <ros/package.h>

using namespace std;

TorqueController::TorqueController(ros::NodeHandle& nh, std::string name) :
    traj(nh, name, boost::bind(&TorqueController::followTrajectory, this, _1), false), DOF(5), duration(0)

{
  // arm joints (always 5)
  m_joint_state.name.assign(5, "0        10       20"); // presized string
  m_joint_state.position.resize(5);
  m_joint_state.velocity.resize(5);
  m_joint_state.effort.resize(5);

  rp.open("values/req_pos.txt", std::ios::out);
  ap.open("values/act_pos.txt", std::ios::out);
  rv.open("values/req_vel.txt", std::ios::out);
  av.open("values/act_vel.txt", std::ios::out);
  ra.open("values/req_acc.txt", std::ios::out);
  fft.open("values/ff_tor.txt", std::ios::out);
  pdt.open("values/pd_tor.txt", std::ios::out);
  eft.open("values/ef_tor.txt", std::ios::out);
  action_name = name;
  mode = "idle";

  nh.param("youBotDriverCycleFrequencyInHz", lr, 50.0);

  joint_state_sub = nh.subscribe("/joint_states", 1, &TorqueController::jointstateCallback, this);

  srv_step = nh.advertiseService("step_response", &TorqueController::stepCallback, this);
  srv_grav_on = nh.advertiseService("turn_gravity_compensation_on", &TorqueController::gravityOnCallback, this);
  srv_grav_off = nh.advertiseService("turn_gravity_compensation_off", &TorqueController::gravityOffCallback, this);

  torque_command_pub = nh.advertise<brics_actuator::JointTorques>("/arm_1/arm_controller/torques_command", 1);
  pos_command_pub = nh.advertise<brics_actuator::JointPositions>("/arm_1/arm_controller/position_command", 1);

  traj.start();
}

bool TorqueController::initialize()
{
  m_q.setZero(DOF);
  m_qdot.setZero(DOF);
  m_qdotdot.setZero(DOF);
  m_torques.setZero(DOF);
  q_tra.setZero(DOF);
  qdot_tra.setZero(DOF);
  qdotdot_tra.setZero(DOF);
  eff_torques.setZero(DOF);
  Kp.setZero(DOF, DOF);
  Kv.setZero(DOF, DOF);
  std::string package_path = ros::package::getPath("torque_control");

  if (!gainMatrices(Kp, Kv, package_path))
  {
    return false;
  }
  cout << "Kp" << endl;
  cout << Kp << endl;
  cout << "Kv" << endl;
  cout << Kv << endl;

  std::stringstream jointName;

  for (int i = 0; i < DOF; i++)
  {
    brics_actuator::JointValue joint;
    joint.value = 0;
    joint.unit = boost::units::to_string(boost::units::si::radian);
    jointName.str("");
    jointName << "arm_joint_" << (i + 1);
    joint.joint_uri = jointName.str();

    stop.positions.push_back(joint);
  }
  return true;
}

brics_actuator::JointTorques TorqueController::generate_joint_torque_msg(Eigen::VectorXd arr)
{
  brics_actuator::JointTorques m_joint_torques;
  //Ros component negates torque values for joints with negative direction (all joints except joint 3)
  arr[2] = -arr[2];
  std::stringstream jointName;
  m_joint_torques.torques.clear();

  for (int i = 0; i < DOF; i++)
  {
    brics_actuator::JointValue joint;
    joint.value = arr[i];
    joint.unit = boost::units::to_string(boost::units::si::newton_meter);
    jointName.str("");
    jointName << "arm_joint_" << (i + 1);
    joint.joint_uri = jointName.str();

    m_joint_torques.torques.push_back(joint);
  }
  return m_joint_torques;
}

void TorqueController::jointstateCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
  int size = 0;
  for (int j = 0; j < msg->position.size(); j++)
  {
    for (int i = 0; i < YOUBOT_NR_OF_JOINTS; i++)
    {
      if (msg->name[j] == joint_names[i])
      {
        m_joint_state.position[i] = msg->position[j];
        m_joint_state.velocity[i] = msg->velocity[j];
        if (msg->effort.size() >= YOUBOT_NR_OF_JOINTS)
        {
          m_joint_state.effort[i] = msg->effort[j];   //no effort msg in webots
        }
        size++;
      }
    }
  }
  if (size == YOUBOT_NR_OF_JOINTS)
  {
    //translate values into Torque positions
    m_q(0) = m_joint_state.position[0] - joint_offsets[0];
    m_q(1) = m_joint_state.position[1] - joint_offsets[1];
    m_q(2) = joint_offsets[2] - m_joint_state.position[2];
    m_q(3) = m_joint_state.position[3] - joint_offsets[3];
    m_q(4) = m_joint_state.position[4] - joint_offsets[4];
    m_qdot(0) = m_joint_state.velocity[0];
    m_qdot(1) = m_joint_state.velocity[1];
    m_qdot(2) = -m_joint_state.velocity[2];
    m_qdot(3) = m_joint_state.velocity[3];
    m_qdot(4) = m_joint_state.velocity[4];
    eff_torques(0) = m_joint_state.effort[0];
    eff_torques(1) = m_joint_state.effort[1];
    eff_torques(2) = -m_joint_state.effort[2];
    eff_torques(3) = m_joint_state.effort[3];
    eff_torques(4) = m_joint_state.effort[4];
    for (int i = 0; i < DOF; i++)
    {
      stop.positions[i].value = m_joint_state.position[i];
    }
  }
  else
  {
    ROS_INFO("NO JOINT STATES FOR YOUBOT ARM RECEIVED");
  }
}

void TorqueController::followTrajectory(const torque_control::torque_trajectoryGoalConstPtr & trajectory)
{
  mode = "trajectory";
  ROS_INFO("Following Trajectory");
  Eigen::VectorXd pos_err(5), vel_err(5), a_pos(5), d_pos(5);
  int rval;
  ros::Rate loop_rate(lr);
  trajectory_msgs::JointTrajectory j_traj = trajectory->trajectory;
  while (!j_traj.points.empty())
  {
    ros::spinOnce();
    if(!pointToEigen(q_tra, qdot_tra, qdotdot_tra, j_traj.points.back(),j_traj.joint_names))
    {
      ros::shutdown();
    }
    j_traj.points.pop_back();

    //translate youbot trajectories into torque trajectories for torque calculation
    q_tra = youbot2torque(q_tra);
    qdot_tra(2) = -qdot_tra(2);
    qdotdot_tra(2) = -qdotdot_tra(2);

    pos_err = m_q - q_tra;
    vel_err = m_qdot - qdot_tra;

    calcTorques(q_tra, qdot_tra, qdotdot_tra, m_torques);

    writeToFile(fft, m_torques);

    pd_controller_Torques(Kp, Kv, m_q, pos_err, vel_err, m_torques);

    rval = limitTorques(m_torques);

    //translate youbot values into torque values for saving
    d_pos = torque2youbot(q_tra);
    a_pos = torque2youbot(m_q);
    qdot_tra(2) = -qdot_tra(2);
    m_qdot(2) = -m_qdot(2);
    qdotdot_tra(2) = -qdotdot_tra(2);
    m_qdotdot(2) = -m_qdotdot(2);

    writeToFile(rp, d_pos);
    writeToFile(ap, a_pos);
    writeToFile(rv, qdot_tra);
    writeToFile(av, m_qdot);
    writeToFile(ra, qdotdot_tra);
    writeToFile(pdt, m_torques);
    writeToFile(eft, eff_torques);
    if (rval != 0)
    {
      j_traj.points.clear();
      ros::shutdown();
      break;
    }
    torque_command_pub.publish(generate_joint_torque_msg(m_torques));
    ros::spinOnce();
    loop_rate.sleep();
  }
  pos_command_pub.publish(stop);
  ros::spinOnce();
  loop_rate.sleep();
  traj.setSucceeded(result);
  mode = "idle";
}
bool TorqueController::stepCallback(torque_control::step::Request &req, torque_control::step::Response &res)
{
  ROS_INFO("step input mode on");
  Eigen::VectorXd pos(5);
  if (!brics2eigen(req.position, pos))
  {
    ROS_ERROR("INVALID STEP POSITION");
    return false;
  }
  q_tra = youbot2torque(pos);
  qdot_tra.setZero(DOF);
  qdotdot_tra.setZero(DOF);
  duration = req.duration;
  mode = "step";
  return true;
}

void TorqueController::stepInput()
{
  ros::Rate loop_rate(lr);
  int rval;
  int k = 0;
  Eigen::VectorXd pos_err(5), vel_err(5), d_pos(5), a_pos(5);
  while (k < (int)(duration * lr))
  {
    //cout << q_tra << endl;
    ros::spinOnce();
    calcTorques(q_tra, qdot_tra, qdotdot_tra, m_torques);

    pos_err = m_q - q_tra;
    vel_err = m_qdot - qdot_tra;

    writeToFile(fft, m_torques);

    pd_controller_Torques(Kp, Kv, m_q, pos_err, vel_err, m_torques);

    rval = limitTorques(m_torques);

    d_pos = torque2youbot(q_tra);
    a_pos = torque2youbot(m_q);
    m_qdot(2)=-m_qdot(2);

    writeToFile(rp, d_pos);
    writeToFile(ap, a_pos);
    writeToFile(rv, qdot_tra);
    writeToFile(av, m_qdot);
    writeToFile(ra, qdotdot_tra);
    writeToFile(pdt, m_torques);
    writeToFile(eft, eff_torques);
    if (rval != 0)
    {
      ros::shutdown();
    }
    torque_command_pub.publish(generate_joint_torque_msg(m_torques));
    ros::spinOnce();
    loop_rate.sleep();
    k++;
  }
  ROS_INFO("step response done");
  pos_command_pub.publish(stop);
  ros::spinOnce();
  loop_rate.sleep();
  mode = "idle";
}

bool TorqueController::gravityOnCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Request &res)
{
  ROS_INFO("turning gravity compensation on");
  mode = "gravity";
  return true;
}

bool TorqueController::gravityOffCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Request &res)
{
  ROS_INFO("turning gravity compensation off");
  pos_command_pub.publish(stop);
  ros::spinOnce();
  mode = "idle";
  return true;
}

void TorqueController::gravityCompensation()
{
  int rval;
  ros::Rate loop_rate(lr);
  Eigen::VectorXd a_pos(5);
  a_pos = torque2youbot(m_q);
  writeToFile(rp, a_pos);
  writeToFile(ap, a_pos);
  writeToFile(rv, m_qdot);
  writeToFile(av, m_qdot);
  writeToFile(ra, qdotdot_tra.setZero(DOF));
  writeToFile(fft, m_torques);
  writeToFile(pdt, m_torques);
  writeToFile(eft, eff_torques);
  m_qdot.setZero(DOF);
  m_qdotdot.setZero(DOF);
  calcTorques(m_q, m_qdot, m_qdotdot, m_torques);
  rval = limitTorques(m_torques);
  if (rval == 0)
  {
    torque_command_pub.publish(generate_joint_torque_msg(m_torques));
  }
  else
  {
    ROS_ERROR("EMERGENCY STOP");
    ros::shutdown();
    mode == "idle";
  }
  ros::spinOnce();
  loop_rate.sleep();
}

timespec diff(timespec start, timespec end)
{
  timespec temp;
  if ((end.tv_nsec - start.tv_nsec) < 0)
  {
    temp.tv_sec = end.tv_sec - start.tv_sec - 1;
    temp.tv_nsec = 1000000000 + end.tv_nsec - start.tv_nsec;
  }
  else
  {
    temp.tv_sec = end.tv_sec - start.tv_sec;
    temp.tv_nsec = end.tv_nsec - start.tv_nsec;
  }
  return temp;
}

int TorqueController::limitTorques(Eigen::VectorXd & torques)
{
  //Implement smart torque limiting feature here based on position and velocity
  Eigen::VectorXd br_vel, br_acc;
  br_vel.resize(5);
  br_acc.resize(5);
  br_vel = m_qdot;
  br_acc = m_qdotdot;
  for (int i = 0; i < 5; i++)
  {
    if (m_q(i) > 0.95 * joint_max_angles[i])
    {
      if (m_qdot(i) > 0.2)
      {
        torque2youbot(m_q);
        ROS_ERROR("CRITICAL POSITION %f AND VELOCITY %f ON JOINT %i", (float )m_q(i), (float )m_qdot(i), i + 1);
        pos_command_pub.publish(stop);
        return -1;
      }
    }
    else if (m_q(i) < 0.95 * joint_min_angles[i])
    {
      if (m_qdot(i) < -0.2)
      {
        torque2youbot(m_q);
        ROS_ERROR("CRITICAL POSITION %f AND VELOCITY %f ON JOINT %i", (float )m_q(i), (float )m_qdot(i), i + 1);
        pos_command_pub.publish(stop);
        return -1;
      }
    }
    if (isnan((double)torques[i]))
    {
      torques(i) = 0;
      ROS_ERROR("NAN TORQUES");
      pos_command_pub.publish(stop);
      return -1;
    }
    if (fabs((double)torques[i]) > 1.0 * joint_torque_max[i])
    {
      ROS_INFO("Torque value on joint %i too high, rescaled to max torque", i + 1);
      if (torques(i) < 0)
        torques(i) = -1 * joint_torque_max[i];
      else
        torques(i) = 1 * joint_torque_max[i];
    }
  }
  return 0;
}

void TorqueController::writeToFile(std::ofstream & file, Eigen::VectorXd & vect)
{
  for (int i = 0; i < vect.size(); i++)
  {
    file << vect[i] << "\t";
  }
  file << endl;
}

bool TorqueController::pointToEigen(Eigen::VectorXd & pos, Eigen::VectorXd & vel, Eigen::VectorXd & acc,
                                    trajectory_msgs::JointTrajectoryPoint & point, std::vector<std::string> & joint_names)
{
  std::string name;
  if(joint_names.size()!=5 || point.positions.size()!=5 || point.velocities.size()!=5 || point.accelerations.size()!=5)
  {
    ROS_ERROR("PLEASE SUPPLY TRAJECTORY FOR ALL 5 JOINTS WITH VALID NAMES, POSITIONS, VELOCITIES AND ACCELERATIONS");
    return false;
  }
  int i = 0;
  int n = 0;
  while (!point.positions.empty())
  {
    name = joint_names.at(n);
    if(name == joint_names[0])
    {
      i=0;
    }
    else if(name == joint_names[1])
    {
      i=1;
    }
    else if(name == joint_names[2])
    {
      i=2;
    }
    else if(name == joint_names[3])
    {
      i=3;
    }
    else if(name == joint_names[4])
    {
      i=4;
    }
    else
    {
      ROS_ERROR("NAMES SUPPLIED ARE INVALID FOR YOUBOT");
      return false;
    }
    pos(i) = point.positions.back();
    vel(i) = point.velocities.back();
    acc(i) = point.accelerations.back();
    point.positions.pop_back();
    point.velocities.pop_back();
    point.accelerations.pop_back();
    n++;
  }
  return true;
}

bool TorqueController::brics2eigen(brics_actuator::JointPositions jpos, Eigen::VectorXd & pos)
{
  Eigen::VectorXd temp(5);
  temp = torque2youbot(m_q);
  ROS_INFO("New Target Position received");
  pos = temp;
  int length = jpos.positions.size();
  const std::string unit = boost::units::to_string(boost::units::si::radian);
  bool valid = true;
  for (int i = 0; i < length; i++)
  {
    int j;
    for (j = 0; j < DOF; j++)
    {
      if (jpos.positions[i].joint_uri == joint_names[j])
      {
        pos(j) = jpos.positions[i].value;
        // Check for correct Unit
        if (unit != jpos.positions[i].unit)
        {
          ROS_WARN("Unit incompatibility for %s position. Are you sure you want to command %s instead of %s ?",
                   joint_names[j].c_str(), jpos.positions[i].unit.c_str(), unit.c_str());
          pos(j) = temp(j);
          valid = false;
          return valid;
        }
        // Check for correct value range
        if (j == 2)
        {
          if (pos(j) > 0 || pos(j) < -(fabs(joint_max_angles[j]) + fabs(joint_min_angles[j])))
          {
            ROS_WARN("Desired joint angle for %s out of range (%f). Should be within [%f, %f].", joint_names[j].c_str(),
                     jpos.positions[j].value, -(fabs(joint_max_angles[j]) + fabs(joint_min_angles[j])), 0.0);
            pos(j) = temp(j);
            valid = false;
            return valid;
          }
        }
        else
        {
          if (pos(j) < 0 || pos(j) > fabs(joint_max_angles[j]) + fabs(joint_min_angles[j]))
          {
            ROS_WARN("Desired joint angle for %s out of range (%f). Should be within [%f, %f].", joint_names[j].c_str(),
                     jpos.positions[j].value, 0.0, fabs(joint_max_angles[j]) + fabs(joint_min_angles[j]));
            pos(j) = temp(j);
            valid = false;
            return valid;
          }
        }
        break;
      }
    }
    if (j > 4)
    {
      ROS_WARN("%s is not a valid joint name.", jpos.positions[i].joint_uri.c_str());
      valid = false;
      return valid;
    }
  }
  if (!valid)
  {
    pos = temp;
    return false;
  }
  return true;
}

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "torque_control");
  ros::NodeHandle nh;
  double loop;
  nh.param("youBotDriverCycleFrequencyInHz", loop, 50.0);
  ros::Rate loop_rate(loop);
  TorqueController torque(nh, ros::this_node::getName());
  if (!torque.initialize())
  {
    return -1;
  }
  ROS_INFO("Torque Controller Running");
  while (ros::ok())
  {
    if (torque.mode == "gravity")
    {
      torque.gravityCompensation();
    }
    else if (torque.mode == "step")
    {
      torque.stepInput();
    }
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
