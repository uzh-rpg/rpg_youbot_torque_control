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

#include "rpg_youbot_common/rpg_youbot_common.h"

namespace rpg_youbot_common
{

double deg2Rad(double angle_in_deg)
{
  return (angle_in_deg * M_PI / 180.0);
}

double normalizeAngle(double angle)
{
  while (angle > M_PI)
    angle -= 2 * M_PI;
  while (angle < -M_PI)
    angle += 2 * M_PI;

  return angle;
}

brics_actuator::JointPositions generateJointPositionMsg(double* joints)
{
  brics_actuator::JointPositions joint_position_msg;

  std::stringstream jointName;
  joint_position_msg.positions.clear();

  for (int i = 0; i < 5; i++)
  {
    brics_actuator::JointValue joint;

    joint.value = joints[i];
    joint.unit = boost::units::to_string(boost::units::si::radians);
    jointName.str("");
    jointName << "arm_joint_" << (i + 1);
    joint.joint_uri = jointName.str();

    joint_position_msg.positions.push_back(joint);
  }

  return joint_position_msg;
}

brics_actuator::JointPositions generateGripperPositionMsg(double gripper_l, double gripper_r)
{
  brics_actuator::JointPositions gripper_position_msg;

  std::stringstream jointName;
  gripper_position_msg.positions.clear();

  brics_actuator::JointValue joint_l;
  joint_l.value = gripper_l;
  joint_l.unit = boost::units::to_string(boost::units::si::meter);
  jointName.str("");
  jointName << "gripper_finger_joint_l";
  joint_l.joint_uri = jointName.str();
  gripper_position_msg.positions.push_back(joint_l);

  brics_actuator::JointValue joint_r;
  joint_r.value = gripper_r;
  joint_r.unit = boost::units::to_string(boost::units::si::meter);
  jointName.str("");
  jointName << "gripper_finger_joint_r";
  joint_r.joint_uri = jointName.str();
  gripper_position_msg.positions.push_back(joint_r);

  return gripper_position_msg;
}

brics_actuator::JointVelocities generateJointVelocityMsg(double* joints)
{
  brics_actuator::JointVelocities joint_velocity_msg;

  std::stringstream jointName;
  joint_velocity_msg.velocities.clear();

  for (int i = 0; i < 5; i++)
  {
    brics_actuator::JointValue joint;

    joint.value = joints[i];
    joint.unit = boost::units::to_string(boost::units::si::radian_per_second);
    jointName.str("");
    jointName << "arm_joint_" << (i + 1);
    joint.joint_uri = jointName.str();

    joint_velocity_msg.velocities.push_back(joint);
  }

  return joint_velocity_msg;
}

} // namespace rpg_youbot_common
