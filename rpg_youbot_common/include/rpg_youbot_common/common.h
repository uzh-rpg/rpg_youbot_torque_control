#ifndef RPG_YOUBOT_COMMON_INLINE_H
#define RPG_YOUBOT_COMMON_INLINE_H

#include "ros/ros.h"
#include <sstream>
#include <brics_actuator/JointPositions.h>
#include <brics_actuator/JointVelocities.h>

#include <boost/units/systems/si.hpp>

#define DEG_TO_RAD(x) ((x) * M_PI / 180.0)

namespace rpg_youbot_common
{

inline double normalize_angle(double angle)
{
  while (angle > M_PI)
    angle -= 2*M_PI;
  while (angle < -M_PI)
    angle += 2*M_PI;

  return angle;
}

inline brics_actuator::JointPositions generate_joint_position_msg(double* joints)
{
	brics_actuator::JointPositions joint_position_msg;

	std::stringstream jointName;
	joint_position_msg.positions.clear();

	for (int i=0; i<5; i++)
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

inline brics_actuator::JointPositions generate_gripper_position_msg(double gripper_l, double gripper_r)
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

inline brics_actuator::JointVelocities generate_joint_velocity_msg(double* joints)
{
	brics_actuator::JointVelocities joint_velocity_msg;

	std::stringstream jointName;
	joint_velocity_msg.velocities.clear();

	for (int i=0; i<5; i++)
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


} // namepsace

#endif