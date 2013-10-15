#ifndef RPG_YOUBOT_COMMON_H
#define RPG_YOUBOT_COMMON_H

#include "ros/ros.h"
#include <sstream>
#include <brics_actuator/JointPositions.h>
#include <brics_actuator/JointVelocities.h>

#include <boost/units/systems/si.hpp>

#define DEG_TO_RAD(x) ((x) * M_PI / 180.0)

namespace rpg_youbot_common
{

double normalize_angle(double angle);

brics_actuator::JointPositions generate_joint_position_msg(double* joints);

brics_actuator::JointPositions generate_gripper_position_msg(double gripper_l, double gripper_r);

brics_actuator::JointVelocities generate_joint_velocity_msg(double* joints);

} // namepsace

#endif

