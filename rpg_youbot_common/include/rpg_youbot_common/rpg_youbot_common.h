#ifndef RPG_YOUBOT_COMMON_H
#define RPG_YOUBOT_COMMON_H

#include "ros/ros.h"
#include <sstream>
#include <brics_actuator/JointPositions.h>
#include <brics_actuator/JointVelocities.h>

#include <boost/units/systems/si.hpp>
#include <boost/units/io.hpp>

#define DEG_TO_RAD(x) ((x) * M_PI / 180.0)

namespace rpg_youbot_common
{

double normalizeAngle(double angle);

brics_actuator::JointPositions generateJointPositionMsg(double* joints);

brics_actuator::JointPositions generateGripperPositionMsg(double gripper_l, double gripper_r);

brics_actuator::JointVelocities generateJointVelocityMsg(double* joints);

} // rpg_youbot_common

#endif

