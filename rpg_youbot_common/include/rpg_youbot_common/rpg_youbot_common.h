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

#ifndef RPG_YOUBOT_COMMON_H
#define RPG_YOUBOT_COMMON_H

#include "ros/ros.h"
#include <sstream>
#include <brics_actuator/JointPositions.h>
#include <brics_actuator/JointVelocities.h>

#include <boost/units/systems/si.hpp>
#include <boost/units/io.hpp>

namespace rpg_youbot_common
{

double deg2Rad(double angle_in_deg);

double normalizeAngle(double angle);

brics_actuator::JointPositions generateJointPositionMsg(double* joints);

brics_actuator::JointPositions generateGripperPositionMsg(double gripper_l, double gripper_r);

brics_actuator::JointVelocities generateJointVelocityMsg(double* joints);

} // namespace rpg_youbot_common

#endif

