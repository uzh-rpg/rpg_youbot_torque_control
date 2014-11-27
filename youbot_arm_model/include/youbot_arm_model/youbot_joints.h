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

#ifndef YOUBOTJOINTS_HPP_
#define YOUBOTJOINTS_HPP_

#define YOUBOT_NR_OF_WHEELS 4
#define YOUBOT_NR_OF_JOINTS 5

const std::string joint_names[5] = {"arm_joint_1", "arm_joint_2", "arm_joint_3", "arm_joint_4", "arm_joint_5"};
const double joint_min_angles[5] = {-2.9496,-1.1344,-2.6354,-1.7889,-2.9234};
const double joint_max_angles[5] = {2.9496,1.5707,2.5481,1.7889,2.9234};
const double joint_min_vel[5] = {-1.0,-1.0,-1.0,-1.0,-1.0};
const double joint_max_vel[5] = {1.0,1.0,1.0,1.0,1.0};
const double joint_offsets[5] = {2.9496, 1.1344, -2.5481, 1.7889, 2.9234};
//const double joint_torque_max[5] = {9.5,9.5,6.0,2.0,1.0};
const double joint_torque_max[5] = {17.0,17.0,8.0,2.7,1.0}; // Values from experiment with position controller

#endif
