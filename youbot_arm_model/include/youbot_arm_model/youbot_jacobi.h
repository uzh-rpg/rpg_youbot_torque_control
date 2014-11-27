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

 * YoubotArmModel.hpp
 *
 *  Created on: May 29, 2013
 *      Author: keiserb
 */

#ifndef YOUBOTJACOBI_HPP_
#define YOUBOTJACOBI_HPP_

#include <math.h>
#include <kdl/jntarray.hpp>
#include <kdl/frames.hpp>
#include <Eigen/Dense>

using namespace Eigen;

inline void getJacobi(MatrixXd & J, KDL::JntArray & pos)
{
	double cos00 = cos(pos(0));
	double sin00 = sin(pos(0));
	double cos01 = cos(pos(1));
	double cos02 = cos(pos(0) - pos(1) + pos(2) - pos(3));
	double sin01 = sin(pos(1));
	double cos03 = cos(pos(0) + pos(1) - pos(2) + pos(3));
	double sin02 = sin(pos(0) + pos(1) - pos(2) + pos(3));
	double sin03 = sin(pos(0) - pos(1) + pos(2) - pos(3));
	double cos04 = cos(pos(1) - pos(2) + pos(3));
	double sin04 = sin(pos(1) - pos(2) + pos(3));
	J(0,0) = 0.0;
	J(0,1) = cos00 * (l1z + l2z + lbz);
	J(0,2) = -(cos00 * (l1z + l2z + lbz + (l3z * cos01)));
	J(0,3) = cos00 * (l1z + l2z + lbz + (l3z * cos01) + (l4z * cos(pos(1) - pos(2))));
	J(0,4) = (l1z * cos02 / 2.0) + (l2z * cos02 / 2.0) + (lbz * cos02 / 2.0) + (l2x * sin03 / 2.0) + (l3z * cos(pos(0) + pos(2) - pos(3)) / 2.0) - (l3z * cos(pos(0) - pos(2) + pos(3)) / 2.0) - (l4z * cos(pos(0) + pos(3)) / 2.0) + (l5x * sin00) - (l1z * cos03 / 2.0) - (l2z * cos03 / 2.0) - (lbz * cos03 / 2.0) + (l2x * sin02 / 2.0) + (l4z * cos(pos(0) - pos(3)) / 2.0);
	J(1,0) = -l1x - lbx;
	J(1,1) = sin00 * (l1z + l2z + lbz);
	J(1,2) = -(sin00 * (l1z + l2z + lbz + (l3z * cos01)));
	J(1,3) = sin00 * (l1z + l2z + lbz + (l3z * cos01) + (l4z * cos(pos(1) - pos(2))));
	J(1,4) = (l1z * sin03 / 2.0) - (l2x * cos02 / 2.0) + (l2z * sin03 / 2.0) + (lbz * sin03 / 2.0) - (l1x * cos04) - (lbx * cos04) + (l3z * sin(pos(0) + pos(2) - pos(3)) / 2.0) - (l3z * sin(pos(0) - pos(2) + pos(3)) / 2.0) - (l4z * sin(pos(0) + pos(3)) / 2.0) - (l5x * cos00) - (l2x * cos03 / 2.0) - (l1z * sin02 / 2.0) - (l2z * sin02 / 2.0) - (lbz * sin02 / 2.0) + (l4z * sin(pos(0) - pos(3)) / 2.0);
	J(2,0) = 0.0;
	J(2,1) = -l2x - (l1x * cos00) - (lbx * cos00);
	J(2,2) = (cos01 * (l1x + l2x + lbx)) - (sin01 * (l1z + l2z + l3z + lbz)) - (cos00 * ((cos00 * (((cos01 - 1.0) * (l1x + l2x + lbx)) - (sin01 * (l1z + l2z + lbz)))) + ((cos00 - 1.0) * (l1x + lbx)))) - (sin00 * ((sin00 * (l1x + lbx)) + (sin00 * (((cos01 - 1.0) * (l1x + l2x + lbx)) - (sin01 * (l1z + l2z + lbz))))));
	J(2,3) = (l3z * sin01) - (l1x * cos00) - (lbx * cos00) - l2x + (l4z * sin(pos(1) - pos(2)));
	J(2,4) = -(sin04 * sin00 * (l1x + lbx));
	J(3,0) = 0.0;
	J(3,1) = sin(pos(0));
	J(3,2) = -sin00;
	J(3,3) = sin(pos(0));
	J(3,4) = -(sin04 * cos00);
	J(4,0) = 0.0;
	J(4,1) = -cos00;
	J(4,2) = cos(pos(0));
	J(4,3) = -cos00;
	J(4,4) = -(sin04 * sin00);
	J(5,0) = 1.0;
	J(5,1) = 0.0;
	J(5,2) = 0.0;
	J(5,3) = 0.0;
	J(5,4) = cos(pos(1) - pos(2) + pos(3));
}

inline void getInverseJacobi(MatrixXd & Jinv, KDL::JntArray & pos)
{
	MatrixXd J(6,5);
	MatrixXd Jt(5,6);
	MatrixXd Jtemp(5,5);
	getJacobi(J, pos);
	Jt = J.transpose();
	Jtemp=Jt*J;
	Jtemp=Jtemp.inverse();
	Jinv=Jtemp*Jt;
}



#endif /* YOUBOTJACOBI_HPP_ */
