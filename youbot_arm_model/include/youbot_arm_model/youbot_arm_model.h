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

#ifndef YOUBOTARMMODEL_HPP_
#define YOUBOTARMMODEL_HPP_

#include <math.h>
#include <kdl/jntarray.hpp>
#include <kdl/frames.hpp>
#include <Eigen/Dense>
#include "ValuesUrdf.hpp"

using namespace Eigen;

void getTwists(MatrixXd & xi_1, MatrixXd & xi_2, MatrixXd & xi_3, MatrixXd & xi_4, MatrixXd & xi_5, MatrixXd & tf,
               Eigen::VectorXd & pos)
{
  double cos1 = cos(pos(0));
  double sin1 = sin(pos(0));
  double sin2 = sin(pos(1));
  double cos2 = cos(pos(1));
  double sin3 = sin(pos(2));
  double cos3 = cos(pos(2));
  double sin4 = sin(pos(3));
  double cos4 = cos(pos(3));
  double cos5 = cos(pos(4));
  double sin5 = sin(pos(4));

  xi_1(0, 0) = cos1;
  xi_1(0, 1) = -sin1;
  xi_1(0, 2) = 0.0;
  xi_1(0, 3) = -((cos1 - 1.0) * (l1x + lbx));
  xi_1(1, 0) = sin1;
  xi_1(1, 1) = cos1;
  xi_1(1, 2) = 0.0;
  xi_1(1, 3) = -(sin1 * (l1x + lbx));
  xi_1(2, 0) = 0.0;
  xi_1(2, 1) = 0.0;
  xi_1(2, 2) = 1.0;
  xi_1(2, 3) = 0.0;
  xi_1(3, 0) = 0.0;
  xi_1(3, 1) = 0.0;
  xi_1(3, 2) = 0.0;
  xi_1(3, 3) = 1.0;

  xi_2(0, 0) = cos2;
  xi_2(0, 1) = 0.0;
  xi_2(0, 2) = -sin2;
  xi_2(0, 3) = (sin2 * (l1z + l2z + lbz)) - ((cos2 - 1.0) * (l1x + l2x + lbx));
  xi_2(1, 0) = 0.0;
  xi_2(1, 1) = 1.0;
  xi_2(1, 2) = 0.0;
  xi_2(1, 3) = 0.0;
  xi_2(2, 0) = sin2;
  xi_2(2, 1) = 0.0;
  xi_2(2, 2) = cos2;
  xi_2(2, 3) = -((cos2 - 1.0) * (l1z + l2z + lbz)) - (sin2 * (l1x + l2x + lbx));
  xi_2(3, 0) = 0.0;
  xi_2(3, 1) = 0.0;
  xi_2(3, 2) = 0.0;
  xi_2(3, 3) = 1.0;

  xi_3(0, 0) = cos3;
  xi_3(0, 1) = 0.0;
  xi_3(0, 2) = sin3;
  xi_3(0, 3) = -((cos3 - 1.0) * (l1x + l2x + lbx)) - (sin3 * (l1z + l2z + l3z + lbz));
  xi_3(1, 0) = 0.0;
  xi_3(1, 1) = 1.0;
  xi_3(1, 2) = 0.0;
  xi_3(1, 3) = 0.0;
  xi_3(2, 0) = -sin3;
  xi_3(2, 1) = 0.0;
  xi_3(2, 2) = cos3;
  xi_3(2, 3) = (sin3 * (l1x + l2x + lbx)) - ((cos3 - 1.0) * (l1z + l2z + l3z + lbz));
  xi_3(3, 0) = 0.0;
  xi_3(3, 1) = 0.0;
  xi_3(3, 2) = 0.0;
  xi_3(3, 3) = 1.0;

  xi_4(0, 0) = cos4;
  xi_4(0, 1) = 0.0;
  xi_4(0, 2) = -sin4;
  xi_4(0, 3) = (sin4 * (l1z + l2z + l3z + l4z + lbz)) - ((cos4 - 1.0) * (l1x + l2x + lbx));
  xi_4(1, 0) = 0.0;
  xi_4(1, 1) = 1.0;
  xi_4(1, 2) = 0.0;
  xi_4(1, 3) = 0.0;
  xi_4(2, 0) = sin4;
  xi_4(2, 1) = 0.0;
  xi_4(2, 2) = cos4;
  xi_4(2, 3) = -((cos4 - 1.0) * (l1z + l2z + l3z + l4z + lbz)) - (sin4 * (l1x + l2x + lbx));
  xi_4(3, 0) = 0.0;
  xi_4(3, 1) = 0.0;
  xi_4(3, 2) = 0.0;
  xi_4(3, 3) = 1.0;

  xi_5(0, 0) = cos5;
  xi_5(0, 1) = -sin5;
  xi_5(0, 2) = 0.0;
  xi_5(0, 3) = -((cos5 - 1.0) * (l1x + l2x + l5x + lbx));
  xi_5(1, 0) = sin5;
  xi_5(1, 1) = cos5;
  xi_5(1, 2) = 0.0;
  xi_5(1, 3) = -(sin5 * (l1x + l2x + l5x + lbx));
  xi_5(2, 0) = 0.0;
  xi_5(2, 1) = 0.0;
  xi_5(2, 2) = 1.0;
  xi_5(2, 3) = 0.0;
  xi_5(3, 0) = 0.0;
  xi_5(3, 1) = 0.0;
  xi_5(3, 2) = 0.0;
  xi_5(3, 3) = 1.0;

  tf(0, 0) = 0;
  tf(0, 1) = 0;
  tf(0, 2) = -1;
  tf(0, 3) = l1x + l2x + l5x + lbx + lfx;
  tf(1, 0) = 0;
  tf(1, 1) = 1;
  tf(1, 2) = 0;
  tf(1, 3) = 0;
  tf(2, 0) = 1;
  tf(2, 1) = 0;
  tf(2, 2) = 0;
  tf(2, 3) = l1z + l2z + l3z + l4z + l5z + lbz + lfz;
  tf(3, 0) = 0.0;
  tf(3, 1) = 0.0;
  tf(3, 2) = 0.0;
  tf(3, 3) = 1.0;
}

void forwardKin(MatrixXd & xi_1, MatrixXd & xi_2, MatrixXd & xi_3, MatrixXd & xi_4, MatrixXd & xi_5, MatrixXd & tf,
                KDL::Frame & cart_pos)
{
  MatrixXd temp(4, 4);
  temp = xi_1 * xi_2 * xi_3 * xi_4 * xi_5 * tf;
  for (int i = 0; i < 3; i++)
  {
    for (int j = 0; j < 3; j++)
    {
      cart_pos.M.data[i * 3 + j] = temp(i, j);
    }
    cart_pos.p.data[i] = temp(i, 3);
  }

}

void getCartPos(Eigen::VectorXd & j_pos, KDL::Frame & cart_pos)
{
  MatrixXd xi_1(4, 4);
  MatrixXd xi_2(4, 4);
  MatrixXd xi_3(4, 4);
  MatrixXd xi_4(4, 4);
  MatrixXd xi_5(4, 4);
  MatrixXd tf(4, 4);
  getTwists(xi_1, xi_2, xi_3, xi_4, xi_5, tf, j_pos);
  forwardKin(xi_1, xi_2, xi_3, xi_4, xi_5, tf, cart_pos);
}

#endif /* YOUBOTARMMODEL_HPP_ */
