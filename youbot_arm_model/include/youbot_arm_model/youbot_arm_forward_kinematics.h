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

#ifndef YOUBOTARMFKIN_HPP_
#define YOUBOTARMFKIN_HPP_

#include <math.h>
#include <kdl/jntarray.hpp>
#include <kdl/frames.hpp>
#include <Eigen/Dense>

#define l2 0.155
#define l3 0.135
#define l4 0.218
#define lox 0.033
#define loz 0.1472
#define lax 0

inline void getTwists(Eigen::MatrixXd & xi_1, Eigen::MatrixXd & xi_2, Eigen::MatrixXd & xi_3, Eigen::MatrixXd & xi_4,
                      Eigen::MatrixXd & xi_5, Eigen::MatrixXd & tf, Eigen::VectorXd & pos)
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
  xi_1(0, 2) = 0;
  xi_1(0, 3) = -lax * (cos1 - 1.0);
  xi_1(1, 0) = sin1;
  xi_1(1, 1) = cos1;
  xi_1(1, 2) = 0;
  xi_1(1, 3) = -lax * sin1;
  xi_1(2, 0) = 0;
  xi_1(2, 1) = 0;
  xi_1(2, 2) = 1.0;
  xi_1(2, 3) = 0;
  xi_1(3, 0) = 0;
  xi_1(3, 1) = 0;
  xi_1(3, 2) = 0;
  xi_1(3, 3) = 1.0;

  xi_2(0, 0) = cos2;
  xi_2(0, 1) = 0;
  xi_2(0, 2) = sin2;
  xi_2(0, 3) = -loz * sin2 - (cos2 - 1.0) * (lax + lox);
  xi_2(1, 0) = 0;
  xi_2(1, 1) = 1.0;
  xi_2(1, 2) = 0;
  xi_2(1, 3) = 0;
  xi_2(2, 0) = -sin2;
  xi_2(2, 1) = 0;
  xi_2(2, 2) = cos2;
  xi_2(2, 3) = sin2 * (lax + lox) - loz * (cos2 - 1.0);
  xi_2(3, 0) = 0;
  xi_2(3, 1) = 0;
  xi_2(3, 2) = 0;
  xi_2(3, 3) = 1.0;

  xi_3(0, 0) = cos3;
  xi_3(0, 1) = 0;
  xi_3(0, 2) = sin3;
  xi_3(0, 3) = -sin3 * (l2 + loz) - (cos3 - 1.0) * (lax + lox);
  xi_3(1, 0) = 0;
  xi_3(1, 1) = 1.0;
  xi_3(1, 2) = 0;
  xi_3(1, 3) = 0;
  xi_3(2, 0) = -sin3;
  xi_3(2, 1) = 0;
  xi_3(2, 2) = cos3;
  xi_3(2, 3) = sin3 * (lax + lox) - (cos3 - 1.0) * (l2 + loz);
  xi_3(3, 0) = 0;
  xi_3(3, 1) = 0;
  xi_3(3, 2) = 0;
  xi_3(3, 3) = 1.0;

  xi_4(0, 0) = cos4;
  xi_4(0, 1) = 0;
  xi_4(0, 2) = sin4;
  xi_4(0, 3) = -(cos4 - 1.0) * (lax + lox) - sin4 * (l2 + l3 + loz);
  xi_4(1, 0) = 0;
  xi_4(1, 1) = 1.0;
  xi_4(1, 2) = 0;
  xi_4(1, 3) = 0;
  xi_4(2, 0) = -sin4;
  xi_4(2, 1) = 0;
  xi_4(2, 2) = cos4;
  xi_4(2, 3) = sin4 * (lax + lox) - (cos4 - 1.0) * (l2 + l3 + loz);
  xi_4(3, 0) = 0;
  xi_4(3, 1) = 0;
  xi_4(3, 2) = 0;
  xi_4(3, 3) = 1.0;

  xi_5(0, 0) = cos5;
  xi_5(0, 1) = -sin5;
  xi_5(0, 2) = 0;
  xi_5(0, 3) = -(cos5 - 1.0) * (lax + lox);
  xi_5(1, 0) = sin5;
  xi_5(1, 1) = cos5;
  xi_5(1, 2) = 0;
  xi_5(1, 3) = -sin5 * (lax + lox);
  xi_5(2, 0) = 0;
  xi_5(2, 1) = 0;
  xi_5(2, 2) = 1.0;
  xi_5(2, 3) = 0;
  xi_5(3, 0) = 0;
  xi_5(3, 1) = 0;
  xi_5(3, 2) = 0;
  xi_5(3, 3) = 1.0;

  tf(0, 0) = 0;
  tf(0, 1) = 0;
  tf(0, 2) = -1.0;
  tf(0, 3) = lax + lox;
  tf(1, 0) = 0;
  tf(1, 1) = 1.0;
  tf(1, 2) = 0;
  tf(1, 3) = 0;
  tf(2, 0) = 1.0;
  tf(2, 1) = 0;
  tf(2, 2) = 0;
  tf(2, 3) = l2 + l3 + l4 + loz;
  tf(3, 0) = 0;
  tf(3, 1) = 0;
  tf(3, 2) = 0;
  tf(3, 3) = 1.0;
}

inline void forwardKin(Eigen::MatrixXd & xi_1, Eigen::MatrixXd & xi_2, Eigen::MatrixXd & xi_3, Eigen::MatrixXd & xi_4,
                       Eigen::MatrixXd & xi_5, Eigen::MatrixXd & tf, Eigen::Affine3d & cart_pos)
{
  Eigen::MatrixXd temp(4, 4);
  temp = xi_1 * xi_2 * xi_3 * xi_4 * xi_5 * tf;
  for (int i = 0; i < 4; i++)
  {
    for (int j = 0; j < 4; j++)
    {
      cart_pos(i, j) = temp(i, j);
    }
  }
}

inline void getCartPos(Eigen::VectorXd & j_pos, Eigen::Affine3d & cart_pos)
{
  /*
   MatrixXd xi_1(4, 4);
   MatrixXd xi_2(4, 4);
   MatrixXd xi_3(4, 4);
   MatrixXd xi_4(4, 4);
   MatrixXd xi_5(4, 4);
   MatrixXd tf(4, 4);
   getTwists(xi_1, xi_2, xi_3, xi_4, xi_5, tf, j_pos);
   forwardKin(xi_1, xi_2, xi_3, xi_4, xi_5, tf, cart_pos);
   */
  cart_pos(0,0) = sin(j_pos(1) + j_pos(2) + j_pos(3)) * cos(j_pos(1));
  cart_pos(0,1) = -cos(j_pos(4)) * sin(j_pos(0))
      - cos(j_pos(1) + j_pos(2) + j_pos(3)) * cos(j_pos(0)) * sin(j_pos(4));
  cart_pos(0,2) = sin(j_pos(0)) * sin(j_pos(4))
      - cos(j_pos(1) + j_pos(2) + j_pos(3)) * cos(j_pos(0)) * cos(j_pos(4));
  cart_pos(1,0) = sin(j_pos(1) + j_pos(2) + j_pos(3)) * sin(j_pos(0));
  cart_pos(1,1) = cos(j_pos(0)) * cos(j_pos(4))
      - cos(j_pos(1) + j_pos(2) + j_pos(3)) * sin(j_pos(0)) * sin(j_pos(4));
  cart_pos(1,2) = -cos(j_pos(0)) * sin(j_pos(4))
      - cos(j_pos(1) + j_pos(2) + j_pos(3)) * cos(j_pos(4)) * sin(j_pos(0));
  cart_pos(2,0) = cos(j_pos(1) + j_pos(2) + j_pos(3));
  cart_pos(2,1) = sin(j_pos(1) + j_pos(2) + j_pos(3)) * sin(j_pos(4));
  cart_pos(2,2) = sin(j_pos(1) + j_pos(2) + j_pos(3)) * cos(j_pos(4));
  cart_pos(0,3) = lax
      + cos(j_pos(0))
          * (lox + l3 * sin(j_pos(1) + j_pos(2)) + l2 * sin(j_pos(1)) + l4 * sin(j_pos(1) + j_pos(2) + j_pos(3)));
  cart_pos(1,3) = sin(j_pos(0))
      * (lox + l3 * sin(j_pos(1) + j_pos(2)) + l2 * sin(j_pos(1)) + l4 * sin(j_pos(1) + j_pos(2) + j_pos(3)));
  cart_pos(2,3) = loz + l3 * cos(j_pos(1) + j_pos(2)) + l2 * cos(j_pos(1))
      + l4 * cos(j_pos(1) + j_pos(2) + j_pos(3));
}

#endif
