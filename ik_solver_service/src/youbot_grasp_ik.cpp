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

#include "ik_solver_service/youbot_grasp_ik.h"

namespace ik_solver_service {

using namespace rpg_youbot_common;

// Compute the solution which is closest to the given configuration
joint_positions_solution_t YoubotGraspIK::solveClosestIK(joint_positions_solution_t current_joint_positions,
                                                         geometry_msgs::Point desired_position,
                                                         geometry_msgs::Vector3 desired_normal)
{

  std::vector<joint_positions_solution_t> solutions;
  solutions = getAllIKSolutions(desired_position, desired_normal);

  joint_positions_solution_t final_solution;
  if (0 == solutions.size())
  {
    final_solution.feasible = false;
  }
  else if (1 == solutions.size())
  {
    return solutions[0];
  }
  else
  {
    final_solution = takeClosestSolution(current_joint_positions, solutions);
  }

  return final_solution;
}

// Compute a solution, preferably with the given gripper pitch
joint_positions_solution_t YoubotGraspIK::solvePreferredPitchIK(double preferred_pitch,
                                                                geometry_msgs::Point desired_position,
                                                                geometry_msgs::Vector3 desired_normal)
{

  joint_positions_solution_t single_solution = computeExactPitchSolution(preferred_pitch, desired_position,
                                                                         desired_normal);
  if (single_solution.feasible)
  {
    return single_solution;
  }
  else
  {
    std::vector<joint_positions_solution_t> solutions;
    solutions = getAllIKSolutions(desired_position, desired_normal);

    joint_positions_solution_t final_solution;
    if (0 == solutions.size())
    {
      final_solution.feasible = false;
    }
    else if (1 == solutions.size())
    {
      return solutions[0];
    }
    else
    {
      final_solution = takeClosestPitchSolution(preferred_pitch, solutions);
    }

    return final_solution;
  }
}

// Compute a solution, preferably of the given type
joint_positions_solution_t YoubotGraspIK::solvePreferredTypeIK(bool arm_to_front, bool arm_bended_up,
                                                               bool gripper_downwards,
                                                               geometry_msgs::Point desired_position,
                                                               geometry_msgs::Vector3 desired_normal)
{

  std::vector<joint_positions_solution_t> solutions;
  solutions = getAllIKSolutions(desired_position, desired_normal);

  joint_positions_solution_t final_solution;
  if (0 == solutions.size())
  {
    final_solution.feasible = false;
  }
  else if (1 == solutions.size())
  {
    return solutions[0];
  }
  else
  {
    final_solution = takeClosestTypeSolution(arm_to_front, arm_bended_up, gripper_downwards, solutions);
  }

  return final_solution;
}

// Compute a solution of given solution id, which is fully constrained even in the degenerative case
joint_positions_solution_t YoubotGraspIK::solveFullyConstrainedIK(int id, double pitch,
                                                                  geometry_msgs::Point desired_position,
                                                                  geometry_msgs::Vector3 desired_normal)
{

  joint_positions_solution_t fully_constrained_solution;

  // desired position as Eigen type
  Eigen::Vector3d des_position(desired_position.x, desired_position.y, desired_position.z);

  // desired normal as Eigen type
  Eigen::Vector3d des_normal(desired_normal.x, desired_normal.y, desired_normal.z);
  des_normal.normalize();

  // Angle of the desired position wrt the x-axis around the z-axis
  double phi = atan2(des_position[1], des_position[0]);

  // x-axis of the Gripper
  Eigen::Vector3d x_gripper(cos(phi) * cos(pitch), sin(phi) * cos(pitch), -sin(pitch));
  if (fabs(x_gripper.dot(des_normal)) > 0.0001)
  {
    // Since x_gripper and des_normal are not perpendicular, no feasible solution with this pitch can exist
    fully_constrained_solution.feasible = false;
    return fully_constrained_solution;
  }

  // y-axis of intermediate frame which is rotatet around z
  Eigen::Vector3d y_inter(-sin(phi), cos(phi), 0);

  double roll = 0.0;
  if (fabs(y_inter.dot(des_normal)) <= ALMOST_ONE)
  {
    // Roll of the gripper making its y-axis coincide with the desired normal given the preferred pitch
    double roll = acos(y_inter.dot(des_normal) / (y_inter.norm() * des_normal.norm()));

    if (des_normal.dot(x_gripper.cross(y_inter)) < 0)
    {
      roll = normalizeAngle(-roll);
    }
  }

  // Compute IK for a single solution
  fully_constrained_solution = computeSingleIKSolution(des_position, roll, pitch, id);
  if (fully_constrained_solution.feasible)
  {
    // Sett attributes of solution type
    if (id == 1)
    {
      fully_constrained_solution.arm_to_front = true;
      fully_constrained_solution.arm_bended_up = true;
    }
    else if (id == 2)
    {
      fully_constrained_solution.arm_to_front = true;
      fully_constrained_solution.arm_bended_up = false;
    }
    else if (id == 3)
    {
      fully_constrained_solution.arm_to_front = false;
      fully_constrained_solution.arm_bended_up = true;
    }
    else if (id == 4)
    {
      fully_constrained_solution.arm_to_front = false;
      fully_constrained_solution.arm_bended_up = false;
    }
    if (pitch >= 0)
    {
      fully_constrained_solution.gripper_downwards = true;
    }
    else
    {
      fully_constrained_solution.gripper_downwards = false;
    }
  }

  return fully_constrained_solution;
}

// Choose solution with minimum joint difference maximum
joint_positions_solution_t YoubotGraspIK::takeClosestSolution(joint_positions_solution_t current_joint_positions,
                                                              std::vector<joint_positions_solution_t> solutions)
{
  int best_solution_index = 0;
  double best_solution_max_difference = getMaxJointDifference(current_joint_positions, solutions[0]);
  for (int i = 0; i < (int)solutions.size(); i++)
  {
    double current_solution_max_difference = getMaxJointDifference(current_joint_positions, solutions[i]);
    if (current_solution_max_difference < best_solution_max_difference)
    {
      best_solution_max_difference = current_solution_max_difference;
      best_solution_index = i;
    }
  }
  return solutions[best_solution_index];
}

// Choose solution with minimum difference to the desired gripper pitch
joint_positions_solution_t YoubotGraspIK::takeClosestPitchSolution(double preferred_pitch,
                                                                   std::vector<joint_positions_solution_t> solutions)
{
  int best_solution_index = 0;
  double best_solution_difference = getPitchDifference(preferred_pitch, solutions[0]);
  for (int i = 0; i < (int)solutions.size(); i++)
  {
    double current_solution_difference = getPitchDifference(preferred_pitch, solutions[i]);
    if (current_solution_difference < best_solution_difference)
    {
      best_solution_difference = current_solution_difference;
      best_solution_index = i;
    }
  }
  return solutions[best_solution_index];
}

// Compute the configuration which is closest to the desired solution type. Priorities are 1. gripper_downwards 2. arm_to_front 3. arm_bended_up
joint_positions_solution_t YoubotGraspIK::takeClosestTypeSolution(bool arm_to_front, bool arm_bended_up,
                                                                  bool gripper_downwards,
                                                                  std::vector<joint_positions_solution_t> solutions)
{

  std::vector<int> correct_p1_idx; // indexes of solutions with correct gripper downwards attribute
  std::vector<int> correct_p12_idx; // indexes of solutions with correct gripper downwards and arm to front attributes
  std::vector<int> correct_p2_idx; // indexes of solutions with correct arm to front attribute
  std::vector<int> final_idx; // indexes of the final solution candidates

  for (int i = 0; i < (int)solutions.size(); i++)
  {
    if (solutions[i].gripper_downwards == gripper_downwards)
    {
      correct_p1_idx.push_back(i);
    }
  }
  if (correct_p1_idx.size() > 0)
  {
    for (int i = 0; i < (int)correct_p1_idx.size(); i++)
    {
      if (solutions[correct_p1_idx[i]].arm_to_front == arm_to_front)
      {
        correct_p12_idx.push_back(correct_p1_idx[i]);
      }
    }
    if (correct_p12_idx.size() > 0)
    {
      for (int i = 0; i < (int)correct_p12_idx.size(); i++)
      {
        if (solutions[correct_p12_idx[i]].arm_bended_up == arm_bended_up)
        {
          final_idx.push_back(correct_p12_idx[i]);
        }
      }
      if (final_idx.size() == 0)
      {
        final_idx = correct_p12_idx;
      }
    }
    else
    {
      for (int i = 0; i < (int)correct_p1_idx.size(); i++)
      {
        if (solutions[correct_p1_idx[i]].arm_bended_up == arm_bended_up)
        {
          final_idx.push_back(correct_p1_idx[i]);
        }
      }
      if (final_idx.size() == 0)
      {
        final_idx = correct_p1_idx;
      }
    }
  }
  else
  {
    for (int i = 0; i < (int)solutions.size(); i++)
    {
      if (solutions[i].arm_to_front == arm_to_front)
      {
        correct_p2_idx.push_back(i);
      }
    }
    if (correct_p2_idx.size() > 0)
    {
      for (int i = 0; i < (int)correct_p2_idx.size(); i++)
      {
        if (solutions[correct_p2_idx[i]].arm_bended_up == arm_bended_up)
        {
          final_idx.push_back(correct_p2_idx[i]);
        }
      }
      if (final_idx.size() == 0)
      {
        final_idx = correct_p2_idx;
      }
    }
    else
    {
      for (int i = 0; i < (int)solutions.size(); i++)
      {
        if (solutions[i].arm_bended_up == arm_bended_up)
        {
          final_idx.push_back(i);
        }
      }
      if (final_idx.size() == 0)
      {
        final_idx.push_back(0);
      }
    }
  }

  return solutions[final_idx[0]];
}

// Compute the maximum of all the joint differences between two configurations
double YoubotGraspIK::getMaxJointDifference(joint_positions_solution_t solution_a,
                                            joint_positions_solution_t solution_b)
{
  double max_joint_difference = 0.0;
  for (uint i = 0; i < 5; i++)
    max_joint_difference = std::max(max_joint_difference, fabs(solution_a.joints[i] - solution_b.joints[i]));
  return max_joint_difference;
}

// Compute the difference of the preferred gripper pitch and the gripper pitch of a given configuration
double YoubotGraspIK::getPitchDifference(double preferred_pitch, joint_positions_solution_t solution)
{
  const double joint_offsets[5] = {deg2Rad(169.0), deg2Rad(65.0), deg2Rad(-146.0), deg2Rad(102.5),
                                   deg2Rad(167.5)};
  double max_pitch_difference = fabs(
      preferred_pitch + M_PI / 2.0 - (solution.joints[1] - joint_offsets[1]) - (solution.joints[2] - joint_offsets[2])
          - (solution.joints[3] - joint_offsets[3]));
  return max_pitch_difference;
}

// Compute either the unique pitch solutions if it exists or all solutions of the degenerative case otherwise
std::vector<joint_positions_solution_t> YoubotGraspIK::getAllIKSolutions(geometry_msgs::Point desired_position,
                                                                         geometry_msgs::Vector3 desired_normal)
{
  std::vector<joint_positions_solution_t> solutions;

  // Detect whether unique solution exists or not
  if (existsUniquePitchSolution(desired_position, desired_normal))
  {
    calculateUniquePitchSolutions(solutions, desired_position, desired_normal);
  }
  else
  {
    calculateDegenerativeSolutions(solutions, desired_position);
  }

  return solutions;
}

// Check if unique pitch solutions exist
bool YoubotGraspIK::existsUniquePitchSolution(geometry_msgs::Point desired_position,
                                              geometry_msgs::Vector3 desired_normal)
{

  // r and z direction vector
  Eigen::Vector3d z_direction(0, 0, 1);
  Eigen::Vector3d r_direction(desired_position.x, desired_position.y, desired_position.z);
  r_direction.normalize();

  // normal of plane spaned by z direction and vector in direction of desired position
  Eigen::Vector3d rz_normal = z_direction.cross(r_direction);

  // desired normal as Eigen type
  Eigen::Vector3d des_normal(desired_normal.x, desired_normal.y, desired_normal.z);

  double cos_angle = rz_normal.dot(des_normal) / (rz_normal.norm() * des_normal.norm());

  // decide based on tolerance
  if (cos_angle < -ALMOST_ONE || cos_angle > ALMOST_ONE)
    return false;
  else
    return true;
}

// Normalize an angle to be between -pi and pi
double YoubotGraspIK::normalizeAngle(double angle)
{
  // Normalize the angle to between -pi and pi
  while (angle > M_PI)
    angle -= 2 * M_PI;
  while (angle < -M_PI)
    angle += 2 * M_PI;

  return angle;
}

// Simple signum function
double YoubotGraspIK::sign(double value)
{
  // Need to return 1.0 if value is exatly zero!
  if (value >= 0)
    return 1.0;
  else
    return -1.0;
}

// Check wether a given configuration of the arm is feasible. i.e. within joint limits
bool YoubotGraspIK::checkSingleSolutionFeasability(joint_positions_solution_t single_solution)
{

  const double joint_min_angles[5] = {deg2Rad(-169.0), deg2Rad(-65.0), deg2Rad(-151.0), deg2Rad(-102.5),
                                      deg2Rad(-167.5)};
  const double joint_max_angles[5] = {deg2Rad(169.0), deg2Rad(90.0), deg2Rad(146.0), deg2Rad(102.5),
                                      deg2Rad(167.5)};

  bool feasibility = true;
  for (int i = 0; i < 5; i++)
  {
    if (single_solution.joints[i] < joint_min_angles[i] || single_solution.joints[i] > joint_max_angles[i])
    {
      feasibility = false;
    }
  }
  return feasibility;
}

// Compute one single arm configuration
joint_positions_solution_t YoubotGraspIK::computeSingleIKSolution(Eigen::Vector3d des_position, double des_roll,
                                                                  double des_pitch, int id)
{
  joint_positions_solution_t single_solution;

  if (id == 1 || id == 2)
  {
    single_solution.joints[0] = atan2(des_position[1], des_position[0]);
    single_solution.joints[4] = normalizeAngle(des_roll);

    double r_4 = sqrt(des_position[0] * des_position[0] + des_position[1] * des_position[1]) - lox_
        - l_4_ * cos(des_pitch);
    double z_4 = des_position[2] - loz_ + l_4_ * sin(des_pitch);

    double alpha_cos = (l_2_ * l_2_ + l_3_ * l_3_ - r_4 * r_4 - z_4 * z_4) / (2 * l_2_ * l_3_);
    double beta_cos = (r_4 * r_4 + z_4 * z_4 + l_2_ * l_2_ - l_3_ * l_3_) / (2 * l_2_ * sqrt(r_4 * r_4 + z_4 * z_4));
    double alpha, beta;

    if (alpha_cos < -1.0 || beta_cos > 1.0)
    {
      // Point not reachable -> No feasible solution with this id can be found
      single_solution.feasible = false;
    }
    else
    {
      if (alpha_cos < -ALMOST_ONE)
      { // alpha cannot be close to 0 so only check if close to pi
        alpha = M_PI;
      }
      else
      {
        alpha = acos(alpha_cos);
      }
      if (beta_cos > ALMOST_ONE)
      { // beta cannot be close to pi so only check if close to 0
        beta = 0.0;
      }
      else
      {
        beta = acos(beta_cos);
      }

      if (id == 1)
      {
        single_solution.joints[1] = normalizeAngle(atan2(r_4, z_4) - beta);
        single_solution.joints[2] = normalizeAngle(M_PI - alpha);
        single_solution.joints[3] = normalizeAngle(
            des_pitch + M_PI / 2 - single_solution.joints[2] - single_solution.joints[1]);
      }
      else
      {
        single_solution.joints[1] = normalizeAngle(atan2(r_4, z_4) + beta);
        single_solution.joints[2] = normalizeAngle(M_PI + alpha);
        single_solution.joints[3] = normalizeAngle(
            des_pitch + M_PI / 2 - single_solution.joints[2] - single_solution.joints[1]);
      }
      // check feasibility
      single_solution.feasible = checkSingleSolutionFeasability(single_solution);
    }
  }
  else if (id == 3 || id == 4)
  {
    single_solution.joints[0] = normalizeAngle(atan2(des_position[1], des_position[0]) + M_PI);
    single_solution.joints[4] = normalizeAngle(des_roll + M_PI);

    double r_4 = sqrt(des_position[0] * des_position[0] + des_position[1] * des_position[1]) + lox_
        - l_4_ * cos(des_pitch);
    double z_4 = des_position[2] - loz_ + l_4_ * sin(des_pitch);

    double alpha_cos = (l_2_ * l_2_ + l_3_ * l_3_ - r_4 * r_4 - z_4 * z_4) / (2 * l_2_ * l_3_);
    double beta_cos = (r_4 * r_4 + z_4 * z_4 + l_2_ * l_2_ - l_3_ * l_3_) / (2 * l_2_ * sqrt(r_4 * r_4 + z_4 * z_4));
    double alpha, beta;

    if (alpha_cos < -1.0 || beta_cos > 1.0)
    {
      // Point not reachable -> No feasible solution with this id can be found
      single_solution.feasible = false;
    }
    else
    {
      if (alpha_cos < -ALMOST_ONE)
      { // alpha cannot be close to 0 so only check if close to pi
        alpha = M_PI;
      }
      else
      {
        alpha = acos(alpha_cos);
      }
      if (beta_cos > ALMOST_ONE)
      { // beta cannot be close to pi so only check if close to 0
        beta = 0.0;
      }
      else
      {
        beta = acos(beta_cos);
      }

      if (id == 3)
      {
        single_solution.joints[1] = normalizeAngle(-atan2(r_4, z_4) + beta);
        single_solution.joints[2] = normalizeAngle(-M_PI + alpha);
        single_solution.joints[3] = normalizeAngle(
            -des_pitch - M_PI / 2 - single_solution.joints[2] - single_solution.joints[1]);
      }
      else
      {
        single_solution.joints[1] = normalizeAngle(-atan2(r_4, z_4) - beta);
        single_solution.joints[2] = normalizeAngle(-M_PI - alpha);
        single_solution.joints[3] = normalizeAngle(
            -des_pitch - M_PI / 2 - single_solution.joints[2] - single_solution.joints[1]);
      }
      // check feasibility
      single_solution.feasible = checkSingleSolutionFeasability(single_solution);
    }
  }
  // Add joint angle offsets
  const double joint_offsets[5] = {deg2Rad(169.0), deg2Rad(65.0), deg2Rad(-146.0), deg2Rad(102.5),
                                   deg2Rad(167.5)};

  single_solution.joints[0] = -single_solution.joints[0] + joint_offsets[0]; // positive rotation of joint 1 is in negative coordinate axis direction
  single_solution.joints[4] = -single_solution.joints[4] + joint_offsets[4]; // positive rotation of joint 5 is in negative coordinate axis direction
  for (int i = 1; i < 4; i++)
  {
    single_solution.joints[i] += joint_offsets[i];
  }

  return single_solution;
}

// Compute the configuration with exactly the desired gripper pitch in the degenerative case
joint_positions_solution_t YoubotGraspIK::computeExactPitchSolution(double preferred_pitch,
                                                                    geometry_msgs::Point desired_position,
                                                                    geometry_msgs::Vector3 desired_normal)
{

  joint_positions_solution_t exact_pitch_solution;

  // desired position as Eigen type
  Eigen::Vector3d des_position(desired_position.x, desired_position.y, desired_position.z);

  // desired normal as Eigen type
  Eigen::Vector3d des_normal(desired_normal.x, desired_normal.y, desired_normal.z);
  des_normal.normalize();

  // Angle of the desired position wrt the x-axis around the z-axis
  double phi = atan2(des_position[1], des_position[0]);

  // x-axis of the Gripper
  Eigen::Vector3d x_gripper(cos(phi) * cos(preferred_pitch), sin(phi) * cos(preferred_pitch), -sin(preferred_pitch));
  if (fabs(x_gripper.dot(des_normal)) > 0.0001)
  {
    // Since x_gripper and des_normal are not perpendicular, no feasible solution with this pitch can exist
    exact_pitch_solution.feasible = false;
    return exact_pitch_solution;
  }

  // y-axis of intermediate frame which is rotatet around z
  Eigen::Vector3d y_inter(-sin(phi), cos(phi), 0);

  double roll = 0.0;
  if (fabs(y_inter.dot(des_normal)) <= ALMOST_ONE) // M_PI is not possible anyways
  {
    // Roll of the gripper making its y-axis coincide with the desired normal given the preferred pitch
    roll = acos(y_inter.dot(des_normal) / (y_inter.norm() * des_normal.norm()));

    if (des_normal.dot(x_gripper.cross(y_inter)) < 0)
    {
      roll = normalizeAngle(-roll);
    }
  }

  // Compute all the 8 possible solutions and return the first feasible one we find
  for (int k = 0; k < 2; k++)
  {
    for (int solution_id = 1; solution_id <= 4; solution_id++)
    {
      // Compute IK for a single solution
      exact_pitch_solution = computeSingleIKSolution(des_position, roll, preferred_pitch, solution_id);
      if (exact_pitch_solution.feasible)
      {
        // Sett attributes of solution type
        if (solution_id == 1)
        {
          exact_pitch_solution.arm_to_front = true;
          exact_pitch_solution.arm_bended_up = true;
        }
        else if (solution_id == 2)
        {
          exact_pitch_solution.arm_to_front = true;
          exact_pitch_solution.arm_bended_up = false;
        }
        else if (solution_id == 3)
        {
          exact_pitch_solution.arm_to_front = false;
          exact_pitch_solution.arm_bended_up = true;
        }
        else if (solution_id == 4)
        {
          exact_pitch_solution.arm_to_front = false;
          exact_pitch_solution.arm_bended_up = false;
        }
        if (preferred_pitch >= 0)
        {
          exact_pitch_solution.gripper_downwards = true;
        }
        else
        {
          exact_pitch_solution.gripper_downwards = false;
        }
        // Returning this solution since it is feasible
        return exact_pitch_solution;
      }
    }
    roll = normalizeAngle(roll + M_PI);
  }
  // If we reach this point no feasible solution is found
  exact_pitch_solution.feasible = false;
  return exact_pitch_solution;
}

// Compute all the so called unique pitch solutions
void YoubotGraspIK::calculateUniquePitchSolutions(std::vector<joint_positions_solution_t>& solutions,
                                                  geometry_msgs::Point desired_position,
                                                  geometry_msgs::Vector3 desired_normal)
{

  // desired position as Eigen type
  Eigen::Vector3d des_position(desired_position.x, desired_position.y, desired_position.z);

  // desired normal as Eigen type
  Eigen::Vector3d des_normal(desired_normal.x, desired_normal.y, desired_normal.z);
  des_normal.normalize();

  // Angle of the desired position wrt the x-axis around the z-axis
  double phi = atan2(des_position[1], des_position[0]);

  // Some vectors required for the computation
  Eigen::Vector3d x_inter(cos(phi), sin(phi), 0); // x-axis of intermediate frame which is rotatet around z
  Eigen::Vector3d y_inter(-sin(phi), cos(phi), 0); // y-axis of intermediate frame which is rotatet around z
  Eigen::Vector3d z(0, 0, 1);
  Eigen::Vector3d x_des = y_inter.cross(des_normal); // Desired x direction of the gripper
  x_des.normalize();

  // Roll and pitch of the gripper making its y-axis coincide with the desired normal
  double pitch = -sign(x_des.dot(z)) * acos(x_inter.dot(x_des) / (x_inter.norm() * x_des.norm()));
  double roll = acos(y_inter.dot(des_normal) / (y_inter.norm() * des_normal.norm()));
  double temp_roll = roll;
  double temp_pitch = pitch;

  // Compute all the 16 possible solutions and push the feasible ones back
  for (int k = 0; k < 2; k++)
  {
    for (int j = 0; j < 2; j++)
    {
      for (int solution_id = 1; solution_id <= 4; solution_id++)
      {
        // Compute IK for a single solution
        joint_positions_solution_t single_solution = computeSingleIKSolution(des_position, temp_roll, temp_pitch,
                                                                             solution_id);
        if (single_solution.feasible)
        {
          // Sett attributes of solution type
          if (solution_id == 1)
          {
            single_solution.arm_to_front = true;
            single_solution.arm_bended_up = true;
          }
          else if (solution_id == 2)
          {
            single_solution.arm_to_front = true;
            single_solution.arm_bended_up = false;
          }
          else if (solution_id == 3)
          {
            single_solution.arm_to_front = false;
            single_solution.arm_bended_up = true;
          }
          else if (solution_id == 4)
          {
            single_solution.arm_to_front = false;
            single_solution.arm_bended_up = false;
          }
          if (temp_pitch >= 0)
          {
            single_solution.gripper_downwards = true;
          }
          else
          {
            single_solution.gripper_downwards = false;
          }
          // Pushing back single solution
          solutions.push_back(single_solution);
        }
      }
      temp_pitch = normalizeAngle(pitch + M_PI);
      temp_roll = normalizeAngle(-roll);
    }
    roll = normalizeAngle(roll + M_PI);
  }

}

// Compute all solutions of the degenerative case with a 5 degrees step of the gripper pitch
void YoubotGraspIK::calculateDegenerativeSolutions(std::vector<joint_positions_solution_t>& solutions,
                                                   geometry_msgs::Point desired_position)
{
  // desired position as Eigen type
  Eigen::Vector3d des_position(desired_position.x, desired_position.y, desired_position.z);

  // loop over all pitches
  double step_size = deg2Rad(5.0);
  for (double pitch = -M_PI; pitch <= M_PI; pitch += step_size)
  {
    // loop over all four solution types
    for (int solution_id = 1; solution_id <= 4; solution_id++)
    {
      joint_positions_solution_t single_solution;
      if (solution_id < 3)
      { // Destinction required because of how compute_single_ik_solution works
        single_solution = computeSingleIKSolution(des_position, 0.0, pitch, solution_id);
      }
      else
      {
        single_solution = computeSingleIKSolution(des_position, M_PI, pitch, solution_id);
      }
      if (single_solution.feasible)
      {
        // Sett attributes of solution type
        if (solution_id == 1)
        {
          single_solution.arm_to_front = true;
          single_solution.arm_bended_up = true;
        }
        else if (solution_id == 2)
        {
          single_solution.arm_to_front = true;
          single_solution.arm_bended_up = false;
        }
        else if (solution_id == 3)
        {
          single_solution.arm_to_front = false;
          single_solution.arm_bended_up = true;
        }
        else if (solution_id == 4)
        {
          single_solution.arm_to_front = false;
          single_solution.arm_bended_up = false;
        }
        if (pitch >= 0)
        {
          single_solution.gripper_downwards = true;
        }
        else
        {
          single_solution.gripper_downwards = false;
        }
        // Pushing back single solution
        solutions.push_back(single_solution);
      }
    }
  }
}

} // namespace ik_solver_service
