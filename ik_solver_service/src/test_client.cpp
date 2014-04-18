#include "ros/ros.h"
#include "ik_solver_service/SolveClosestIK.h"
#include "ik_solver_service/SolvePreferredPitchIK.h"
#include "ik_solver_service/SolvePreferredTypeIK.h"
#include "ik_solver_service/SolveFullyConstrainedIK.h"

#include <math.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "solve_ik_client");

  ros::NodeHandle n;

  // Example for computing closest IK solution
  ros::ServiceClient solve_closest_ik_client = n.serviceClient<ik_solver_service::SolveClosestIK>("solve_closest_ik");

  ik_solver_service::SolveClosestIK srv;

  srv.request.joint_angles[0] = 2.9496;
  srv.request.joint_angles[1] = 1.1345;
  srv.request.joint_angles[2] = -2.5482;
  srv.request.joint_angles[3] = 1.7890;
  srv.request.joint_angles[4] = 2.9234;

  srv.request.des_position[0] = 0.25;
  srv.request.des_position[1] = 0.0;
  srv.request.des_position[2] = 0.0;
  srv.request.des_normal[0] = 0.0;
  srv.request.des_normal[1] = 1.0;
  srv.request.des_normal[2] = 0.0;

  if (solve_closest_ik_client.call(srv))
  {
    for (int j = 0; j < 1; j++)
    { // In case multiple solutions should be printed
      for (int i = 0; i < 5; i++)
      {
        printf("%f\t", srv.response.joint_angles[i]);
      }
      printf("\nfeasible: %d arm_to_front: %d arm_bended_up: %d gripper_downwards: %d\n\n", srv.response.feasible,
             srv.response.arm_to_front, srv.response.arm_bended_up, srv.response.gripper_downwards);
    }
  }
  else
  {
    ROS_ERROR("Failed to call service solve_closest_ik");
    return 1;
  }

  // Example for computing preferred pitch IK solution
  ros::ServiceClient solve_preferred_pitch_ik_client = n.serviceClient<ik_solver_service::SolvePreferredPitchIK>(
      "solve_preferred_pitch_ik");

  ik_solver_service::SolvePreferredPitchIK pp_srv;

  pp_srv.request.preferred_pitch = 3.0 * M_PI / 4.0;
  pp_srv.request.des_position[0] = 0.25;
  pp_srv.request.des_position[1] = 0.0;
  pp_srv.request.des_position[2] = 0.0;
  pp_srv.request.des_normal[0] = 0.0;
  pp_srv.request.des_normal[1] = 1.0;
  pp_srv.request.des_normal[2] = 0.0;

  if (solve_preferred_pitch_ik_client.call(pp_srv))
  {
    for (int j = 0; j < 1; j++)
    { // In case multiple solutions should be printed
      for (int i = 0; i < 5; i++)
      {
        printf("%f\t", pp_srv.response.joint_angles[i]);
      }
      printf("\nfeasible: %d arm_to_front: %d arm_bended_up: %d gripper_downwards: %d\n\n", pp_srv.response.feasible,
             pp_srv.response.arm_to_front, pp_srv.response.arm_bended_up, pp_srv.response.gripper_downwards);
    }
  }
  else
  {
    ROS_ERROR("Failed to call service solve_preferred_pitch_ik");
    return 1;
  }

  // Example for computing preferred type IK solution
  ros::ServiceClient solve_preferred_type_ik_client = n.serviceClient<ik_solver_service::SolvePreferredTypeIK>(
      "solve_preferred_type_ik");

  ik_solver_service::SolvePreferredTypeIK pt_srv;

  pt_srv.request.arm_to_front = false;
  pt_srv.request.arm_bended_up = false;
  pt_srv.request.gripper_downwards = true;
  pt_srv.request.des_position[0] = 0.25;
  pt_srv.request.des_position[1] = 0.1;
  pt_srv.request.des_position[2] = 0.38;
  pt_srv.request.des_normal[0] = -0.3714;
  pt_srv.request.des_normal[1] = 0.9285;
  pt_srv.request.des_normal[2] = 0.0;

  if (solve_preferred_type_ik_client.call(pt_srv))
  {
    for (int j = 0; j < 1; j++)
    { // In case multiple solutions should be printed
      for (int i = 0; i < 5; i++)
      {
        printf("%f\t", pt_srv.response.joint_angles[i]);
      }
      printf("\nfeasible: %d arm_to_front: %d arm_bended_up: %d gripper_downwards: %d\n\n", pt_srv.response.feasible,
             pt_srv.response.arm_to_front, pt_srv.response.arm_bended_up, pt_srv.response.gripper_downwards);
    }
  }
  else
  {
    ROS_ERROR("Failed to call service solve_preferred_type_ik");
    return 1;
  }

  // Example for computing fully constrained IK solution
  ros::ServiceClient solve_fully_constrained_ik_client = n.serviceClient<ik_solver_service::SolveFullyConstrainedIK>(
      "solve_fully_constrained_ik");

  ik_solver_service::SolveFullyConstrainedIK fc_srv;

  fc_srv.request.id = 1;
  fc_srv.request.pitch = M_PI / 2.0;
  fc_srv.request.des_position[0] = 0.25;
  fc_srv.request.des_position[1] = 0.0;
  fc_srv.request.des_position[2] = 0.0;
  fc_srv.request.des_normal[0] = 1.0;
  fc_srv.request.des_normal[1] = 1.0;
  fc_srv.request.des_normal[2] = 0.0;

  if (solve_fully_constrained_ik_client.call(fc_srv))
  {
    for (int j = 0; j < 1; j++)
    { // In case multiple solutions should be printed
      for (int i = 0; i < 5; i++)
      {
        printf("%f\t", fc_srv.response.joint_angles[i]);
      }
      printf("\nfeasible: %d arm_to_front: %d arm_bended_up: %d gripper_downwards: %d\n\n", fc_srv.response.feasible,
             fc_srv.response.arm_to_front, fc_srv.response.arm_bended_up, fc_srv.response.gripper_downwards);
    }
  }
  else
  {
    ROS_ERROR("Failed to call service solve_fully_constrained_ik");
    return 1;
  }

  return 0;
}
