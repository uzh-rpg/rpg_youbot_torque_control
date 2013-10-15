#include "ros/ros.h"
#include "ros/console.h"
#include "math.h"
#include "brics_actuator/JointPositions.h"
#include <sstream>

#include <boost/units/systems/si/plane_angle.hpp>
#include <boost/units/io.hpp>

#include <boost/units/systems/angle/degrees.hpp>
#include <boost/units/conversion.hpp>

#include "control_msgs/FollowJointTrajectoryAction.h"
#include "trajectory_msgs/JointTrajectoryPoint.h"

#define DEG_TO_RAD(x) ((x) * M_PI / 180.0)
#define ALMOST_ONE 0.9999999

typedef struct
{
  unsigned int id;
  bool feasible;
  double joint_angles[5];
} ik_solution_t;

typedef double traj_pos_t[5];
typedef double j_off_t[5];

double normalize_angle(double angle){
  while (angle > M_PI)
    angle -= 2*M_PI;
  while (angle < -M_PI)
    angle += 2*M_PI;

  return angle;
}

void joint_offsets(j_off_t& offsets){
  offsets[0] = DEG_TO_RAD(169.0);
  offsets[1] = DEG_TO_RAD(65.0);
  offsets[2] = DEG_TO_RAD(-146.0);
  offsets[3] = DEG_TO_RAD(102.5);
  offsets[4] = DEG_TO_RAD(167.5);
}

void youbot_joint_angle_feasible(ik_solution_t& ik_solution){
  double min_angle[5];
  double max_angle[5];
  min_angle[0] = DEG_TO_RAD(-169.0);
  min_angle[1] = DEG_TO_RAD(-65.0);
  min_angle[2] = DEG_TO_RAD(-151.0);
  min_angle[3] = DEG_TO_RAD(-102.5);
  min_angle[4] = DEG_TO_RAD(-167.5);

  max_angle[0] = -min_angle[0];
  max_angle[1] = DEG_TO_RAD(90.0);
  max_angle[2] = DEG_TO_RAD(146.0);
  max_angle[3] = -min_angle[3];
  max_angle[4] = -min_angle[4];

  ik_solution.feasible = true;
  for (int i = 0; i < 5; i++){
    if (ik_solution.joint_angles[i] < min_angle[i] || ik_solution.joint_angles[i] > max_angle[i]){
      ik_solution.feasible = false;
      break;
    }
  }
}

void youbot_arm_only_ik(ik_solution_t& solution, unsigned int id, traj_pos_t traj_pos){
  solution.id = id;
  solution.feasible = false;
  
  double r_4, z_4, alpha, beta;
  double alpha_cos, beta_cos;
  double lox = 0.033;
  double loz = 0.147;
  double l_2 = 0.155;
  double l_3 = 0.135;
  double l_4 = 0.171;

  if (id == 1 || id == 2){
    solution.joint_angles[0] = atan2(traj_pos[1],traj_pos[0]);
    solution.joint_angles[4] = normalize_angle(traj_pos[3]);
    
    r_4 = sqrt(traj_pos[0]*traj_pos[0] + traj_pos[1]*traj_pos[1]) - lox - l_4*cos(traj_pos[4]);
    z_4 = traj_pos[2] - loz + l_4*sin(traj_pos[4]);

    alpha_cos = (l_2*l_2 + l_3*l_3 - r_4*r_4 - z_4*z_4)/(2*l_2*l_3);
    beta_cos = (r_4*r_4 + z_4*z_4 + l_2*l_2 - l_3*l_3)/(2*l_2*sqrt(r_4*r_4 + z_4*z_4));
    
    if (alpha_cos < -1 || beta_cos > 1){
	// Point not reachable -> No feasible solution possible
    }else{
      if (alpha_cos < -ALMOST_ONE){ // alpha cannot be close to 0 so only check if close to pi
	alpha = M_PI;
      }else{
	alpha = acos(alpha_cos);
      }
      if (beta_cos > ALMOST_ONE){ // beta cannot be close to pi so only check if close to 0
	beta = 0.0;
      }else{
	beta = acos(beta_cos);
      }

      if (id == 1){
	solution.joint_angles[1] = normalize_angle(atan2(r_4,z_4) - beta);
	solution.joint_angles[2] = normalize_angle(M_PI - alpha);
	solution.joint_angles[3] = normalize_angle(traj_pos[4] + M_PI/2 - solution.joint_angles[2] - solution.joint_angles[1]);
      }else{
	solution.joint_angles[1] = normalize_angle(atan2(r_4,z_4) + beta);
	solution.joint_angles[2] = normalize_angle(M_PI + alpha);
	solution.joint_angles[3] = normalize_angle(traj_pos[4] + M_PI/2 - solution.joint_angles[2] - solution.joint_angles[1]);
      }
      // check feasibility
      youbot_joint_angle_feasible(solution);
    }
  }else if (id == 3 || id == 4){
    solution.joint_angles[0] = normalize_angle(atan2(traj_pos[1],traj_pos[0]) + M_PI);
    solution.joint_angles[4] = normalize_angle(traj_pos[3] + M_PI);
    
    r_4 = sqrt(traj_pos[0]*traj_pos[0] + traj_pos[1]*traj_pos[1]) + lox - l_4*cos(traj_pos[4]);
    z_4 = traj_pos[2] - loz + l_4*sin(traj_pos[4]);

    alpha_cos = (l_2*l_2 + l_3*l_3 - r_4*r_4 - z_4*z_4)/(2*l_2*l_3);
    beta_cos = (r_4*r_4 + z_4*z_4 + l_2*l_2 - l_3*l_3)/(2*l_2*sqrt(r_4*r_4 + z_4*z_4));

    if (alpha_cos < -1 || beta_cos > 1){
	// Point not reachable -> No feasible solution possible
    }else{
      if (alpha_cos < -ALMOST_ONE){ // alpha cannot be close to 0 so only check if close to pi
	alpha = M_PI;
      }else{
	alpha = acos(alpha_cos);
      }
      if (beta_cos > ALMOST_ONE){ // beta cannot be close to pi so only check if close to 0
	beta = 0.0;
      }else{
	beta = acos(beta_cos);
      }

      if (id == 3){
        solution.joint_angles[1] = normalize_angle(-atan2(r_4,z_4) + beta);
        solution.joint_angles[2] = normalize_angle(-M_PI + alpha);
        solution.joint_angles[3] = normalize_angle(-traj_pos[4] - M_PI/2 - solution.joint_angles[2] - solution.joint_angles[1]);
      }else{
        solution.joint_angles[1] = normalize_angle(-atan2(r_4,z_4) - beta);
        solution.joint_angles[2] = normalize_angle(-M_PI - alpha);
        solution.joint_angles[3] = normalize_angle(-traj_pos[4] - M_PI/2 - solution.joint_angles[2] - solution.joint_angles[1]);
      }
      // check feasibility
      youbot_joint_angle_feasible(solution);
    }
  }else{
    // Error
    return;
  }

  if (!solution.feasible){ // Set values to zero if solution is not feasible eve though this solution cannot be sent
    solution.joint_angles[0] = 0;
    solution.joint_angles[1] = 0;
    solution.joint_angles[2] = 0;
    solution.joint_angles[3] = 0;
    solution.joint_angles[4] = 0;
  }

  // add offset
  j_off_t offsets;
  joint_offsets(offsets);
  solution.joint_angles[0] = -solution.joint_angles[0] + offsets[0]; // positive rotation of joint 1 is in negative axis direction
  solution.joint_angles[4] = -solution.joint_angles[4] + offsets[4]; // positive rotation of joint 5 is in negative axis direction
  for (int i = 1; i < 4; i++){
    solution.joint_angles[i] += offsets[i];
  }
}

void circle_trajectory_position(traj_pos_t& position, double time){
  double omega;
  double radius = 0.1;
  double center[3] = {0.3,0.0,0.3};
  omega = 2*M_PI/5.0;

  position[0] = center[0]; // x
  position[1] = radius*sin(omega*time) + center[1]; // y
  position[2] = radius*cos(omega*time) + center[2]; // z
  position[3] = 0.0; // Roll
  position[4] = 0.0; // Pitch
}

void youbot_generate_circle_trajectory(control_msgs::FollowJointTrajectoryActionGoal& trajectory_msg, double target_angle, double duration){
  int frequency = 50; // determine how many points per second are created
  double dt = 1.0/frequency;
  traj_pos_t cart_position;
  ik_solution_t ik_solution;

  for (double t = 0; t <= duration; t += dt){
    trajectory_msgs::JointTrajectoryPoint trajectory_point_message;
    trajectory_point_message.time_from_start = ros::Duration(t);

    circle_trajectory_position(cart_position, t);
    youbot_arm_only_ik(ik_solution, 1, cart_position);

    if (ik_solution.feasible){
      for (int i = 0; i < 5; i++){
        trajectory_point_message.positions.push_back(ik_solution.joint_angles[i]);
      }
      trajectory_msg.goal.trajectory.points.push_back(trajectory_point_message);
    }else{
      ROS_INFO("Solution to Inverse Kinematics is not feasible!");
    }
  }
  
  trajectory_msg.goal.trajectory.joint_names.push_back("arm_joint_1");
  trajectory_msg.goal.trajectory.joint_names.push_back("arm_joint_2");
  trajectory_msg.goal.trajectory.joint_names.push_back("arm_joint_3");
  trajectory_msg.goal.trajectory.joint_names.push_back("arm_joint_4");
  trajectory_msg.goal.trajectory.joint_names.push_back("arm_joint_5");
}

int main(int argc, char **argv){
  
  ros::init(argc, argv, "drive_circle");

  ros::NodeHandle n;

  // Initialize publisher
  ros::Publisher armPositionsPublisher;
  armPositionsPublisher = n.advertise<brics_actuator::JointPositions > ("arm_1/arm_controller/position_command", 1);
  
  ros::Publisher jointTrajectoryPublisher;
  jointTrajectoryPublisher = n.advertise<control_msgs::FollowJointTrajectoryActionGoal > ("arm_1/arm_controller/joint_trajectory_action/goal", 1);

  ros::Rate loop_rate(50);
  
  // Variables
  static const int numberOfArmJoints = 5;
  int i;

  brics_actuator::JointPositions command;
  std::vector <brics_actuator::JointValue> armJointPositions;
  armJointPositions.resize(numberOfArmJoints);
  std::stringstream jointName;

  traj_pos_t traj_position;
  ik_solution_t ik_solution;
  j_off_t offsets;

  // Drive Gripper to upright position
  ros::Duration(1.0).sleep(); // Wait until publisher is running properly
  joint_offsets(offsets);
  for (i = 0; i < numberOfArmJoints; ++i) {
      jointName.str("");
      jointName << "arm_joint_" << (i+1);
      armJointPositions[i].joint_uri = jointName.str();
      armJointPositions[i].value = offsets[i];
      armJointPositions[i].unit = boost::units::to_string(boost::units::si::radians);
  }
  command.positions = armJointPositions;
  armPositionsPublisher.publish(command);
  ros::Duration(3.0).sleep();

  // Drive Gripper to initial position and sleep for a while
  circle_trajectory_position(traj_position, 0);
  youbot_arm_only_ik(ik_solution, 1, traj_position);
  for (i = 0; i < numberOfArmJoints; ++i) {
    jointName.str("");
    jointName << "arm_joint_" << (i+1);
    armJointPositions[i].joint_uri = jointName.str();
    armJointPositions[i].value = ik_solution.joint_angles[i];
    armJointPositions[i].unit = boost::units::to_string(boost::units::si::radians);
  }
  
  if (ik_solution.feasible){
    command.positions = armJointPositions;
    armPositionsPublisher.publish(command);
  }
  ros::Duration(2.0).sleep();
  
  // Creating and publishing a FollowJointTrajectoryActionGoal message
  control_msgs::FollowJointTrajectoryActionGoal trajectory_msg;
  youbot_generate_circle_trajectory(trajectory_msg, 2*2*M_PI, 10.0);
  jointTrajectoryPublisher.publish(trajectory_msg);

}
