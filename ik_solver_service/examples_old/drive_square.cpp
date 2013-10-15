#include "ros/ros.h"
#include "ros/console.h"
#include "std_srvs/Empty.h"
#include "math.h"
#include "brics_actuator/JointPositions.h"
#include <sstream>

#include <boost/units/systems/si/plane_angle.hpp>
#include <boost/units/io.hpp>

#include <boost/units/systems/angle/degrees.hpp>
#include <boost/units/conversion.hpp>

#define DEG_TO_RAD(x) ((x) * M_PI / 180.0)
#define ALMOST_ONE 0.9999999

typedef struct
{
  unsigned int id;
  bool feasible;
  double joint_angles[5];
} ik_solution_t;

typedef double traj_pos_t[5];
typedef double grasp_pos_t[4];
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

void youbot_arm_grasp_ik(ik_solution_t& solution, unsigned int id, grasp_pos_t grasp_pos) {
  traj_pos_t traj_pos;
  traj_pos[0] = grasp_pos[0];
  traj_pos[1] = grasp_pos[1];
  traj_pos[2] = grasp_pos[2];
  traj_pos[3] = M_PI/2.0;
  traj_pos[4] = 0.0;
  youbot_arm_only_ik(solution, id, traj_pos);
  solution.joint_angles[4] = solution.joint_angles[4] + normalize_angle( solution.joint_angles[0] + grasp_pos[3] );
}

void youbot_arm_grasp_ik(ik_solution_t& solution, grasp_pos_t grasp_pos) {
  // try id=1
  youbot_arm_grasp_ik(solution, 1, grasp_pos);
  
  // try id=3 if not feasible
  if (!solution.feasible) {
    youbot_arm_grasp_ik(solution, 3, grasp_pos);
  }
  
}

int main(int argc, char **argv){
  
  ros::init(argc, argv, "drive_circle");

  ros::NodeHandle n;

  // Initialize publisher
  ros::Publisher armPositionsPublisher;
  armPositionsPublisher = n.advertise<brics_actuator::JointPositions > ("arm_1/arm_controller/position_command", 1);

  ros::Rate loop_rate(0.5);
  
  // Variables
  static const int numberOfArmJoints = 5;


  brics_actuator::JointPositions command;
  std::vector <brics_actuator::JointValue> armJointPositions;
  armJointPositions.resize(numberOfArmJoints);
  std::stringstream jointName;

  traj_pos_t traj_position;
  ik_solution_t ik_solution;
  j_off_t offsets;

  // Drive Gripper to upright position and sleep for a while
  ros::Duration(1.0).sleep(); // Wait until publisher is running properly
  joint_offsets(offsets);
  for (int i = 0; i < numberOfArmJoints; ++i) {
      jointName.str("");
      jointName << "arm_joint_" << (i+1);
      armJointPositions[i].joint_uri = jointName.str();
      armJointPositions[i].value = offsets[i];
      armJointPositions[i].unit = boost::units::to_string(boost::units::si::radians);
  }
  command.positions = armJointPositions;
  armPositionsPublisher.publish(command);
  ros::Duration(3.0).sleep();

  
  // Stop base?
  bool stop_base = true;
  if (stop_base) {
    ros::ServiceClient client = n.serviceClient< std_srvs::Empty::Request, std_srvs::Empty::Response > ("base/switchOffMotors");
    std_srvs::Empty::Request req;
    std_srvs::Empty::Response res;
    if (client.call(req, res)) {
      ROS_INFO("TURNED OFF BASE");
    } else {
      ROS_ERROR("Failed to connect to switch off base (service call failed).");
    }
  }
  ros::Duration(3.0).sleep();


  // Move on trajectory
  std::vector< std::vector<double> > points;
  
  static const double p1arr[] = { 0.22,  0.05, 0.1, 0.0};
  static const double p2arr[] = { 0.22, -0.05, 0.1, 0.0};
  static const double p3arr[] = { 0.28, -0.05, 0.1, 0.0};
  static const double p4arr[] = { 0.28,  0.05, 0.1, 0.0};
  
  std::vector<double> p1 (p1arr, p1arr + sizeof(p1arr) / sizeof(p1arr[0]) );
  std::vector<double> p2 (p2arr, p2arr + sizeof(p2arr) / sizeof(p2arr[0]) );
  std::vector<double> p3 (p3arr, p3arr + sizeof(p3arr) / sizeof(p3arr[0]) );
  std::vector<double> p4 (p4arr, p4arr + sizeof(p4arr) / sizeof(p4arr[0]) );

  points.push_back(p1);
  points.push_back(p2);
  points.push_back(p3);
  points.push_back(p4);
    
  int current_point = 0;
  
  // Execute trajectory
  grasp_pos_t grasp_pos;
  while (ros::ok())
  {    
    for (int i=0; i<4; i++)
    	grasp_pos[i] = points[current_point % points.size()][i];
    
    youbot_arm_grasp_ik(ik_solution, grasp_pos);


    for (int i = 0; i < numberOfArmJoints; ++i) {
      jointName.str("");
      jointName << "arm_joint_" << (i+1);
      armJointPositions[i].joint_uri = jointName.str();
      armJointPositions[i].value = ik_solution.joint_angles[i];
      armJointPositions[i].unit = boost::units::to_string(boost::units::si::radians);
    }
    

    if (ik_solution.feasible){
      command.positions = armJointPositions;
      armPositionsPublisher.publish(command);
    } else {
      ROS_ERROR("Joint positions not feasible!");
    }
    
    ros::spinOnce();

    loop_rate.sleep();
    
    ROS_INFO("should be at point %i", current_point % (int) points.size());
    current_point++;
  }

}
