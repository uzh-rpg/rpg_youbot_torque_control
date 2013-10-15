#include "ros/ros.h"
#include "ros/console.h"
#include "std_srvs/Empty.h"

#include "geometry_msgs/PoseStamped.h"
#include "sensor_msgs/JointState.h"

#include "math.h"
#include "brics_actuator/JointPositions.h"
#include <sstream>

#include <boost/units/systems/si/plane_angle.hpp>
#include <boost/units/io.hpp>

#include <boost/units/systems/angle/degrees.hpp>
#include <boost/units/conversion.hpp>

#define DEG_TO_RAD(x) ((x) * M_PI / 180.0)
#define ALMOST_ONE 0.9999999

/////////////////////////////////////
// VICON stuff

geometry_msgs::PoseStamped last_base_pose;
geometry_msgs::PoseStamped last_gripper_pose;
sensor_msgs::JointState last_joint_states;

void base_callback(const geometry_msgs::PoseStampedConstPtr& pose) {
  last_base_pose = *pose;
}

void gripper_callback(const geometry_msgs::PoseStampedConstPtr& pose) {
  last_gripper_pose = *pose;
}

void joint_states_callback(const sensor_msgs::JointStateConstPtr& joint_states) {
  last_joint_states = *joint_states;
}

/////////////////////////////////////


const double joint_offsets[5] = {DEG_TO_RAD(169.0), DEG_TO_RAD(65.0), DEG_TO_RAD(-146.0), DEG_TO_RAD(102.5), DEG_TO_RAD(167.5)};
const double joint_min_angles[5] = {DEG_TO_RAD(-169.0), DEG_TO_RAD(-65.0), DEG_TO_RAD(-151.0), DEG_TO_RAD(-102.5), DEG_TO_RAD(-167.5)};
const double joint_max_angles[5] = {DEG_TO_RAD(169.0), DEG_TO_RAD(90.0), DEG_TO_RAD(146.0), DEG_TO_RAD(102.5), DEG_TO_RAD(167.5)};
const double lox = 0.033;
const double loz = 0.147;
const double l_2 = 0.155;
const double l_3 = 0.135;
const double l_4 = 0.171;

struct grasp_pose_t {
  double x, y, z;
  double theta;
};

struct joint_positions_t {
  double joints[5];
  bool feasible;
};

double normalize_angle(double angle){
  while (angle > M_PI)
    angle -= 2*M_PI;
  while (angle < -M_PI)
    angle += 2*M_PI;

  return angle;
}

joint_positions_t arm_grasp_ik(grasp_pose_t desired_pose, int solution_id) {

  joint_positions_t solution;
  solution.feasible = true;
  double r = sqrt(desired_pose.x * desired_pose.x + desired_pose.y*desired_pose.y);
  double dz = desired_pose.z + l_4 - loz;
  double r2;
  
  // Calculate joint 0, and check if desired pose is reachable
  if (1 == solution_id) {
    r2 = r - lox;
    if (r2 <= l_2 + l_3) {
      solution.joints[0] = normalize_angle(atan2(desired_pose.y, desired_pose.x));

    } else {
      ROS_WARN("Solution 1 not feasible, since arm is too short.");
      solution.feasible = false;
      return solution;
    }
  } else {
    r2 = r + lox;
    if (r2 <= l_2 + l_3) {
      solution.joints[0] = normalize_angle(atan2(desired_pose.y, desired_pose.x) + M_PI);

    } else {
      ROS_WARN("Solution 2 not feasible, since arm is too short.");
      solution.feasible = false;
      return solution;
    }
  }
  
  // Calculate joints 1, 2, and 3
  double l23 = sqrt( r2*r2 + dz*dz ); 
  if (l23 > l_2 + l_3) {
    ROS_WARN("l23 > l2+l3: Not feasible!");
    solution.feasible = false;
    return solution;
  }
  double phi2 = acos((l_2*l_2 + l23*l23 - l_3*l_3) / (2*l_2*l23));
  ROS_INFO("l23: %f, phi2: %f", l23, phi2);
  if (1 == solution_id) {
    solution.joints[1] = M_PI/2.0 - (atan2(dz, r2) + phi2);
  } else {
    solution.joints[1] = atan2(dz, r2) + phi2 - M_PI/2.0;
  }
  
  double phi3 = M_PI - acos((l_2*l_2 + l_3*l_3 - l23*l23) / (2*l_2*l_3)); 
  ROS_INFO("phi3: %f", phi3);
  if (1 == solution_id) {
    solution.joints[2] = phi3;
  } else {
    solution.joints[2] = -phi3;
  }
  
  double phi4 = acos((l_3*l_3 + l23*l23 - l_2*l_2) / (2*l_3*l23)); 
  ROS_INFO("phi4: %f", phi4);
  if (1 == solution_id) {
    solution.joints[3] = M_PI - (asin(r2/l23) + phi4);
  } else {
    solution.joints[3] = asin(r2/l23) + phi4 - M_PI;
  }
  
  // Calculate joint 4
  double phi1 = normalize_angle(atan2(desired_pose.y, desired_pose.x));
  solution.joints[4] = normalize_angle(desired_pose.theta - phi1);
  
  if (solution.joints[4] < joint_min_angles[4] || solution.joints[4] > joint_max_angles[4]) {
    solution.joints[4] = normalize_angle( solution.joints[4] + M_PI);
    ROS_INFO("Added PI to joint 4");
  }
  
  
  // joint 0 turns in the other direction...
  solution.joints[0] = normalize_angle(-solution.joints[0]);
  
  // check feasibility, then add offset
  for (int i=0; i<5; i++) {
  
    if (solution.joints[i] < joint_min_angles[i] || solution.joints[i] > joint_max_angles[i]) {
      solution.feasible = false;
      ROS_WARN("Joint %i is not feasible: %f (Limits: %f / %f)", i, solution.joints[i], joint_min_angles[i], joint_max_angles[i]);
    }
    
    solution.joints[i] = solution.joints[i] + joint_offsets[i];
  }
  
  
  return solution;
}

joint_positions_t arm_grasp_ik(grasp_pose_t desired_pose) {
  // try solution 1
  joint_positions_t solution = arm_grasp_ik(desired_pose, 2);
  
  // otherwise try solution 2
  if (!solution.feasible) {
    ROS_INFO("Solution 1 not feasible, trying solution 2.");
    solution = arm_grasp_ik(desired_pose, 1);
  }
    
  if (solution.feasible)
    ROS_INFO("Joint positions: %f, %f, %f, %f, %f", solution.joints[0], solution.joints[1], solution.joints[2], solution.joints[3], solution.joints[4]);
  else 
    ROS_ERROR("Solution 2 is not feasible either.");
  
  return solution;
}


int main(int argc, char **argv){
  
  ros::init(argc, argv, "drive_circle");

  ros::NodeHandle n;
  ros::Subscriber base_sub;
  ros::Subscriber gripper_sub;
  ros::Subscriber joint_states_sub;

  // Initialize publisher
  ros::Publisher armPositionsPublisher;
  armPositionsPublisher = n.advertise<brics_actuator::JointPositions > ("arm_1/arm_controller/position_command", 1);

  ros::Rate loop_rate(0.15);
  
  // Variables
  static const int numberOfArmJoints = 5;


  brics_actuator::JointPositions command;
  std::vector <brics_actuator::JointValue> armJointPositions;
  armJointPositions.resize(numberOfArmJoints);
  std::stringstream jointName;


  // Drive Gripper to upright position and sleep for a while
  ros::Duration(1.0).sleep(); // Wait until publisher is running properly
  for (int i = 0; i < numberOfArmJoints; ++i) {
      jointName.str("");
      jointName << "arm_joint_" << (i+1);
      armJointPositions[i].joint_uri = jointName.str();
      armJointPositions[i].value = joint_offsets[i];
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
  
  
  // Use VICON
  bool use_vicon = true;
  if (use_vicon) {
    base_sub = n.subscribe("/Base_Trackable", 1, base_callback);
    gripper_sub = n.subscribe("/Gripper_Trackable", 1, gripper_callback);
    ros::Duration(0.01).sleep();

    while (base_sub.getNumPublishers() == 0) {
      ROS_ERROR("Not connected to publisher... (no connections: %i) waiting 1s", base_sub.getNumPublishers());
      ros::Duration(1.0).sleep();
    }

    while (gripper_sub.getNumPublishers() == 0) {
      ROS_ERROR("Not connected to publisher... (no connections: %i) waiting 1s", gripper_sub.getNumPublishers());
      ros::Duration(1.0).sleep();
    }

    ROS_INFO("Subscribed to VICON messages");
  }

  joint_states_sub = n.subscribe("/joint_states", 1, joint_states_callback);


  // Move on trajectory
  std::vector< std::vector<double> > points;
  
  static const double p1arr[] = { 0.20,  0.06, 0.1, 0.0};
  static const double p2arr[] = { 0.20, -0.06, 0.1, 0.0};
  static const double p3arr[] = { 0.25, -0.06, 0.1, 0.0};
  static const double p4arr[] = { 0.25,  0.06, 0.1, 0.0};
  
  std::vector<double> p1 (p1arr, p1arr + sizeof(p1arr) / sizeof(p1arr[0]) );
  std::vector<double> p2 (p2arr, p2arr + sizeof(p2arr) / sizeof(p2arr[0]) );
  std::vector<double> p3 (p3arr, p3arr + sizeof(p3arr) / sizeof(p3arr[0]) );
  std::vector<double> p4 (p4arr, p4arr + sizeof(p4arr) / sizeof(p4arr[0]) );

  points.push_back(p1);
  points.push_back(p2);
  points.push_back(p3);
  points.push_back(p4);
    
  int current_point = 0;
  
  // vicon variables
  double dx, dy, dz;
  double last_dx=0.0, last_dy=0.0, last_dz=0.0;
  
  // Execute trajectory  
  while (ros::ok())
  {   
    int index =  current_point % points.size();
    grasp_pose_t grasp_pose;
    grasp_pose.x = points[index][0];
    grasp_pose.y = points[index][1];
    grasp_pose.z = points[index][2];
    grasp_pose.theta = points[index][3];

    joint_positions_t ik_solution = arm_grasp_ik(grasp_pose);


    for (int i = 0; i < numberOfArmJoints; ++i) {
      jointName.str("");
      jointName << "arm_joint_" << (i+1);
      armJointPositions[i].joint_uri = jointName.str();
      armJointPositions[i].value = ik_solution.joints[i];
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
    ros::spinOnce();

    ROS_INFO("should be at point %i", current_point % (int) points.size());
    
    dx = last_gripper_pose.pose.position.x - last_base_pose.pose.position.x;
    dy = last_gripper_pose.pose.position.y - last_base_pose.pose.position.y;
    dz = last_gripper_pose.pose.position.z - last_base_pose.pose.position.z;
    
    
    if (current_point > 0) {
      double dist_x = dx - last_dx;
      double dist_y = dy - last_dy;
      double dist_z = dz - last_dz;
      
      double des_x = points[current_point % (int) points.size()][0] - points[(current_point - 1) % (int) points.size()][0];
      double des_y = points[current_point % (int) points.size()][1] - points[(current_point - 1) % (int) points.size()][1];
      double des_z = points[current_point % (int) points.size()][2] - points[(current_point - 1) % (int) points.size()][2];
      
      double des_dist = sqrt(des_x*des_x + des_y*des_y + des_z*des_z);
      double meas_dist = sqrt(dist_x*dist_x + dist_y*dist_y + dist_z*dist_z);

      ROS_ERROR("way was: %f, should be %f, error: %f", meas_dist, des_dist, meas_dist - des_dist);
      ROS_ERROR("command was: %2.5f %2.5f %2.5f %2.5f %2.5f", ik_solution.joints[0], ik_solution.joints[1], ik_solution.joints[2], ik_solution.joints[3], ik_solution.joints[4]);
      ROS_ERROR(" joints are: %2.5f %2.5f %2.5f %2.5f %2.5f", last_joint_states.position[0], last_joint_states.position[1], last_joint_states.position[2], last_joint_states.position[3], last_joint_states.position[4]);
      ROS_ERROR("differences: %2.5f %2.5f %2.5f %2.5f %2.5f", ik_solution.joints[0]-last_joint_states.position[0], ik_solution.joints[1]-last_joint_states.position[1], ik_solution.joints[2]-last_joint_states.position[2], ik_solution.joints[3]-last_joint_states.position[3], ik_solution.joints[4]-last_joint_states.position[4]);
    }
    
    last_dx = dx;
    last_dy = dy;
    last_dz = dz;
    
    // next point
    current_point++;
  }

}
