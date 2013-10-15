#include "ros/ros.h"
#include "ros/console.h"
#include "std_srvs/Empty.h"


#include <stdlib.h>
#include <fstream>

#include "geometry_msgs/PoseStamped.h"
#include "sensor_msgs/JointState.h"

#include <math.h>
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

geometry_msgs::PoseStamped last_gripper_pose;
sensor_msgs::JointState last_joint_states;

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

int main(int argc, char **argv){
  
  ros::init(argc, argv, "drive_circle");

  ros::NodeHandle n;
  ros::Subscriber base_sub;
  ros::Subscriber gripper_sub;
  ros::Subscriber joint_states_sub;

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
    gripper_sub = n.subscribe("/Gripper_Trackable", 1, gripper_callback);
    ros::Duration(0.01).sleep();

//    while (gripper_sub.getNumPublishers() == 0) {
//      ROS_ERROR("Not connected to publisher... (no connections: %i) waiting 1s", gripper_sub.getNumPublishers());
//      ros::Duration(1.0).sleep();
//    }

    ROS_INFO("Subscribed to VICON messages");
  }

  joint_states_sub = n.subscribe("/joint_states", 1, joint_states_callback);

  const int num_points = 13*11;
  double points[num_points][5] = {
		  {-150.0, -20.0, -20.0, -20.0, 0.0},
		  {-150.0, -20.0, -20.0, -10.0, 0.0},
		  {-150.0, -20.0, -10.0, -10.0, 0.0},
		  {-150.0, -10.0, -10.0, -10.0, 0.0},
		  {-150.0, -10.0, -10.0,   0.0, 0.0},
		  {-150.0, -10.0,   0.0,   0.0, 0.0},
		  {-150.0,   0.0,   0.0,   0.0, 0.0},
		  {-150.0,  10.0,   0.0,   0.0, 0.0},
		  {-150.0,  10.0,  10.0,   0.0, 0.0},
		  {-150.0,  10.0,  10.0,  10.0, 0.0},
		  {-150.0,  20.0,  10.0,  10.0, 0.0},
		  {-150.0,  20.0,  20.0,  10.0, 0.0},
		  {-150.0,  20.0,  20.0,  20.0, 0.0},

		  {-120.0,  20.0,  20.0,  20.0, 0.0},
		  {-120.0,  20.0,  20.0,  10.0, 0.0},
		  {-120.0,  20.0,  10.0,  10.0, 0.0},
		  {-120.0,  10.0,  10.0,  10.0, 0.0},
		  {-120.0,  10.0,  10.0,   0.0, 0.0},
		  {-120.0,  10.0,   0.0,   0.0, 0.0},
		  {-120.0,   0.0,   0.0,   0.0, 0.0},
		  {-120.0, -10.0,   0.0,   0.0, 0.0},
		  {-120.0, -10.0, -10.0,   0.0, 0.0},
		  {-120.0, -10.0, -10.0, -10.0, 0.0},
		  {-120.0, -20.0, -10.0, -10.0, 0.0},
		  {-120.0, -20.0, -20.0, -10.0, 0.0},
		  {-120.0, -20.0, -20.0, -20.0, 0.0},

		  {-90.0, -20.0, -20.0, -20.0, 0.0},
		  {-90.0, -20.0, -20.0, -10.0, 0.0},
		  {-90.0, -20.0, -10.0, -10.0, 0.0},
		  {-90.0, -10.0, -10.0, -10.0, 0.0},
		  {-90.0, -10.0, -10.0,   0.0, 0.0},
		  {-90.0, -10.0,   0.0,   0.0, 0.0},
		  {-90.0,   0.0,   0.0,   0.0, 0.0},
		  {-90.0,  10.0,   0.0,   0.0, 0.0},
		  {-90.0,  10.0,  10.0,   0.0, 0.0},
		  {-90.0,  10.0,  10.0,  10.0, 0.0},
		  {-90.0,  20.0,  10.0,  10.0, 0.0},
		  {-90.0,  20.0,  20.0,  10.0, 0.0},
		  {-90.0,  20.0,  20.0,  20.0, 0.0},

		  {-60.0,  20.0,  20.0,  20.0, 0.0},
		  {-60.0,  20.0,  20.0,  10.0, 0.0},
		  {-60.0,  20.0,  10.0,  10.0, 0.0},
		  {-60.0,  10.0,  10.0,  10.0, 0.0},
		  {-60.0,  10.0,  10.0,   0.0, 0.0},
		  {-60.0,  10.0,   0.0,   0.0, 0.0},
		  {-60.0,   0.0,   0.0,   0.0, 0.0},
		  {-60.0, -10.0,   0.0,   0.0, 0.0},
		  {-60.0, -10.0, -10.0,   0.0, 0.0},
		  {-60.0, -10.0, -10.0, -10.0, 0.0},
		  {-60.0, -20.0, -10.0, -10.0, 0.0},
		  {-60.0, -20.0, -20.0, -10.0, 0.0},
		  {-60.0, -20.0, -20.0, -20.0, 0.0},

		  {-30.0, -20.0, -20.0, -20.0, 0.0},
		  {-30.0, -20.0, -20.0, -10.0, 0.0},
		  {-30.0, -20.0, -10.0, -10.0, 0.0},
		  {-30.0, -10.0, -10.0, -10.0, 0.0},
		  {-30.0, -10.0, -10.0,   0.0, 0.0},
		  {-30.0, -10.0,   0.0,   0.0, 0.0},
		  {-30.0,   0.0,   0.0,   0.0, 0.0},
		  {-30.0,  10.0,   0.0,   0.0, 0.0},
		  {-30.0,  10.0,  10.0,   0.0, 0.0},
		  {-30.0,  10.0,  10.0,  10.0, 0.0},
		  {-30.0,  20.0,  10.0,  10.0, 0.0},
		  {-30.0,  20.0,  20.0,  10.0, 0.0},
		  {-30.0,  20.0,  20.0,  20.0, 0.0},

		  {  0.0,  20.0,  20.0,  20.0, 0.0},
		  {  0.0,  20.0,  20.0,  10.0, 0.0},
		  {  0.0,  20.0,  10.0,  10.0, 0.0},
		  {  0.0,  10.0,  10.0,  10.0, 0.0},
		  {  0.0,  10.0,  10.0,   0.0, 0.0},
		  {  0.0,  10.0,   0.0,   0.0, 0.0},
		  {  0.0,   0.0,   0.0,   0.0, 0.0},
		  {  0.0, -10.0,   0.0,   0.0, 0.0},
		  {  0.0, -10.0, -10.0,   0.0, 0.0},
		  {  0.0, -10.0, -10.0, -10.0, 0.0},
		  {  0.0, -20.0, -10.0, -10.0, 0.0},
		  {  0.0, -20.0, -20.0, -10.0, 0.0},
		  {  0.0, -20.0, -20.0, -20.0, 0.0},

		  { 30.0, -20.0, -20.0, -20.0, 0.0},
		  { 30.0, -20.0, -20.0, -10.0, 0.0},
		  { 30.0, -20.0, -10.0, -10.0, 0.0},
		  { 30.0, -10.0, -10.0, -10.0, 0.0},
		  { 30.0, -10.0, -10.0,   0.0, 0.0},
		  { 30.0, -10.0,   0.0,   0.0, 0.0},
		  { 30.0,   0.0,   0.0,   0.0, 0.0},
		  { 30.0,  10.0,   0.0,   0.0, 0.0},
		  { 30.0,  10.0,  10.0,   0.0, 0.0},
		  { 30.0,  10.0,  10.0,  10.0, 0.0},
		  { 30.0,  20.0,  10.0,  10.0, 0.0},
		  { 30.0,  20.0,  20.0,  10.0, 0.0},
		  { 30.0,  20.0,  20.0,  20.0, 0.0},

		  { 60.0,  20.0,  20.0,  20.0, 0.0},
		  { 60.0,  20.0,  20.0,  10.0, 0.0},
		  { 60.0,  20.0,  10.0,  10.0, 0.0},
		  { 60.0,  10.0,  10.0,  10.0, 0.0},
		  { 60.0,  10.0,  10.0,   0.0, 0.0},
		  { 60.0,  10.0,   0.0,   0.0, 0.0},
		  { 60.0,   0.0,   0.0,   0.0, 0.0},
		  { 60.0, -10.0,   0.0,   0.0, 0.0},
		  { 60.0, -10.0, -10.0,   0.0, 0.0},
		  { 60.0, -10.0, -10.0, -10.0, 0.0},
		  { 60.0, -20.0, -10.0, -10.0, 0.0},
		  { 60.0, -20.0, -20.0, -10.0, 0.0},
		  { 60.0, -20.0, -20.0, -20.0, 0.0},

		  { 90.0, -20.0, -20.0, -20.0, 0.0},
		  { 90.0, -20.0, -20.0, -10.0, 0.0},
		  { 90.0, -20.0, -10.0, -10.0, 0.0},
		  { 90.0, -10.0, -10.0, -10.0, 0.0},
		  { 90.0, -10.0, -10.0,   0.0, 0.0},
		  { 90.0, -10.0,   0.0,   0.0, 0.0},
		  { 90.0,   0.0,   0.0,   0.0, 0.0},
		  { 90.0,  10.0,   0.0,   0.0, 0.0},
		  { 90.0,  10.0,  10.0,   0.0, 0.0},
		  { 90.0,  10.0,  10.0,  10.0, 0.0},
		  { 90.0,  20.0,  10.0,  10.0, 0.0},
		  { 90.0,  20.0,  20.0,  10.0, 0.0},
		  { 90.0,  20.0,  20.0,  20.0, 0.0},

		  { 120.0,  20.0,  20.0,  20.0, 0.0},
		  { 120.0,  20.0,  20.0,  10.0, 0.0},
		  { 120.0,  20.0,  10.0,  10.0, 0.0},
		  { 120.0,  10.0,  10.0,  10.0, 0.0},
		  { 120.0,  10.0,  10.0,   0.0, 0.0},
		  { 120.0,  10.0,   0.0,   0.0, 0.0},
		  { 120.0,   0.0,   0.0,   0.0, 0.0},
		  { 120.0, -10.0,   0.0,   0.0, 0.0},
		  { 120.0, -10.0, -10.0,   0.0, 0.0},
		  { 120.0, -10.0, -10.0, -10.0, 0.0},
		  { 120.0, -20.0, -10.0, -10.0, 0.0},
		  { 120.0, -20.0, -20.0, -10.0, 0.0},
		  { 120.0, -20.0, -20.0, -20.0, 0.0},

		  { 150.0, -20.0, -20.0, -20.0, 0.0},
		  { 150.0, -20.0, -20.0, -10.0, 0.0},
		  { 150.0, -20.0, -10.0, -10.0, 0.0},
		  { 150.0, -10.0, -10.0, -10.0, 0.0},
		  { 150.0, -10.0, -10.0,   0.0, 0.0},
		  { 150.0, -10.0,   0.0,   0.0, 0.0},
		  { 150.0,   0.0,   0.0,   0.0, 0.0},
		  { 150.0,  10.0,   0.0,   0.0, 0.0},
		  { 150.0,  10.0,  10.0,   0.0, 0.0},
		  { 150.0,  10.0,  10.0,  10.0, 0.0},
		  { 150.0,  20.0,  10.0,  10.0, 0.0},
		  { 150.0,  20.0,  20.0,  10.0, 0.0},
		  { 150.0,  20.0,  20.0,  20.0, 0.0}
  };

  // convert all points to radian and add offset
  for (int i=0; i<num_points; i++) {
	  for (int j=0; j<5; j++) {
		  points[i][j] = DEG_TO_RAD(points[i][j]) + joint_offsets[j];
	  }
  }

  int point_id = 0;

  // file
  FILE * pFile;
  std::string homedir = getenv("HOME");
  pFile = fopen ((homedir + "/offset.log").c_str(),"w");

  // Execute trajectory  
  while (ros::ok())
  {

	// send to joints
    for (int i = 0; i < numberOfArmJoints; ++i) {
      jointName.str("");
      jointName << "arm_joint_" << (i+1);
      armJointPositions[i].joint_uri = jointName.str();
      armJointPositions[i].value = points[point_id][i];
      armJointPositions[i].unit = boost::units::to_string(boost::units::si::radians);
    }
    
    command.positions = armJointPositions;
    armPositionsPublisher.publish(command);
    
    // wait for execution
    ros::spinOnce();
    loop_rate.sleep();

    // wait more the first time
    if (point_id%13 == 0) {
      ROS_ERROR("wait longer");
      loop_rate.sleep();
      loop_rate.sleep();
    }
    
    // read current optitrack message
    ros::spinOnce();

    // output message
    ROS_ERROR("j0, j1, j2, j3, j4, x, y, z: %4.10f, %4.10f, %4.10f, %4.10f, %4.10f, %4.10f, %4.10f, %4.10f",
    		points[point_id][0], points[point_id][1], points[point_id][2], points[point_id][3], points[point_id][4],
    		last_gripper_pose.pose.position.x, last_gripper_pose.pose.position.y, last_gripper_pose.pose.position.z);

    fprintf(pFile, "%4.10f, %4.10f, %4.10f, %4.10f, %4.10f, %4.10f, %4.10f, %4.10f\n",
    		points[point_id][0], points[point_id][1], points[point_id][2], points[point_id][3], points[point_id][4],
    		last_gripper_pose.pose.position.x, last_gripper_pose.pose.position.y, last_gripper_pose.pose.position.z);

    point_id++;
    if (point_id >= num_points) {
    	ROS_INFO("All points sent, shutting down.");
    	fclose (pFile);
    	ros::spinOnce();
    	ros::shutdown();
    }
  }

}
