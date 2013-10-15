#include "ros/ros.h"
#include "ros/console.h"
#include "std_srvs/Empty.h"

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

// REAL VALUES
const double joint_offsets[5] = {DEG_TO_RAD(169.0), DEG_TO_RAD(65.0), DEG_TO_RAD(-146.0), DEG_TO_RAD(102.5), DEG_TO_RAD(167.5)};
// TEST VALUES
//const double joint_offsets[5] = {DEG_TO_RAD(169.0), DEG_TO_RAD(65.0+10.0), DEG_TO_RAD(-146.0+10.0), DEG_TO_RAD(102.5+10.0), DEG_TO_RAD(167.5)};

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

  ros::Rate loop_rate(1);
  
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

  // REAL VALUES
  double calibration_min = DEG_TO_RAD(-5.0);
  double calibration_max = DEG_TO_RAD(+5.0);
  double calibration_step =  2.0*M_PI / 4000.0;


  // TEST VALUES
//  double calibration_min = DEG_TO_RAD(-40.0);
//  double calibration_max = DEG_TO_RAD(+40.0);
//  double calibration_step = DEG_TO_RAD(1.0);


  int num_calibration_steps = ceil( (calibration_max - calibration_min) / calibration_step );
  
  double joints_calibrated_offsets[5];
  joints_calibrated_offsets[0] = 0.0;
  joints_calibrated_offsets[1] = 0.0;
  joints_calibrated_offsets[2] = 0.0;
  joints_calibrated_offsets[3] = 0.0;
  joints_calibrated_offsets[4] = 0.0;

  int calibration_joint_id = 3;
  int calibration_step_id = 0;

  double joint_pos[5];

  double max_z = 0.0;
  double best_calibration = 0.0;

  // Execute trajectory  
  while (ros::ok())
  {
	// set joints as offsets
	for (int i=0; i<5; i++) {
		joint_pos[i] = joint_offsets[i];
	}

	// add calibration parameters
	double current_offset_angle = calibration_min + calibration_step_id * calibration_step;

	for (int i=0; i<5; i++) {
		joint_pos[i] += joints_calibrated_offsets[i];
	}

	joint_pos[calibration_joint_id] += current_offset_angle;

	// compensate upper joint
	if (calibration_joint_id == 2) {
		joint_pos[3] -= current_offset_angle;
	}

	// compensate upper joint
	if (calibration_joint_id == 1) {
		joint_pos[2] -= current_offset_angle;
	}

	//ROS_INFO("joints: %2.7f %2.7f %2.7f %2.7f %2.7f", joint_pos[0], joint_pos[1], joint_pos[2], joint_pos[3], joint_pos[4]);

	// send to joints
    for (int i = 0; i < numberOfArmJoints; ++i) {
      jointName.str("");
      jointName << "arm_joint_" << (i+1);
      armJointPositions[i].joint_uri = jointName.str();
      armJointPositions[i].value = joint_pos[i];
      armJointPositions[i].unit = boost::units::to_string(boost::units::si::radians);
    }
    
    command.positions = armJointPositions;
    armPositionsPublisher.publish(command);
    
    // wait for execution
    ros::spinOnce();
    loop_rate.sleep();
    ros::spinOnce();


    ROS_ERROR("Joint %i with additional offset of %2.7f gives z %2.7f", calibration_joint_id,
    		current_offset_angle, last_gripper_pose.pose.position.z);
    
    // remember best value
    if (last_gripper_pose.pose.position.z > max_z) {
    	best_calibration = current_offset_angle;
	max_z = last_gripper_pose.pose.position.z;
    }


    // goto next step
    calibration_step_id++;
    if (calibration_step_id >= num_calibration_steps) {
    	calibration_step_id = 0;


    	// add offset for next calibration of joint

    	joints_calibrated_offsets[calibration_joint_id] = best_calibration;
    	if (calibration_joint_id == 2) {
    		joints_calibrated_offsets[3] -= best_calibration;
    	}
    	if (calibration_joint_id == 1) {
		joints_calibrated_offsets[2] -= best_calibration;
	}

    	ROS_ERROR("---------------------------------");
    	ROS_ERROR("Calibrated joint %i", calibration_joint_id);
    	ROS_ERROR("Best value for additional joint offset was: %2.7f", best_calibration);
    	ROS_ERROR("Maximum height of gripper was:              %2.7f", max_z);
    	ROS_ERROR("---------------------------------");

    	max_z = 0.0; // reset
    	calibration_joint_id--;
    }
    
    // done if all joints calibrated
    if (calibration_joint_id <= 0) {
	ROS_ERROR("original offset:   %2.7f %2.7f %2.7f %2.7f %2.7f", joint_offsets[0], joint_offsets[1], joint_offsets[2], joint_offsets[3], joint_offsets[4]);
	ROS_ERROR("additional offset: %2.7f %2.7f %2.7f %2.7f %2.7f", joints_calibrated_offsets[0], joints_calibrated_offsets[1], joints_calibrated_offsets[2], joints_calibrated_offsets[3], joints_calibrated_offsets[4]);

	double final_offset[5];
	for (int k=0; k<5; k++) 
		final_offset[k] = joint_offsets[k] + joints_calibrated_offsets[k];

	ROS_ERROR("optimized offset:  %2.7f %2.7f %2.7f %2.7f %2.7f", final_offset[0], final_offset[1], final_offset[2], final_offset[3], final_offset[4]);
    	ROS_ERROR("---------------------------------");
    	ROS_ERROR("Done with all calibrations. Shutdown.");

      // Drive Gripper to upright position and sleep for a while
      ros::Duration(1.0).sleep(); // Wait until publisher is running properly
      for (int i = 0; i < numberOfArmJoints; ++i) {
        jointName.str("");
        jointName << "arm_joint_" << (i+1);
        armJointPositions[i].joint_uri = jointName.str();
        armJointPositions[i].value = joint_offsets[i];
	if (i == 1)
           armJointPositions[i].value += joints_calibrated_offsets[2];
	if (i == 2)
           armJointPositions[i].value += joints_calibrated_offsets[1];
	if (i == 3)
           armJointPositions[i].value += joints_calibrated_offsets[0];
        armJointPositions[i].unit = boost::units::to_string(boost::units::si::radians);
      }
      command.positions = armJointPositions;
      armPositionsPublisher.publish(command);
    ros::spinOnce();

	ros::Duration(3.0).sleep();
    	ros::shutdown();
    }
  }

}
