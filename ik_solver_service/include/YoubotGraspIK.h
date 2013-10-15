#ifndef __YOUBOT_GRASP_IK__
#define __YOUBOT_GRASP_IK__

#include <vector>
#include <algorithm>
#include <Eigen/Dense>

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>

#define ALMOST_ONE 0.99999
#define DEG_TO_RAD(x) ((x) * M_PI / 180.0)

struct joint_positions_solution_t {
	double joints[5];
	bool feasible;
	bool arm_to_front;
	bool arm_bended_up;
	bool gripper_downwards;
};

class YoubotGraspIK
{
public:
	YoubotGraspIK() { }
	
	static joint_positions_solution_t solve_closest_ik(joint_positions_solution_t current_joint_positions, 
		geometry_msgs::Point desired_position, geometry_msgs::Vector3 desired_normal);
	
	static joint_positions_solution_t solve_preferred_pitch_ik(double preferred_pitch, geometry_msgs::Point desired_position, 
		geometry_msgs::Vector3 desired_normal);

	static joint_positions_solution_t solve_preferred_type_ik(bool arm_to_front, bool arm_bended_up, bool gripper_downwards, geometry_msgs::Point desired_position, 
		geometry_msgs::Vector3 desired_normal);

	static joint_positions_solution_t solve_fully_constrained_ik(int id, double pitch, geometry_msgs::Point desired_position, 
		geometry_msgs::Vector3 desired_normal);

private:
	
	// find all solutions
	static std::vector<joint_positions_solution_t> get_all_ik_solutions(geometry_msgs::Point desired_position,
		geometry_msgs::Vector3 desired_normal);

	static void calculate_unique_pitch_solutions(std::vector<joint_positions_solution_t>& solutions, geometry_msgs::Point desired_position,
		geometry_msgs::Vector3 desired_normal);

	static void calculate_degenerative_solutions(std::vector<joint_positions_solution_t>& solutions, geometry_msgs::Point desired_position);
	static bool exists_unique_pitch_solution(geometry_msgs::Point desired_position, geometry_msgs::Vector3 desired_normal);

	// finding solution with minimum joint angle difference
	static joint_positions_solution_t take_closest_solution(joint_positions_solution_t current_joint_positions,
		std::vector<joint_positions_solution_t> solutions);
	static double get_max_joint_difference(joint_positions_solution_t solution_a, joint_positions_solution_t solution_b);

	// finding solution with closest gripper pitch
	static joint_positions_solution_t compute_exact_pitch_solution(double preferred_pitch, geometry_msgs::Point desired_position, 
		geometry_msgs::Vector3 desired_normal);
	static joint_positions_solution_t take_closest_pitch_solution(double preferred_pitch, std::vector<joint_positions_solution_t> solutions);
	static double get_pitch_difference(double preferred_pitch, joint_positions_solution_t solution);
 
	// finding solution of desired type
	static joint_positions_solution_t take_closest_type_solution(bool arm_to_front, bool arm_bended_up, bool gripper_downwards, std::vector<joint_positions_solution_t> solutions);

	// finding unique pitch solution
	static double normalize_angle(double angle);
	static double sign(double value);
	static bool check_single_solution_feasability(joint_positions_solution_t single_solution);
	static joint_positions_solution_t compute_single_ik_solution(Eigen::Vector3d des_position, double des_roll, double des_pitch, int id);

	// Geometric Constants
	static const double lox = 0.033;
	static const double loz = 0.1472;
	static const double l_2 = 0.155;
	static const double l_3 = 0.135;
	static const double l_4 = 0.218; // Including sensor carrier which is 1.5mm thick

};

#endif
