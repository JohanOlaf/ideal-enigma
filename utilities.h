
#include <cstdio>
#include <memory>

#include "iostream"

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

#include <moveit_msgs/msg/robot_state.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>

#include <sensor_msgs/msg/joint_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>



int log(std::string data, const int verbose = 0);
std::vector<std::pair<std::string, std::pair<size_t, size_t>>> findNumberOfMatches(const std::vector<std::string>& keywords, const std::vector<std::string>& dataset);
int printTrajectory(moveit::planning_interface::MoveGroupInterface::Plan plan);
bool compareByIndex(const std::pair<std::string, std::pair<size_t, size_t>>& a, const std::pair<std::string, std::pair<size_t, size_t>>& b);
bool isGroupInPlan(const std::string& group, const moveit::planning_interface::MoveGroupInterface::Plan& plan);
int8_t isGroupInPlans(const std::string& group, const std::vector<moveit::planning_interface::MoveGroupInterface::Plan>& plans);
moveit::planning_interface::MoveGroupInterface::Plan expandTrajectory(moveit::planning_interface::MoveGroupInterface::Plan plan, size_t lengthOfTrajectory);
moveit::planning_interface::MoveGroupInterface::Plan newPlanFromStartState(moveit::planning_interface::MoveGroupInterface::Plan templatePlan, std::string name, size_t numberOfJoints, size_t startIndex);
int8_t findIndex(std::vector<std::string> subset, std::vector<std::string> set);
sensor_msgs::msg::JointState concatenateStates(const std::vector<moveit::planning_interface::MoveGroupInterface::Plan>& plans);
std::vector<moveit::planning_interface::MoveGroupInterface::Plan> findPlans(const std::vector<geometry_msgs::msg::Pose>& points, const std::string& group, const std::shared_ptr<rclcpp::Node> node);


std::vector<geometry_msgs::msg::Pose> createStraightPathPoints(std::vector<double> xyz_start, std::vector<double> xyz_stop, std::vector<double> xyzw_orientation, int num_points);