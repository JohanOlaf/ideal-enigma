#include "utilities.h"

const std::string NODENAME = "hello_moveit";

int main(int argc, char ** argv)
{
  log("Initializing ROS node", 1);
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(NODENAME, rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
  log("Creating ros logger", 1);
  auto const logger = rclcpp::get_logger(NODENAME);

  using moveit::planning_interface::MoveGroupInterface;

  //-- lager punkter --
  std::vector<double> startPoint = {-0.2, 1.2, 1.2};
  std::vector<double> stopPoint = {0.4, 1.2, 1.2};
  std::vector<double> orientationVector = {1, 0, 0, 0};
  int numPoints = 10;
  std::vector<geometry_msgs::msg::Pose> points = createStraightPathPoints(startPoint, stopPoint, orientationVector, numPoints);

  //finner plan som korresponderer med punkter Denne kan erstattes med calculateCartesianPath
  std::vector<moveit::planning_interface::MoveGroupInterface::Plan> plans = findPlans(points, "group_2", node);
  //Node is bound to the move_group above which will not be follow_joint_trajectory. Need to make a new node and movegroup for this node
  auto const followJointTrajectoryNode = std::make_shared<rclcpp::Node>("follow_joint_trajectory", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
  auto move_group_interface = MoveGroupInterface(followJointTrajectoryNode, "follow_joint_trajectory");
  move_group_interface.setMaxVelocityScalingFactor(0.02); //setter skalering -> nærmere 0 == tregere
  move_group_interface.setReplanAttempts(20);
  move_group_interface.setPlanningTime(10);
  moveit_msgs::msg::OrientationConstraint oconst;
  oconst.orientation.x = 1;
  oconst.link_name = "group_2/link_6_t";
  oconst.weight = 1;

  moveit_msgs::msg::JointConstraint jconst;
  jconst.joint_name = "group_2/joint_4_r";
  jconst.position = 0;
  jconst.weight = 1;


  moveit_msgs::msg::Constraints constraints;
  constraints.name = "group_2_constraints";
  constraints.orientation_constraints.push_back(oconst);
  constraints.joint_constraints.push_back(jconst);

  moveit_msgs::msg::TrajectoryConstraints trajconst;
  trajconst.constraints.push_back(constraints);

  move_group_interface.setTrajectoryConstraints(trajconst);
  move_group_interface.setWorkspace(-1.5, -1,5, 1.5, 1.5, 3);//setter workspace
  
  
  //kan ikke planlegge etter punkt, men kan planlegge etter jointvalues
  //regner derfor ut bane for gruppe og kalkulerer ny bane etter sluttverdiene på på hver bane
  //For hver gruppe, finn ønsket baneetPoseTargetsplan()
  //Lag en ny jointstate som inneholder sluttverdiene til alle planene
  //kalkuler ny plan for systemet.
  

  std::vector<moveit::planning_interface::MoveGroupInterface::Plan> follow_joint_trajectory_plans;
  for (auto const& planset : plans){

    //plansset skal inneholde planer for alle grupper (per nå bare en gruppe)

    log("concatenate jointstates...", 1);
    sensor_msgs::msg::JointState desiredState = concatenateStates(std::vector<moveit::planning_interface::MoveGroupInterface::Plan>{planset});
    move_group_interface.setJointValueTarget(desiredState);
    if (&plans.front() != &planset){ //startposisjon er endret
      moveit_msgs::msg::RobotState startState;
      sensor_msgs::msg::JointState temp = concatenateStates(std::vector<moveit::planning_interface::MoveGroupInterface::Plan>{follow_joint_trajectory_plans.back()});
      startState.joint_state.name = temp.name;
      startState.joint_state.position = temp.position;
      startState.joint_state.velocity = temp.velocity;
      startState.joint_state.effort = temp.effort;
      log("setting start state to previous end state", 1);
      move_group_interface.setStartState(startState);

    }

    log("Planning...", 1);
    auto const [success, plan] = [&move_group_interface]{
      moveit::planning_interface::MoveGroupInterface::Plan msg;
      auto const ok = static_cast<bool>(move_group_interface.plan(msg));
      return std::make_pair(ok, msg);
    }();
    follow_joint_trajectory_plans.push_back(plan);
    log("Created plan", 1);
    //if (DEBUG){printTrajectory(plan);};
    //std::cout << "Execute plan? (yes): ";
    //std::string input;
    //std::cin >> input;
    //if (input == "yes"){
    //move_group_interface.execute(plan);
    //}
  }
  rclcpp::shutdown();
  return 0;
}
