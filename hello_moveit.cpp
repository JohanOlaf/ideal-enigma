#include <cstdio>
#include <memory>

#include "iostream"

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

#include <moveit_msgs/msg/robot_state.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>

#include <sensor_msgs/msg/joint_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

bool DEBUG = true;
const std::string NODENAME = "hello_moveit";

int log(std::string data, const int verbose = 0){
  if (verbose == 0){

    std::cout << data << std::endl;
  }
  else if (verbose > 0 and DEBUG){
    std::cout << data << std::endl;
  }
  return 1;
}



std::vector<std::pair<std::string, std::pair<size_t, size_t>>> findNumberOfMatches(const std::vector<std::string>& keywords, const std::vector<std::string>& dataset) {
  std::vector<std::pair<std::string, std::pair<size_t, size_t>>> results;

  for (const auto& sequence : keywords){
    size_t counter  = 0;
    size_t idx_counter = 0;
    //for alle gruppene 
    for (const auto& group : dataset){
      //se om sequence finnes i gruppe, evt hvor mange
      if (group.find(sequence) != std::string::npos){
        counter ++;
      }
      if (counter == 0){
        idx_counter ++; //vi vil bare finne posisjonen til den første matches.
      }
    }
    results.push_back(std::make_pair(sequence, std::make_pair(counter, idx_counter)));

  }
  return results;
}

int printTrajectory(moveit::planning_interface::MoveGroupInterface::Plan plan){
  for (auto it : plan.trajectory_.joint_trajectory.points){
    for (auto point : it.positions){
      std::cout << point << " ";
    }
    std::cout << std::endl;
  }
  return 1;
}

bool compareByIndex(const std::pair<std::string, std::pair<size_t, size_t>>& a, const std::pair<std::string, std::pair<size_t, size_t>>& b) {
    return a.second.second < b.second.second;
}

bool isGroupInPlan(const std::string& group, const moveit::planning_interface::MoveGroupInterface::Plan& plan){
    if (plan.trajectory_.joint_trajectory.joint_names[0].find(group) != std::string::npos){
      return true;
    }
  return false;
}


int8_t isGroupInPlans(const std::string& group, const std::vector<moveit::planning_interface::MoveGroupInterface::Plan>& plans){
  //returnerer indeksen gruppen finnes i planvektoren, -1 hvis den ikke finnes
  int8_t index = 0;
  for (const auto& plan : plans){
    if (isGroupInPlan(group, plan)){
      return index;
    }
    index ++;
  }
  return -1;
}


moveit::planning_interface::MoveGroupInterface::Plan expandTrajectory(moveit::planning_interface::MoveGroupInterface::Plan plan, size_t lengthOfTrajectory){
  //hver plan skal inneholde start_state_
  
  moveit::planning_interface::MoveGroupInterface::Plan newPlan = plan;

  for(size_t i = plan.trajectory_.joint_trajectory.points.size(); i < lengthOfTrajectory; i++){
    newPlan.trajectory_.joint_trajectory.points.push_back(plan.trajectory_.joint_trajectory.points.back());
  }
  return newPlan;
}
moveit::planning_interface::MoveGroupInterface::Plan newPlanFromStartState(moveit::planning_interface::MoveGroupInterface::Plan templatePlan, std::string name, size_t numberOfJoints, size_t startIndex){

  moveit::planning_interface::MoveGroupInterface::Plan newPlan(templatePlan);

  newPlan.trajectory_.joint_trajectory.joint_names = std::vector<std::string>(templatePlan.start_state_.joint_state.name.begin() + startIndex, templatePlan.start_state_.joint_state.name.begin() + startIndex + numberOfJoints);
  newPlan.trajectory_.joint_trajectory.points[0].positions = std::vector<double>(templatePlan.start_state_.joint_state.position.begin() + startIndex, templatePlan.start_state_.joint_state.position.begin() + startIndex + numberOfJoints);
  //effort er tom slik at dette leder til segfault
  //newPlan.trajectory_.joint_trajectory.points[0].effort = std::vector<double>(templatePlan.start_state_.joint_state.effort.begin() + startIndex, templatePlan.start_state_.joint_state.effort.begin() + startIndex + numberOfJoints);
  newPlan.trajectory_.joint_trajectory.points[0].velocities = std::vector<double>(templatePlan.start_state_.joint_state.velocity.begin() + startIndex, templatePlan.start_state_.joint_state.velocity.begin() + startIndex + numberOfJoints);
  newPlan.trajectory_.joint_trajectory.points[0].time_from_start = templatePlan.trajectory_.joint_trajectory.points[0].time_from_start; //dette vil finnes første punkt [0]
  newPlan.trajectory_.joint_trajectory.points[0].accelerations = std::vector<double>(numberOfJoints, 0.0);

  //Er bare interessert i første punkt
  std::vector<trajectory_msgs::msg::JointTrajectoryPoint> firstPoint;
  firstPoint.push_back(newPlan.trajectory_.joint_trajectory.points[0]);
  newPlan.trajectory_.joint_trajectory.points = firstPoint;

  return newPlan;
}


moveit::planning_interface::MoveGroupInterface::Plan createPlan(std::vector<moveit::planning_interface::MoveGroupInterface::Plan> plans, const std::vector<std::string> group_names){
  assert(plans.size()); //assert that there is a plan
  moveit::planning_interface::MoveGroupInterface::Plan newPlan;
  newPlan.planning_time_ = 0;
  newPlan.start_state_   = plans[0].start_state_; //Alle start_state_ skal være like

  uint8_t n_joints = plans[0].start_state_.joint_state.name.size(); //15 for dette systemet
  uint8_t longestTrajectory = 0; //banen kan ikke være lenger enn 200 punkter uint8 bør være nok
  uint8_t longestTrajectoryIndex = 0;

  uint8_t counter = 0;
  for (auto const plan : plans){
    newPlan.planning_time_ += plan.planning_time_;
    uint8_t n = plan.trajectory_.joint_trajectory.points.size();
    if (n > longestTrajectory){
      longestTrajectory = n;
      longestTrajectoryIndex = counter;
    }
    counter ++;
  }
  newPlan.trajectory_.joint_trajectory.joint_names = newPlan.start_state_.joint_state.name;
  newPlan.trajectory_.joint_trajectory.header = plans[0].trajectory_.joint_trajectory.header; //Tror disse skal være like :^)

  std::vector<std::pair<std::string, std::pair<size_t, size_t>>> numberOfJointsInGroup = findNumberOfMatches(group_names, newPlan.start_state_.joint_state.name);
  //Sorterer siden vi ikke kan anta at gruppe 3 ligger "3 grupper inn", feks hvis gruppe 2 er montert på gruppe 3 kan det se sånn ut:
  // gruppe 1, gruppe 3, gruppe 2...;
  std::sort(numberOfJointsInGroup.begin(), numberOfJointsInGroup.end(), compareByIndex); 

  //Tar utgangspunktet i den lengste banen blir ikke brukt !!
  newPlan.trajectory_.joint_trajectory.points = plans[longestTrajectoryIndex].trajectory_.joint_trajectory.points;

  std::vector<moveit::planning_interface::MoveGroupInterface::Plan> tempPlans;
  

  for(const auto& group : numberOfJointsInGroup){
    std::string groupName = group.first;
    size_t index = group.second.second;
    size_t numberOfJoints = group.second.first;
    int8_t indexInPlans = isGroupInPlans(groupName, plans);


    //hvis indexInPlans er negativ må vi lage ny plan
    if (indexInPlans < 0){
      log("lager og utvider for " + groupName, 1);
      moveit::planning_interface::MoveGroupInterface::Plan tempPlan = expandTrajectory(newPlanFromStartState(plans[0], groupName, numberOfJoints, index), longestTrajectory);
      tempPlans.push_back(tempPlan);
    }
    else{
      //yes sir denne finnes så vi trenger bare å expande
      moveit::planning_interface::MoveGroupInterface::Plan tempPlan = expandTrajectory(plans[indexInPlans], longestTrajectory);
      tempPlans.push_back(tempPlan);
    }
  }

  //har nå n mange planer som er like lange :) ikke like brede!


  for (size_t k = 1; k < tempPlans.size(); k ++){
    for (uint8_t i = 0; i < longestTrajectory; i++){
      tempPlans[0].trajectory_.joint_trajectory.points[i].positions.insert(std::end(tempPlans[0].trajectory_.joint_trajectory.points[i].positions), std::begin(tempPlans[k].trajectory_.joint_trajectory.points[i].positions), std::end(tempPlans[k].trajectory_.joint_trajectory.points[i].positions));
      tempPlans[0].trajectory_.joint_trajectory.points[i].velocities.insert(std::end(tempPlans[0].trajectory_.joint_trajectory.points[i].velocities), std::begin(tempPlans[k].trajectory_.joint_trajectory.points[i].velocities), std::end(tempPlans[k].trajectory_.joint_trajectory.points[i].velocities));
      tempPlans[0].trajectory_.joint_trajectory.points[i].accelerations.insert(std::end(tempPlans[0].trajectory_.joint_trajectory.points[i].accelerations), std::begin(tempPlans[k].trajectory_.joint_trajectory.points[i].accelerations), std::end(tempPlans[k].trajectory_.joint_trajectory.points[i].accelerations));
      tempPlans[0].trajectory_.joint_trajectory.points[i].effort.insert(std::end(tempPlans[0].trajectory_.joint_trajectory.points[i].effort), std::begin(tempPlans[k].trajectory_.joint_trajectory.points[i].effort), std::end(tempPlans[k].trajectory_.joint_trajectory.points[i].effort));

    }
  }
  printTrajectory(tempPlans[0]);
  return tempPlans[0];

}


int sniff(moveit::planning_interface::MoveGroupInterface::Plan plan){
  for (auto it: plan.trajectory_.joint_trajectory.points){
    for(auto point : it.positions){
      std::cout << point << " ";
    }
    std::cout << std::endl;
  }
  return 1;
}


int8_t findIndex(std::vector<std::string> subset, std::vector<std::string> set){
  //rekkefølgen er lik slik at vi kan bare iterere opp
  //samtidig vil set.size() > subset.size() slik at vi kan iterere gjennom null problem
  for (uint8_t i = 0; i < set.size(); i ++){
    //finner den første matchen
    if(set.at(i) == subset.front()){
      return i;
    }
  }
  return -1;
}



sensor_msgs::msg::JointState concatenateStates(const std::vector<moveit::planning_interface::MoveGroupInterface::Plan>& plans){
  sensor_msgs::msg::JointState state(plans[0].start_state_.joint_state);
  /*
  må finne ut hvilken posisjon i joint_state de forskjellige verdiene skal
  group_names = vector<string>
  */
  for (auto const& plan : plans){
    int8_t idx = findIndex(plan.trajectory_.joint_trajectory.joint_names, plan.start_state_.joint_state.name);
    int8_t sizeOfGroup = plan.trajectory_.joint_trajectory.joint_names.size();
    if (idx >= 0){
      state.position.erase(state.position.begin() + idx, state.position.begin() + idx + sizeOfGroup);
      state.position.insert(state.position.begin() + idx, plan.trajectory_.joint_trajectory.points.back().positions.begin(), plan.trajectory_.joint_trajectory.points.back().positions.end());
    }
  }
  return state;
}


int main(int argc, char ** argv)
{
  log("Initializing ROS node", 1);
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(NODENAME, rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
  log("Creating ros logger", 1);
  auto const logger = rclcpp::get_logger(NODENAME);

  std::vector<std::string> groupNames = {"group_1", "group_2", "group_3", "group_4"};


  //toodoo siden gruppe 1 inneholder gruppe 3 ledd -> plan inneholder verdier for båd egruppe 1 og 3. Må splitte disse
  //potensiell quickfix: virtuelle grupper
  std::vector<std::string> groups = {"group_2"};
  std::vector<moveit::planning_interface::MoveGroupInterface::Plan> plans;
  using moveit::planning_interface::MoveGroupInterface;
  for(auto const& group : groups){
    log("creating move group for:" + group);
    auto move_group_interface = MoveGroupInterface(node, group);

    auto const target_pose = []{
      geometry_msgs::msg::Pose msg;
      msg.position.x = 0;
      msg.position.y = 1.2;
      msg.position.z = 2;
      return msg;
    }();
    log("setting target pose", 1);
    move_group_interface.setPoseTarget(target_pose);
    log("creating plan", 1);
    auto const [success, plan] = [&move_group_interface]{
      moveit::planning_interface::MoveGroupInterface::Plan msg;
      auto const ok = static_cast<bool>(move_group_interface.plan(msg));
      return std::make_pair(ok, msg);
    }();
  
    if(success){
      log("Found plan");
      //move_group_interface.execute(plan);
      plans.push_back(plan);
    }
    else{
      RCLCPP_ERROR(logger, "Planing failed!");
      log("Exiting");
      return 0;
    }

  
//  newPlanFromStartState(plans[0], "group_4", 3, 0);
//  log("hvis dette ikke printes er funksjonen feil");
  }

  //slår sammen planene for alle grupper. vil sansynligvis ikke trenges men henger igjen etter utviklinen
  //moveit::planning_interface::MoveGroupInterface::Plan p = createPlan(plans, groupNames);


  //Node is bound to the move_group above which will not be follow_joint_trajectory. Need to make a new node and movegroup for this node
  auto const followJointTrajectoryNode = std::make_shared<rclcpp::Node>("follow_joint_trajectory", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
  auto move_group_interface = MoveGroupInterface(followJointTrajectoryNode, "follow_joint_trajectory");
  move_group_interface.setMaxVelocityScalingFactor(0.02); //setter skalering -> nærmere 0 == tregere

  //moveit::planning_interface::MoveGroupInterface::Plan tempPlan(p); //lager en ny plan som vi kopierer over for å unngå referansekrasj


  log("slår sammen lister");
  sensor_msgs::msg::JointState desiredState = concatenateStates(plans);

  /*
  //lager en ny plan
  //Denne skal kun inneholde sluttverdiene på jointene
  //Dermed skal move_group_interface.plan(denne nye planen)
  
  moveit::planning_interface::MoveGroupInterface::Plan emptyPlan = newPlanFromStartState(tempPlan, "group_1", 15, 0);
  for (std::string name : emptyPlan.trajectory_.joint_trajectory.joint_names){
    std::cout << name << " ";
  }
  std::cout << std::endl;
  emptyPlan.trajectory_.joint_trajectory.points = std::vector<trajectory_msgs::msg::JointTrajectoryPoint>{tempPlan.trajectory_.joint_trajectory.points.back()};
  printTrajectory(emptyPlan);

  sensor_msgs::msg::JointState desiredState = emptyPlan.start_state_.joint_state;
  desiredState.position = emptyPlan.trajectory_.joint_trajectory.points.back().positions;
  */

  move_group_interface.setJointValueTarget(desiredState);

  auto const [success, plan] = [&move_group_interface]{
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    auto const ok = static_cast<bool>(move_group_interface.plan(msg));
    return std::make_pair(ok, msg);
  }();
  

  printTrajectory(plan);
  log("laget bane");

  std::cout << "prøv å utfør bane?: ";
  std::string input;
  std::cin >> input;
  if (input == "yes"){
  move_group_interface.execute(plan);
  }
  rclcpp::shutdown();
  return 0;
}
