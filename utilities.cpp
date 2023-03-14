#include "utilities.h"

bool DEBUG = true;

int log(std::string data, const int verbose){
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



sensor_msgs::msg::JointState concatenateStates(const std::vector<moveit::planning_interface::MoveGroupInterface::Plan>& plans){ //
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


std::vector<moveit::planning_interface::MoveGroupInterface::Plan> findPlans(const std::vector<geometry_msgs::msg::Pose>& points, const std::string& group, const std::shared_ptr<rclcpp::Node> node){
  std::vector<moveit::planning_interface::MoveGroupInterface::Plan> plans;
  //Hentet fra moveit cpp tutorial
  using moveit::planning_interface::MoveGroupInterface;
  auto move_group_interface = MoveGroupInterface(node, group);
  //log(move_group_interface.getEndEffectorLink());

  //the first point has the robot current startvalues
  for (auto const& point : points){
    if (point == points.front()){ //if first plans is empty 
      log("setting target pose", 1);
      move_group_interface.setPoseTarget(point);
      log("creating plan", 1);
      auto const [success, plan] = [&move_group_interface]{
        moveit::planning_interface::MoveGroupInterface::Plan msg;
        auto const ok = static_cast<bool>(move_group_interface.plan(msg));
        return std::make_pair(ok, msg);
      }();
      if (success){
        plans.push_back(plan);
      }
      else{
        log("PLANNING FAILED! EXITING");
        exit(-1);
      } 
    }
    else{//else calculate from previous position
      moveit_msgs::msg::RobotState startState;
      sensor_msgs::msg::JointState temp = concatenateStates(std::vector<moveit::planning_interface::MoveGroupInterface::Plan>{plans.back()});
      startState.joint_state.name = temp.name;
      startState.joint_state.position = temp.position;
      startState.joint_state.velocity = temp.velocity;
      startState.joint_state.effort = temp.effort;

      log("setting start state to previous end state", 1);
      move_group_interface.setStartState(startState);
      log("setting target pose", 1);
      move_group_interface.setPoseTarget(point);
      log("creating plan", 1);
      auto const [success, plan] = [&move_group_interface]{
        moveit::planning_interface::MoveGroupInterface::Plan msg;
        auto const ok = static_cast<bool>(move_group_interface.plan(msg));
        return std::make_pair(ok, msg);
      }();
      if (success){
        plans.push_back(plan);
      }
      else{
        log("PLANNING FAILED! EXITING");
        exit(-1);
      } 
    }
  }
  return plans;
}




std::vector<geometry_msgs::msg::Pose> createStraightPathPoints(std::vector<double> xyz_start, std::vector<double> xyz_stop, std::vector<double> xyzw_orientation, int num_points){
  std::vector<geometry_msgs::msg::Pose> points;

    std::cout << xyz_start.size();
  assert(xyzw_orientation.size() == 4); //må være verdier for alle 
  //assert((xyz_start.size() == xyz_stop.size()) == 3); //må være verdier for xyz

  auto const target_pose = [](double x, double y, double z, std::vector<double> xyzw_orientation){
    geometry_msgs::msg::Pose msg;
    msg.position.x = x;
    msg.position.y = y;
    msg.position.z = z;
    msg.orientation.x = xyzw_orientation.at(0);
    msg.orientation.y = xyzw_orientation.at(1);
    msg.orientation.z = xyzw_orientation.at(2);
    msg.orientation.w = xyzw_orientation.at(3);
    return msg;
  };

  double dx = (xyz_stop.at(0) - xyz_start.at(0))/num_points;
  double dy = (xyz_stop.at(1) - xyz_start.at(1))/num_points;
  double dz = (xyz_stop.at(2) - xyz_start.at(2))/num_points;

  for(int i = 0; i < num_points - 1; i++){
    points.push_back(target_pose(xyz_start.at(0) + dx * i, xyz_start.at(1) + dy * i, xyz_start.at(2) + dz * i, xyzw_orientation));
  }
  points.push_back(target_pose(xyz_stop.at(0), xyz_stop.at(1), xyz_stop.at(2), xyzw_orientation));
  return points;
}