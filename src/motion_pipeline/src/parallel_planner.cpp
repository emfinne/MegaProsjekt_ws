#include "motion_pipeline/parallel_planner.hpp"
#include <algorithm>



ParallelPlanner::ParallelPlanner(rclcpp::Node::SharedPtr node, Config config)
  : node_{ std::move(node) }
  , logger_{ rclcpp::get_logger("parallel_planner") }
  , config_{ std::move(config) }
  , moveit_cpp_{ std::make_shared<moveit_cpp::MoveItCpp>(node_) }
  , planning_component_{ std::make_shared<moveit_cpp::PlanningComponent>(config_.planning_group, moveit_cpp_) }
  , multi_request_{ node_, config_.pipeline_presets }
{
  moveit_cpp_->getPlanningSceneMonitorNonConst()->providePlanningSceneService();
}

std::optional<robot_trajectory::RobotTrajectoryPtr>
ParallelPlanner::planToJointGoal(const std::vector<double>& joint_values, Selector selector)
{
  if (joint_values.size() != config_.joint_names.size())
  {
    RCLCPP_ERROR(logger_, "Joint goal size %zu does not match configured group size %zu",
                 joint_values.size(), config_.joint_names.size());
    return std::nullopt;
  }

  auto goal = planning_component_->getStartState();
  for (size_t i = 0; i < config_.joint_names.size(); ++i)
  {
    goal->setJointPositions(config_.joint_names[i], &joint_values[i]);
  }
  planning_component_->setGoal(*goal);
  return runPlan(std::move(selector));
}

std::optional<robot_trajectory::RobotTrajectoryPtr>
ParallelPlanner::planToPoseGoal(const geometry_msgs::msg::PoseStamped& pose,
                                 const std::string& ee_link, Selector selector)
{
  planning_component_->setGoal(pose, ee_link);
  return runPlan(std::move(selector));
}

std::optional<robot_trajectory::RobotTrajectoryPtr>
ParallelPlanner::runPlan(Selector selector)
{
  planning_component_->setStartStateToCurrentState();
  auto solution = planning_component_->plan(multi_request_, std::move(selector));
  if (!solution)
  {
    RCLCPP_ERROR(logger_, "Parallel planning failed");
    return std::nullopt;
  }
  return solution.trajectory;
}

bool ParallelPlanner::execute(const robot_trajectory::RobotTrajectoryPtr& trajectory)
{
  if (!trajectory)
  {
    RCLCPP_ERROR(logger_, "Refusing to execute null trajectory");
    return false;
  }
  return static_cast<bool>(moveit_cpp_->execute(trajectory, config_.controllers));
}

planning_interface::MotionPlanResponse
ParallelPlanner::shortestPath(const std::vector<planning_interface::MotionPlanResponse>& solutions)
{
  static auto const logger = rclcpp::get_logger("parallel_planner.selector");
  RCLCPP_INFO(logger, "###################### Results ######################");
  for (auto const& s : solutions)
  {
    RCLCPP_INFO(logger, "Planner '%s' -> '%s'", s.planner_id.c_str(),
                moveit::core::errorCodeToString(s.error_code).c_str());
    if (s.trajectory)
    {
      RCLCPP_INFO(logger, "  path length: %f, planning time: %f",
                  robot_trajectory::pathLength(*s.trajectory), s.planning_time);
    }
  }

  auto const best = std::min_element(
      solutions.begin(), solutions.end(),
      [](const planning_interface::MotionPlanResponse& a,
         const planning_interface::MotionPlanResponse& b) {
        if (a && b)
          return robot_trajectory::pathLength(*a.trajectory) <
                 robot_trajectory::pathLength(*b.trajectory);
        if (a) return true;
        return false;
      });

  RCLCPP_INFO(logger, "Chosen: '%s'", best->planner_id.c_str());
  return *best;
}
