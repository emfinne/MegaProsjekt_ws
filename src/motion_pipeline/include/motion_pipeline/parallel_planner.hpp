#ifndef MEGAPROSJEKT_WS_PARALLEL_PLANNER_HPP
#define MEGAPROSJEKT_WS_PARALLEL_PLANNER_HPP

#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/moveit_cpp/moveit_cpp.hpp>
#include <moveit/moveit_cpp/planning_component.hpp>
#include <moveit/planning_interface/planning_response.hpp>
#include <moveit/robot_trajectory/robot_trajectory.hpp>

struct Config
{
    std::string planning_group;
    std::vector<std::string> joint_names;          // order matches joint-goal vectors
    std::vector<std::string> pipeline_presets;     // e.g. {"ompl_rrtc", "pilz_lin", ...}
    std::vector<std::string> controllers;          // for execute()
};

class ParallelPlanner
{
public:


  using Selector = std::function<planning_interface::MotionPlanResponse(
      const std::vector<planning_interface::MotionPlanResponse>&)>;

  ParallelPlanner(rclcpp::Node::SharedPtr node, Config config);

  std::optional<robot_trajectory::RobotTrajectoryPtr>
  planToJointGoal(const std::vector<double>& joint_values,
                  Selector selector = shortestPath);

  std::optional<robot_trajectory::RobotTrajectoryPtr>
  planToPoseGoal(const geometry_msgs::msg::PoseStamped& pose,
                 const std::string& ee_link,
                 Selector selector = shortestPath);

  bool execute(const robot_trajectory::RobotTrajectoryPtr& trajectory);

  // Default selector — picks shortest path length among successful solutions.
  static planning_interface::MotionPlanResponse
  shortestPath(const std::vector<planning_interface::MotionPlanResponse>& solutions);

private:
  std::optional<robot_trajectory::RobotTrajectoryPtr> runPlan(Selector selector);

  rclcpp::Node::SharedPtr node_;
  rclcpp::Logger logger_;
  Config config_;
  std::shared_ptr<moveit_cpp::MoveItCpp> moveit_cpp_;
  std::shared_ptr<moveit_cpp::PlanningComponent> planning_component_;
  moveit_cpp::PlanningComponent::MultiPipelinePlanRequestParameters multi_request_;
};


#endif //MEGAPROSJEKT_WS_PARALLEL_PLANNER_HPP
