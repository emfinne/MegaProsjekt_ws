
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <thread>
#include <rclcpp_action/rclcpp_action.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include "motion_pipeline/parallel_planner.hpp"
#include "interfaces/action/plan_and_execute.hpp"

using PlanAndExecute = interfaces::action::PlanAndExecute;
using GoalHandle = rclcpp_action::ServerGoalHandle<PlanAndExecute>;

class PlannerNode
{
public:
  PlannerNode(rclcpp::Node::SharedPtr node, Config config)
    : node_{ node }
    , planner_{ std::make_shared<ParallelPlanner>(node, std::move(config)) }
  {
    action_server_ = rclcpp_action::create_server<PlanAndExecute>(
        node_, "plan_and_execute",
        std::bind(&PlannerNode::handleGoal, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&PlannerNode::handleCancel, this, std::placeholders::_1),
        std::bind(&PlannerNode::handleAccepted, this, std::placeholders::_1));
  }

private:
  rclcpp_action::GoalResponse handleGoal(const rclcpp_action::GoalUUID&,
                                          std::shared_ptr<const PlanAndExecute::Goal> goal)
  {
    RCLCPP_INFO(node_->get_logger(), "Received goal (type=%u, plan_only=%d)",
                goal->goal_type, goal->plan_only);
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handleCancel(const std::shared_ptr<GoalHandle>)
  {
    RCLCPP_INFO(node_->get_logger(), "Cancel requested");
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handleAccepted(const std::shared_ptr<GoalHandle> goal_handle)
  {
    // Run on a background thread so the executor stays responsive.
    std::thread{ std::bind(&PlannerNode::execute, this, goal_handle) }.detach();
  }

  void execute(const std::shared_ptr<GoalHandle> goal_handle)
  {
    auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<PlanAndExecute::Feedback>();
    auto result = std::make_shared<PlanAndExecute::Result>();

    feedback->state = "PLANNING";
    goal_handle->publish_feedback(feedback);

    std::optional<robot_trajectory::RobotTrajectoryPtr> trajectory;
    if (goal->goal_type == PlanAndExecute::Goal::GOAL_TYPE_JOINT)
    {
      trajectory = planner_->planToJointGoal(goal->joint_values);
    }
    else if (goal->goal_type == PlanAndExecute::Goal::GOAL_TYPE_POSE)
    {
      trajectory = planner_->planToPoseGoal(goal->target_pose, goal->ee_link);
    }
    else
    {
      result->success = false;
      result->message = "Unknown goal_type";
      goal_handle->abort(result);
      return;
    }

    if (!trajectory)
    {
      result->success = false;
      result->message = "Planning failed";
      goal_handle->abort(result);
      return;
    }

    // Fill trajectory in result so the client can inspect / re-execute later.
    moveit_msgs::msg::RobotTrajectory traj_msg;
    (*trajectory)->getRobotTrajectoryMsg(traj_msg);
    result->trajectory = traj_msg;
    result->path_length = robot_trajectory::pathLength(**trajectory);

    if (goal->plan_only)
    {
      result->success = true;
      result->message = "Planning succeeded (plan_only)";
      goal_handle->succeed(result);
      return;
    }

    if (goal_handle->is_canceling())
    {
      result->success = false;
      result->message = "Cancelled before execution";
      goal_handle->canceled(result);
      return;
    }

    feedback->state = "EXECUTING";
    goal_handle->publish_feedback(feedback);

    bool ok = planner_->execute(*trajectory);
    result->success = ok;
    result->message = ok ? "Done" : "Execution failed";

    if (ok)
      goal_handle->succeed(result);
    else
      goal_handle->abort(result);
  }

  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<ParallelPlanner> planner_;
  rclcpp_action::Server<PlanAndExecute>::SharedPtr action_server_;
};



int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);

  auto node = rclcpp::Node::make_shared("motion_pipeline_node", "", options);

  Config config;

  config.planning_group = "ur_arm";
  config.joint_names = {
    "ur3_shoulder_pan_joint",
    "ur3_shoulder_lift_joint",
    "ur3_elbow_joint",
    "ur3_wrist_1_joint",
    "ur3_wrist_2_joint",
    "ur3_wrist_3_joint",
  };
  config.pipeline_presets = { "ompl_rrtc", "pilz_lin", "chomp_planner", "ompl_rrt_star" };
  config.controllers = { "scaled_joint_trajectory_controller" };

  PlannerNode planner_node(node, config);

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}