#include <chrono>
#include <cmath>
#include <memory>
#include <thread>

#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <rclcpp/rclcpp.hpp>

class SetTargetPosition : public rclcpp::Node
{
public:
  SetTargetPosition() : Node("set_target_position")
  {
    declare_parameter("target_x", 0.10);
    declare_parameter("target_y", -0.063);
    declare_parameter("target_z", 0.35);
    declare_parameter("target_qx", 0.479426);
    declare_parameter("target_qy", 0.0);
    declare_parameter("target_qz", 0.0);
    declare_parameter("target_qw", 0.877583);
    declare_parameter("reference_frame", "dummy");
    declare_parameter("end_effector_link", "6_Link");
    declare_parameter("planning_time", 5.0);
    declare_parameter("planning_attempts", 1);
    declare_parameter("pipeline_id", "ompl");
    declare_parameter("planner_id", "RRTConnectkConfigDefault");
  }

  bool move()
  {
    using Clock = std::chrono::steady_clock;
    auto elapsed = [](Clock::time_point start) {
      return std::chrono::duration<double>(Clock::now() - start).count();
    };

    geometry_msgs::msg::Pose target;
    target.position.x = get_parameter("target_x").as_double();
    target.position.y = get_parameter("target_y").as_double();
    target.position.z = get_parameter("target_z").as_double();
    target.orientation.x = get_parameter("target_qx").as_double();
    target.orientation.y = get_parameter("target_qy").as_double();
    target.orientation.z = get_parameter("target_qz").as_double();
    target.orientation.w = get_parameter("target_qw").as_double();
    const double norm = std::sqrt(
      target.orientation.x * target.orientation.x + target.orientation.y * target.orientation.y +
      target.orientation.z * target.orientation.z + target.orientation.w * target.orientation.w);
    const double planning_time = get_parameter("planning_time").as_double();
    const int attempts = static_cast<int>(get_parameter("planning_attempts").as_int());
    if (!std::isfinite(target.position.x) || !std::isfinite(target.position.y) ||
        !std::isfinite(target.position.z) || !std::isfinite(norm) || norm < 1e-9 ||
        !std::isfinite(planning_time) || planning_time <= 0.0 || attempts < 1) {
      RCLCPP_ERROR(get_logger(), "Invalid target pose or planning parameters");
      return false;
    }
    target.orientation.x /= norm;
    target.orientation.y /= norm;
    target.orientation.z /= norm;
    target.orientation.w /= norm;

    const auto start = Clock::now();
    moveit::planning_interface::MoveGroupInterface group(shared_from_this(), "arm_group");
    const auto link = get_parameter("end_effector_link").as_string();
    group.setEndEffectorLink(link);
    group.setPoseReferenceFrame(get_parameter("reference_frame").as_string());
    group.setPlanningPipelineId(get_parameter("pipeline_id").as_string());
    group.setPlannerId(get_parameter("planner_id").as_string());
    group.setNumPlanningAttempts(attempts);
    group.setPlanningTime(planning_time);
    if (!group.getCurrentState(10.0)) {
      RCLCPP_ERROR(get_logger(), "Failed to receive a current robot state");
      return false;
    }
    RCLCPP_INFO(get_logger(), "Initialization and state wait: %.3f s", elapsed(start));
    group.setStartStateToCurrentState();
    if (!group.setPoseTarget(target, link)) {
      RCLCPP_ERROR(get_logger(), "Failed to set pose target for %s", link.c_str());
      return false;
    }

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    const auto planning_start = Clock::now();
    const auto plan_result = group.plan(plan);
    RCLCPP_INFO(get_logger(), "Planning: %.3f s (MoveIt reported %.3f s), code %d",
      elapsed(planning_start), plan.planning_time_, plan_result.val);
    if (plan_result != moveit::core::MoveItErrorCode::SUCCESS) {
      return false;
    }

    const auto execution_start = Clock::now();
    const auto execution_result = group.execute(plan);
    RCLCPP_INFO(get_logger(), "Execution: %.3f s, code %d; total: %.3f s",
      elapsed(execution_start), execution_result.val, elapsed(start));
    return execution_result == moveit::core::MoveItErrorCode::SUCCESS;
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SetTargetPosition>();
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread spinner([&executor]() { executor.spin(); });
  bool success = false;
  try {
    success = node->move();
  } catch (const std::exception & error) {
    RCLCPP_ERROR(node->get_logger(), "Move failed: %s", error.what());
  }
  executor.cancel();
  spinner.join();
  rclcpp::shutdown();
  return success ? 0 : 1;
}
