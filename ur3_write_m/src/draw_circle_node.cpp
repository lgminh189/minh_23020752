#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <memory>
#include <iterator>
#include <thread>
#include <vector>

#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/robot_state/robot_state.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>

using namespace std::chrono_literals;

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("draw_circle_node");
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  std::thread spinner([&executor] { executor.spin(); });

  const auto group = node->declare_parameter<std::string>("planning_group", "ur_manipulator");
  const auto eef_link = node->declare_parameter<std::string>("end_effector_link", "tool0");
  const auto base_frame = node->declare_parameter<std::string>("base_frame", "base_link");
  const double radius = node->declare_parameter<double>("circle_radius", 0.04);
  const int points = node->declare_parameter<int>("circle_points", 96);
  const double top_clearance = node->declare_parameter<double>("top_clearance", 0.16);
  const double eef_step = node->declare_parameter<double>("eef_step", 0.003);
  const double workspace_search_step = node->declare_parameter<double>("workspace_search_step", 0.05);
  const bool execute = node->declare_parameter<bool>("execute", true);

  if (radius <= 0.0 || points < 12 || top_clearance < 0.0 || eef_step <= 0.0 || workspace_search_step <= 0.0) {
    RCLCPP_ERROR(node->get_logger(), "circle_radius và eef_step phải dương, top_clearance không âm; circle_points phải từ 12 trở lên.");
    executor.cancel(); spinner.join(); rclcpp::shutdown(); return 1;
  }

  moveit::planning_interface::MoveGroupInterface arm(node, group);
  arm.setPoseReferenceFrame(base_frame);
  arm.setEndEffectorLink(eef_link);
  arm.setPlanningTime(6.0);
  arm.setNumPlanningAttempts(3);
  arm.setMaxVelocityScalingFactor(0.12);
  arm.setMaxAccelerationScalingFactor(0.08);

  const auto state = arm.getCurrentState(20.0);
  if (!state) {
    RCLCPP_ERROR(node->get_logger(), "Không nhận được /joint_states.");
    executor.cancel(); spinner.join(); rclcpp::shutdown(); return 1;
  }

  // Bend the current elbow with a nearby joint-space target. The generic
  // SRDF test_configuration can be too far away for some simulator states.
  const auto & joint_names = arm.getJointNames();
  auto staging_joints = arm.getCurrentJointValues();
  const auto elbow_it = std::find(joint_names.begin(), joint_names.end(), "elbow_joint");
  const auto wrist1_it = std::find(joint_names.begin(), joint_names.end(), "wrist_1_joint");
  const auto wrist2_it = std::find(joint_names.begin(), joint_names.end(), "wrist_2_joint");
  if (elbow_it == joint_names.end() || wrist1_it == joint_names.end() ||
      wrist2_it == joint_names.end() || staging_joints.size() != joint_names.size()) {
    RCLCPP_ERROR(node->get_logger(), "Không tìm thấy elbow_joint/wrist_1_joint/wrist_2_joint trong planning group.");
    executor.cancel(); spinner.join(); rclcpp::shutdown(); return 1;
  }
  const size_t elbow_index = static_cast<size_t>(std::distance(joint_names.begin(), elbow_it));
  const size_t wrist1_index = static_cast<size_t>(std::distance(joint_names.begin(), wrist1_it));
  const size_t wrist2_index = static_cast<size_t>(std::distance(joint_names.begin(), wrist2_it));
  const double elbow_delta = std::clamp(0.60 - staging_joints[elbow_index], -0.60, 0.60);
  const double wrist2_delta = std::clamp(0.80 - staging_joints[wrist2_index], -0.80, 0.80);
  staging_joints[elbow_index] += elbow_delta;
  // Counter-rotate wrist 1 to keep the forearm direction close. Move wrist 2
  // away from its zero-angle wrist singularity as part of the staging move.
  staging_joints[wrist1_index] -= elbow_delta;
  staging_joints[wrist2_index] += wrist2_delta;

  arm.setStartStateToCurrentState();
  if (!arm.setJointValueTarget(staging_joints)) {
    RCLCPP_WARN(node->get_logger(), "Pose khuỷu gập sát giới hạn khớp; thử giá trị hiện tại có giới hạn.");
  }
  moveit::planning_interface::MoveGroupInterface::Plan approach;
  if (!arm.plan(approach) || (execute && arm.execute(approach) != moveit::core::MoveItErrorCode::SUCCESS)) {
    RCLCPP_ERROR(node->get_logger(), "Không thể đưa khuỷu ra khỏi vùng gần duỗi thẳng.");
    executor.cancel(); spinner.join(); rclcpp::shutdown(); return 1;
  }
  RCLCPP_INFO(node->get_logger(),
              "Pose trung gian: khuỷu đổi %.2f rad, cổ tay 2 đổi %.2f rad để tránh singularity.",
              elbow_delta, wrist2_delta);
  std::this_thread::sleep_for(500ms);

  // Keep the drawing plane vertical (XZ), but search nearby positions if the
  // nominal placement sits near a reach or collision boundary.
  auto nominal_center = arm.getCurrentPose(eef_link).pose;
  nominal_center.position.z -= top_clearance + radius;
  const std::array<std::array<double, 3>, 7> offsets{{
    {{0.0, 0.0, 0.0}},
    {{0.0, workspace_search_step, 0.0}}, {{0.0, -workspace_search_step, 0.0}},
    {{workspace_search_step, 0.0, 0.0}}, {{-workspace_search_step, 0.0, 0.0}},
    {{0.0, 0.0, workspace_search_step}}, {{0.0, 0.0, -workspace_search_step}}
  }};
  std::vector<geometry_msgs::msg::Pose> waypoints;
  std::array<double, 3> selected_offset{};
  bool placement_found = false;
  auto reference_state = arm.getCurrentState(1.0);
  if (!reference_state) {
    RCLCPP_ERROR(node->get_logger(), "Mất trạng thái robot khi dò vùng vẽ.");
    executor.cancel(); spinner.join(); rclcpp::shutdown(); return 1;
  }
  const auto * joint_group = reference_state->getJointModelGroup(group);
  if (!joint_group) {
    RCLCPP_ERROR(node->get_logger(), "Không tìm thấy planning group %s.", group.c_str());
    executor.cancel(); spinner.join(); rclcpp::shutdown(); return 1;
  }

  for (const auto & offset : offsets) {
    auto center = nominal_center;
    center.position.x += offset[0];
    center.position.y += offset[1];
    center.position.z += offset[2];
    std::vector<geometry_msgs::msg::Pose> candidate;
    candidate.reserve(static_cast<size_t>(points) + 1);
    for (int i = 0; i <= points; ++i) {
      // Start at the top of the circle and travel clockwise. The new start
      // phase selects a different continuous IK branch from the old route.
      const double angle = M_PI_2 - 2.0 * M_PI * static_cast<double>(i) / static_cast<double>(points);
      auto pose = center;
      pose.position.x += radius * std::cos(angle);
      pose.position.z += radius * std::sin(angle);
      candidate.push_back(pose);
    }

    bool ik_ok = true;
    size_t failed_sample = 0;
    bool singularity_failure = false;
    double failed_elbow = 0.0;
    double failed_wrist2 = 0.0;
    moveit::core::RobotState test_state(*reference_state);
    constexpr double singularity_margin = 0.30;
    for (size_t sample = 0; sample < candidate.size(); ++sample) {
      // Continue each IK solve from the preceding solution so this checks one
      // continuous joint branch around the entire loop, not isolated points.
      if (!test_state.setFromIK(joint_group, candidate[sample], eef_link, 0.04)) {
        ik_ok = false;
        failed_sample = sample;
        break;
      }
      const double elbow = test_state.getVariablePosition("elbow_joint");
      const double wrist_2 = test_state.getVariablePosition("wrist_2_joint");
      if (std::abs(std::sin(elbow)) < singularity_margin ||
          std::abs(std::sin(wrist_2)) < singularity_margin) {
        ik_ok = false;
        singularity_failure = true;
        failed_sample = sample;
        failed_elbow = elbow;
        failed_wrist2 = wrist_2;
        break;
      }
    }
    if (!ik_ok) {
      if (singularity_failure) {
        RCLCPP_INFO(node->get_logger(),
                    "Loại offset (%.0f, %.0f, %.0f) mm tại điểm %zu: khuỷu %.2f, cổ tay 2 %.2f rad.",
                    offset[0] * 1000.0, offset[1] * 1000.0, offset[2] * 1000.0,
                    failed_sample, failed_elbow, failed_wrist2);
      } else {
        RCLCPP_INFO(node->get_logger(),
                    "Loại offset (%.0f, %.0f, %.0f) mm: IK mất liên tục tại điểm %zu.",
                    offset[0] * 1000.0, offset[1] * 1000.0, offset[2] * 1000.0, failed_sample);
      }
      continue;
    }

    arm.setStartStateToCurrentState();
    arm.setPoseTarget(candidate.front(), eef_link);
    moveit::planning_interface::MoveGroupInterface::Plan start_plan;
    const bool planned = static_cast<bool>(arm.plan(start_plan));
    arm.clearPoseTargets();
    if (!planned) {
      RCLCPP_WARN(node->get_logger(), "Không tới được vị trí thử; chuyển vị trí vẽ vòng tròn.");
      continue;
    }
    if (execute && arm.execute(start_plan) != moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_ERROR(node->get_logger(), "Thực thi bước tiếp cận vòng tròn thất bại.");
      executor.cancel(); spinner.join(); rclcpp::shutdown(); return 1;
    }
    waypoints = std::move(candidate);
    selected_offset = offset;
    placement_found = true;
    break;
  }
  if (!placement_found) {
    RCLCPP_ERROR(node->get_logger(), "Không tìm được vùng lân cận có thể vẽ vòng tròn an toàn.");
    executor.cancel(); spinner.join(); rclcpp::shutdown(); return 1;
  }
  RCLCPP_INFO(node->get_logger(), "Chọn quỹ đạo vòng theo chiều kim đồng hồ, lệch (%.0f, %.0f, %.0f) mm; IK liên tục tránh singularity.",
              selected_offset[0] * 1000.0, selected_offset[1] * 1000.0,
              selected_offset[2] * 1000.0);

  auto marker_pub = node->create_publisher<visualization_msgs::msg::Marker>(
    "circle_path", rclcpp::QoS(1).transient_local());
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = base_frame;
  marker.header.stamp = node->now();
  marker.ns = "circle_360"; marker.id = 0;
  marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 0.01;
  marker.color.r = 0.05F; marker.color.g = 0.9F; marker.color.b = 1.0F; marker.color.a = 1.0F;
  for (const auto & pose : waypoints) marker.points.push_back(pose.position);
  marker_pub->publish(marker);

  // The selected point has already been reached with collision-aware planning;
  // now follow the visible path in Cartesian space.
  arm.setStartStateToCurrentState();
  moveit_msgs::msg::RobotTrajectory trajectory;
  const std::vector<geometry_msgs::msg::Pose> destinations(
    std::next(waypoints.begin()), waypoints.end());
  const double fraction = arm.computeCartesianPath(destinations, eef_step, trajectory, true);
  if (fraction >= 0.999) {
    if (execute && arm.execute(trajectory) != moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_ERROR(node->get_logger(), "Thực thi vòng tròn thất bại.");
      executor.cancel(); spinner.join(); rclcpp::shutdown(); return 1;
    }
  } else {
    RCLCPP_WARN(node->get_logger(),
                "Vòng tròn liên tục chỉ đạt %.1f%%; thử theo từng cung ngắn.",
                fraction * 100.0);
    if (!execute) {
      RCLCPP_ERROR(node->get_logger(),
                   "Không thể xác nhận vòng tròn đầy đủ ở chế độ execute:=false.");
      executor.cancel(); spinner.join(); rclcpp::shutdown(); return 1;
    }

    constexpr size_t points_per_chunk = 12;
    for (size_t first = 1; first < waypoints.size();) {
      const size_t last = std::min(first + points_per_chunk - 1, waypoints.size() - 1);
      const std::vector<geometry_msgs::msg::Pose> arc(
        waypoints.begin() + static_cast<std::ptrdiff_t>(first),
        waypoints.begin() + static_cast<std::ptrdiff_t>(last + 1));

      arm.setStartStateToCurrentState();
      moveit_msgs::msg::RobotTrajectory arc_trajectory;
      const double arc_fraction = arm.computeCartesianPath(arc, eef_step, arc_trajectory, true);
      if (arc_fraction >= 0.999) {
        if (arm.execute(arc_trajectory) != moveit::core::MoveItErrorCode::SUCCESS) {
          RCLCPP_ERROR(node->get_logger(), "Thực thi cung %zu của vòng tròn thất bại.", first);
          executor.cancel(); spinner.join(); rclcpp::shutdown(); return 1;
        }
      } else {
        RCLCPP_WARN(node->get_logger(),
                    "Cung %zu-%zu đạt %.1f%%; tính từng điểm để giữ quỹ đạo an toàn.",
                    first, last, arc_fraction * 100.0);
        for (size_t i = first; i <= last; ++i) {
          arm.setStartStateToCurrentState();
          moveit_msgs::msg::RobotTrajectory point_trajectory;
          const double point_fraction =
            arm.computeCartesianPath({waypoints[i]}, eef_step, point_trajectory, true);
          if (point_fraction >= 0.999) {
            if (arm.execute(point_trajectory) != moveit::core::MoveItErrorCode::SUCCESS) {
              RCLCPP_ERROR(node->get_logger(), "Thực thi waypoint %zu vòng tròn thất bại.", i);
              executor.cancel(); spinner.join(); rclcpp::shutdown(); return 1;
            }
            continue;
          }

          arm.setStartStateToCurrentState();
          arm.setPoseTarget(waypoints[i], eef_link);
          moveit::planning_interface::MoveGroupInterface::Plan point_plan;
          bool reached = false;
          for (int retry = 0; retry < 2; ++retry) {
            if (arm.plan(point_plan)) {
              arm.clearPoseTargets();
              reached = arm.execute(point_plan) == moveit::core::MoveItErrorCode::SUCCESS;
              break;
            }
            RCLCPP_WARN(node->get_logger(),
                        "Không lập được waypoint %zu vòng tròn; thử lại (%d/2).",
                        i, retry + 1);
          }
          arm.clearPoseTargets();
          if (!reached) {
            RCLCPP_ERROR(node->get_logger(), "Không thể tiếp cận waypoint %zu của vòng tròn.", i);
            executor.cancel(); spinner.join(); rclcpp::shutdown(); return 1;
          }
        }
      }
      first = last + 1;
    }
  }
  RCLCPP_INFO(node->get_logger(),
              "Hoàn thành quỹ đạo vòng tròn 360 độ. Giữ node chạy để hiển thị /circle_path trong RViz.");

  // A transient-local marker is retained only while its publisher exists.
  // Keep this process alive until the user stops the launch with Ctrl+C.
  while (rclcpp::ok()) std::this_thread::sleep_for(100ms);

  executor.cancel(); spinner.join(); rclcpp::shutdown();
  return 0;
}
