#include <chrono>
#include <cmath>
#include <array>
#include <memory>
#include <iterator>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/robot_state/robot_state.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>

using namespace std::chrono_literals;

class WriteM
{
public:
  explicit WriteM(const rclcpp::Node::SharedPtr & node) : node_(node)
  {
    base_frame_ = node_->declare_parameter<std::string>("base_frame", "base_link");
    group_ = node_->declare_parameter<std::string>("planning_group", "ur_manipulator");
    eef_link_ = node_->declare_parameter<std::string>("end_effector_link", "tool0");
    width_ = node_->declare_parameter<double>("letter_width", 0.12);
    height_ = node_->declare_parameter<double>("letter_height", 0.10);
    top_clearance_ = node_->declare_parameter<double>("top_clearance", 0.12);
    lift_ = node_->declare_parameter<double>("pen_lift", 0.04);
    step_ = node_->declare_parameter<double>("eef_step", 0.003);
    execute_ = node_->declare_parameter<bool>("execute", true);
    velocity_ = node_->declare_parameter<double>("velocity_scale", 0.15);
    acceleration_ = node_->declare_parameter<double>("acceleration_scale", 0.10);
    planning_time_ = node_->declare_parameter<double>("planning_time", 6.0);
    planning_attempts_ = node_->declare_parameter<int>("planning_attempts", 3);
    workspace_search_step_ = node_->declare_parameter<double>("workspace_search_step", 0.05);
    position_tolerance_ = node_->declare_parameter<double>("position_tolerance", 0.003);
    orientation_tolerance_ = node_->declare_parameter<double>("orientation_tolerance", 0.12);
    state_wait_seconds_ = node_->declare_parameter<double>("state_wait_seconds", 60.0);
    project_marker_to_floor_ = node_->declare_parameter<bool>("project_marker_to_floor", true);
    floor_marker_z_ = node_->declare_parameter<double>("floor_marker_z", 0.012);
    
    marker_pub_ = node_->create_publisher<visualization_msgs::msg::Marker>(
      "write_m_path", rclcpp::QoS(1).transient_local());
  }

  bool run()
  {
    moveit::planning_interface::MoveGroupInterface arm(node_, group_);
    arm.setPoseReferenceFrame(base_frame_);
    arm.setEndEffectorLink(eef_link_);
    arm.setPlanningTime(planning_time_);
    arm.setNumPlanningAttempts(planning_attempts_);
    arm.setGoalPositionTolerance(position_tolerance_);
    arm.setGoalOrientationTolerance(orientation_tolerance_);
    arm.setMaxVelocityScalingFactor(velocity_);
    arm.setMaxAccelerationScalingFactor(acceleration_);

    if (!wait_for_current_state(arm)) {
      RCLCPP_ERROR(node_->get_logger(), "Không nhận được /joint_states đầy đủ; hủy lập kế hoạch.");
      return false;
    }

    // Start from a bent-elbow pose instead of the nearly straight "up" pose.
    // The latter puts the UR arm close to its elbow singularity before drawing.
    if (!move_to_named(arm, "test_configuration")) {
      RCLCPP_ERROR(node_->get_logger(), "Không thể di chuyển đến cấu hình trung gian tránh duỗi thẳng khuỷu.");
      return false;
    }
    std::this_thread::sleep_for(500ms);

    auto tcp = arm.getCurrentPose(eef_link_).pose;
    // Search nearby positions from this bent-elbow reference. Candidates are
    // screened along the full stroke for IK continuity and elbow/wrist margin.
    tcp.position.z -= top_clearance_ + height_ / 2.0;
    const std::array<std::array<double, 3>, 7> offsets{{
      {{0.0, 0.0, 0.0}},
      {{0.0, workspace_search_step_, 0.0}}, {{0.0, -workspace_search_step_, 0.0}},
      {{workspace_search_step_, 0.0, 0.0}}, {{-workspace_search_step_, 0.0, 0.0}},
      {{0.0, 0.0, workspace_search_step_}}, {{0.0, 0.0, -workspace_search_step_}}
    }};

    std::vector<std::vector<geometry_msgs::msg::Pose>> drawing_strokes;
    std::array<double, 3> selected_offset{};
    bool placement_found = false;
    for (const auto & candidate_offset : offsets) {
      const auto candidate_center = offset(
        tcp, candidate_offset[0], candidate_offset[1], candidate_offset[2]);
      auto candidate_strokes = make_strokes(candidate_center);
      if (!placement_has_ik(arm, candidate_strokes)) {
        RCLCPP_INFO(node_->get_logger(),
                    "Vị trí lệch (%.0f, %.0f, %.0f) mm không đủ IK; thử vị trí khác.",
                    candidate_offset[0] * 1000.0, candidate_offset[1] * 1000.0,
                    candidate_offset[2] * 1000.0);
        continue;
      }

      // Approach above the first stroke before lowering the tool. If the
      // planner rejects this spot, try another nearby candidate automatically.
      if (!move_to(arm, raised(candidate_strokes.front().front()), "tiếp cận vùng vẽ an toàn") ||
          !move_to(arm, candidate_strokes.front().front(), "hạ đầu công tác vào nét đầu")) {
        RCLCPP_WARN(node_->get_logger(), "Planner không tới được vị trí thử; chuyển sang vị trí vẽ kế tiếp.");
        continue;
      }
      selected_offset = candidate_offset;
      drawing_strokes = std::move(candidate_strokes);
      placement_found = true;
      break;
    }
    if (!placement_found) {
      RCLCPP_ERROR(node_->get_logger(), "Không tìm được vùng lân cận có thể vẽ chữ M an toàn.");
      return false;
    }

    publish_marker(drawing_strokes);
    RCLCPP_INFO(node_->get_logger(),
                "Bắt đầu vẽ chữ M %.0f x %.0f cm từ cấu hình khuỷu gập, độ lệch (%.0f, %.0f, %.0f) mm trên XZ (%s)",
                width_ * 100.0, height_ * 100.0, selected_offset[0] * 1000.0,
                selected_offset[1] * 1000.0, selected_offset[2] * 1000.0, base_frame_.c_str());

    // Retract between strokes, then use collision-aware planning to reach the
    // next start point while keeping the drawn lines Cartesian.
    geometry_msgs::msg::Pose previous_end;
    for (size_t i = 0; i < drawing_strokes.size(); ++i) {
      const auto & stroke = drawing_strokes[i];
      if (i > 0) {
        if (!move_to(arm, raised(previous_end), "nâng đầu công tác sau nét")) {
          return false;
        }
        if (!move_to(arm, raised(stroke.front()), "di chuyển phía trên mặt phẳng vẽ")) {
          return false;
        }
        if (!move_to(arm, stroke.front(), "hạ đầu công tác vào nét")) {
          return false;
        }
      }

      if (!draw_stroke(arm, stroke, i + 1)) {
        RCLCPP_ERROR(node_->get_logger(), "Không thể hoàn tất nét %zu của chữ M.", i + 1);
        return false;
      }
      previous_end = stroke.back();
    }
    RCLCPP_INFO(node_->get_logger(), "Hoàn thành vẽ chữ M bằng một nét Cartesian liên tục!");
    return true;
  }

private:
  std::vector<std::vector<geometry_msgs::msg::Pose>> make_strokes(
    const geometry_msgs::msg::Pose & center) const
  {
    const auto left_bottom = offset(center, -width_ / 2, 0.0, -height_ / 2);
    const auto left_top = offset(center, -width_ / 2, 0.0, height_ / 2);
    const auto middle_valley = center;
    const auto right_top = offset(center, width_ / 2, 0.0, height_ / 2);
    const auto right_bottom = offset(center, width_ / 2, 0.0, -height_ / 2);
    // One uninterrupted right-to-left M path: lower-right -> upper-right ->
    // center valley -> upper-left -> lower-left.
    return {{right_bottom, right_top, middle_valley, left_top, left_bottom}};
  }

  bool placement_has_ik(
    moveit::planning_interface::MoveGroupInterface & arm,
    const std::vector<std::vector<geometry_msgs::msg::Pose>> & strokes)
  {
    auto reference_state = arm.getCurrentState(1.0);
    if (!reference_state) return false;
    const auto * joint_group = reference_state->getJointModelGroup(group_);
    if (!joint_group) return false;

    // Seed each IK query with the solution from the previous sample. This
    // follows one joint branch instead of checking disconnected endpoint IKs.
    moveit::core::RobotState test_state(*reference_state);
    const double singularity_margin = 0.30;
    auto solve_safe = [&](const geometry_msgs::msg::Pose & target) {
      if (!test_state.setFromIK(joint_group, target, eef_link_, 0.04)) return false;
      const double elbow = test_state.getVariablePosition("elbow_joint");
      const double wrist_2 = test_state.getVariablePosition("wrist_2_joint");
      return std::abs(std::sin(elbow)) >= singularity_margin &&
             std::abs(std::sin(wrist_2)) >= singularity_margin;
    };

    for (const auto & stroke : strokes) {
      if (stroke.empty()) return false;
      const auto start_above = raised(stroke.front());
      if (!solve_safe(start_above) || !solve_safe(stroke.front())) return false;
      for (size_t i = 0; i + 1 < stroke.size(); ++i) {
        const auto & from = stroke[i];
        const auto & to = stroke[i + 1];
        const double dx = to.position.x - from.position.x;
        const double dy = to.position.y - from.position.y;
        const double dz = to.position.z - from.position.z;
        const double length = std::sqrt(dx * dx + dy * dy + dz * dz);
        const int samples = std::max(1, static_cast<int>(std::ceil(length / 0.01)));
        for (int sample = 1; sample <= samples; ++sample) {
          const double t = static_cast<double>(sample) / samples;
          auto pose = from;
          pose.position.x += t * dx;
          pose.position.y += t * dy;
          pose.position.z += t * dz;
          if (!solve_safe(pose)) return false;
        }
      }
    }
    return true;
  }

  geometry_msgs::msg::Pose raised(geometry_msgs::msg::Pose pose) const
  {
    pose.position.y += lift_;
    return pose;
  }

  geometry_msgs::msg::Pose offset(geometry_msgs::msg::Pose p, double dx, double dy, double dz) const
  {
    p.position.x += dx;
    p.position.y += dy;
    p.position.z += dz;
    return p;
  }

  void publish_marker(const std::vector<std::vector<geometry_msgs::msg::Pose>> & strokes)
  {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = base_frame_;
    marker.header.stamp = node_->now();
    marker.ns = "letter_m";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::LINE_LIST;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.012;  // Độ dày nét vẽ 12mm
    marker.color.r = 1.0F;
    marker.color.g = 0.05F;
    marker.color.b = 0.05F;
    marker.color.a = 1.0F;

    for (const auto & stroke : strokes) {
      for (size_t i = 0; i + 1 < stroke.size(); ++i) {
        marker.points.push_back(marker_point(stroke[i]));
        marker.points.push_back(marker_point(stroke[i + 1]));
      }
    }
    marker_pub_->publish(marker);
  }

  geometry_msgs::msg::Point marker_point(const geometry_msgs::msg::Pose & pose) const
  {
    if (!project_marker_to_floor_) return pose.position;

    // The robot writes in its safe vertical XZ workspace.  Project that
    // drawing onto the white XY ground plane as the visible ink trace.
    geometry_msgs::msg::Point point;
    point.x = pose.position.x;
    point.y = pose.position.z;
    point.z = floor_marker_z_;
    return point;
  }

  bool move_to_named(moveit::planning_interface::MoveGroupInterface & arm, const std::string & name)
  {
    arm.setStartStateToCurrentState();
    arm.setNamedTarget(name);
    for (int retry = 0; retry < 2; ++retry) {
      moveit::planning_interface::MoveGroupInterface::Plan plan;
      if (arm.plan(plan)) {
        if (!execute_) return true;
        return arm.execute(plan) == moveit::core::MoveItErrorCode::SUCCESS;
      }
      RCLCPP_WARN(node_->get_logger(), "Không lập kế hoạch được cho pose %s, thử lại (%d/2).",
                  name.c_str(), retry + 1);
    }
    return false;
  }

  bool wait_for_current_state(moveit::planning_interface::MoveGroupInterface & arm)
  {
    const auto deadline = std::chrono::steady_clock::now() +
      std::chrono::duration<double>(state_wait_seconds_);
    while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline) {
      if (arm.getCurrentState(2.0)) {
        RCLCPP_INFO(node_->get_logger(), "Đã nhận /joint_states; bắt đầu vẽ chữ M.");
        return true;
      }
      RCLCPP_INFO(node_->get_logger(), "Đang chờ /joint_states...");
    }
    return false;
  }

  bool move_to(moveit::planning_interface::MoveGroupInterface & arm,
               const geometry_msgs::msg::Pose & target, const std::string & label)
  {
    arm.setStartStateToCurrentState();
    // The approach/retract may be a long displacement.  Use the normal
    // collision-aware planner here; the visible strokes use Cartesian interpolation.
    arm.setPoseTarget(target, eef_link_);
    for (int retry = 0; retry < 2; ++retry) {
      moveit::planning_interface::MoveGroupInterface::Plan plan;
      const auto planned = arm.plan(plan);
      if (planned) {
        arm.clearPoseTargets();
        if (!execute_) return true;
        return arm.execute(plan) == moveit::core::MoveItErrorCode::SUCCESS;
      }
      RCLCPP_WARN(node_->get_logger(), "Lập kế hoạch %s chưa có nghiệm, thử lại (%d/2).",
                  label.c_str(), retry + 1);
    }
    arm.clearPoseTargets();
    RCLCPP_ERROR(node_->get_logger(), "Không lập kế hoạch được cho bước %s", label.c_str());
    return false;
  }

  bool draw_stroke(moveit::planning_interface::MoveGroupInterface & arm,
                   const std::vector<geometry_msgs::msg::Pose> & waypoints, size_t number)
  {
    arm.setStartStateToCurrentState();
    moveit_msgs::msg::RobotTrajectory trajectory;
    const std::vector<geometry_msgs::msg::Pose> destinations(
      std::next(waypoints.begin()), waypoints.end());
    const double fraction = arm.computeCartesianPath(destinations, step_, trajectory, true);

    if (fraction >= 0.999) {
      if (!execute_) return true;
      return arm.execute(trajectory) == moveit::core::MoveItErrorCode::SUCCESS;
    }

    RCLCPP_WARN(node_->get_logger(),
                "Nét %zu liên tục chỉ đạt %.1f%%; thử lập kế hoạch riêng cho từng cạnh.",
                number, fraction * 100.0);
    if (!execute_) return true;

    // Restart IK from the latest robot state at each corner. This can avoid
    // solver branch changes in a long path while retaining Cartesian,
    // collision-checked motion for every visible line segment.
    constexpr int max_replans_per_edge = 2;
    for (size_t i = 0; i + 1 < waypoints.size(); ++i) {
      bool edge_complete = false;
      for (int attempt = 0; attempt < max_replans_per_edge; ++attempt) {
        arm.setStartStateToCurrentState();
        moveit_msgs::msg::RobotTrajectory segment;
        const double segment_fraction =
          arm.computeCartesianPath({waypoints[i + 1]}, step_, segment, true);
        if (segment_fraction < 0.05) {
          RCLCPP_ERROR(node_->get_logger(),
                       "Không thể tiếp tục nét Cartesian liên tục tại đoạn %zu của nét %zu; dừng để không rời khỏi đường vẽ.",
                       i + 1, number);
          return false;
        }
        if (arm.execute(segment) != moveit::core::MoveItErrorCode::SUCCESS) {
          RCLCPP_ERROR(node_->get_logger(), "Thực thi cạnh %zu của nét %zu thất bại.", i + 1, number);
          return false;
        }
        if (segment_fraction >= 0.999) {
          edge_complete = true;
          break;
        }
        RCLCPP_WARN(node_->get_logger(),
                    "Cạnh %zu đạt %.1f%%; đã chạy phần an toàn và tính lại từ trạng thái mới (%d/%d).",
                    i + 1, segment_fraction * 100.0, attempt + 1, max_replans_per_edge);
      }
      if (!edge_complete) {
        RCLCPP_ERROR(node_->get_logger(),
                     "Không hoàn tất được cạnh %zu của nét %zu sau %d lần tính lại.",
                     i + 1, number, max_replans_per_edge);
        return false;
      }
    }
    return true;
  }

  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
  std::string base_frame_, group_, eef_link_;
  double width_, height_, top_clearance_, lift_, step_, velocity_, acceleration_;
  double planning_time_, position_tolerance_, orientation_tolerance_;
  double state_wait_seconds_, workspace_search_step_;
  bool project_marker_to_floor_;
  double floor_marker_z_;
  int planning_attempts_;
  bool execute_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("write_m_node");
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  std::thread spinner([&executor] { executor.spin(); });
  
  std::this_thread::sleep_for(1s);
  WriteM writer(node);
  const bool success = writer.run();

  // Keep the transient-local RViz marker available until the user stops the
  // launch, just as the circle node does.
  if (success) {
    RCLCPP_INFO(node->get_logger(), "Giữ marker /write_m_path; nhấn Ctrl+C để kết thúc.");
    while (rclcpp::ok()) std::this_thread::sleep_for(100ms);
  }

  executor.cancel();
  spinner.join();
  rclcpp::shutdown();
  return success ? 0 : 1;
}
