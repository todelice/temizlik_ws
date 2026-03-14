#include <algorithm>
#include <cctype>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <fields2cover.h>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav2_msgs/action/follow_waypoints.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

class F2CNav2WaypointNode : public rclcpp::Node {
public:
  using FollowWaypoints = nav2_msgs::action::FollowWaypoints;
  using GoalHandleFollowWaypoints = rclcpp_action::ClientGoalHandle<FollowWaypoints>;

  F2CNav2WaypointNode()
  : Node("f2c_nav2_waypoint_node") {
    declare_parameter<std::string>("frame_id", "map");
    declare_parameter<std::string>(
      "field_wkt", "POLYGON((-1.0 -1.0, -1.0 1.0, 1.0 1.0, 1.0 -1.0, -1.0 -1.0))");
    declare_parameter<std::vector<std::string>>("exclude_wkts", std::vector<std::string>{});
    declare_parameter<std::string>("action_name", "/follow_waypoints");
    declare_parameter<std::string>("pp_alg", "DUBINS");
    declare_parameter<double>("robot_width", 0.18);
    declare_parameter<double>("coverage_width", 0.18);
    declare_parameter<double>("max_curv", 1.0);
    declare_parameter<double>("max_diff_curv", 0.3);
    declare_parameter<double>("waypoint_spacing", 0.25);
    declare_parameter<bool>("auto_start", false);

    frame_id_ = get_parameter("frame_id").as_string();
    field_wkt_ = get_parameter("field_wkt").as_string();
    get_parameter("exclude_wkts", exclude_wkts_);
    action_name_ = get_parameter("action_name").as_string();
    pp_alg_ = get_parameter("pp_alg").as_string();
    robot_width_ = get_parameter("robot_width").as_double();
    coverage_width_ = get_parameter("coverage_width").as_double();
    max_curv_ = get_parameter("max_curv").as_double();
    max_diff_curv_ = get_parameter("max_diff_curv").as_double();
    waypoint_spacing_ = get_parameter("waypoint_spacing").as_double();
    auto_start_ = get_parameter("auto_start").as_bool();

    if (waypoint_spacing_ <= 0.0) {
      throw std::runtime_error("Parameter 'waypoint_spacing' must be > 0.0");
    }

    const auto latched_qos = rclcpp::QoS(1).reliable().transient_local();
    path_pub_ = create_publisher<nav_msgs::msg::Path>("coverage_path", latched_qos);
    markers_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      "coverage_markers", latched_qos);
    action_client_ = rclcpp_action::create_client<FollowWaypoints>(this, action_name_);
    start_service_ = create_service<std_srvs::srv::Trigger>(
      "start_coverage",
      std::bind(
        &F2CNav2WaypointNode::handle_start_coverage, this, std::placeholders::_1,
        std::placeholders::_2));

    generate_waypoints();
    publish_path();
    publish_markers();

    if (auto_start_) {
      send_timer_ = create_wall_timer(
        std::chrono::seconds(1), std::bind(&F2CNav2WaypointNode::try_send_waypoints, this));
    } else {
      RCLCPP_INFO(
        get_logger(),
        "auto_start is false. Generated %zu waypoints. Call '/start_coverage' service to send.",
        waypoints_.size());
    }
  }

private:
  std::string to_upper(std::string value) {
    std::transform(
      value.begin(), value.end(), value.begin(),
      [](unsigned char c) { return static_cast<char>(std::toupper(c)); });
    return value;
  }

  std::shared_ptr<f2c::pp::TurningBase> create_turning_algorithm(const std::string &alg_name) {
    const auto alg = to_upper(alg_name);
    if (alg == "DUBINS") {
      return std::make_shared<f2c::pp::DubinsCurves>();
    }
    if (alg == "DUBINS_CC") {
      return std::make_shared<f2c::pp::DubinsCurvesCC>();
    }
    if (alg == "REEDS_SHEPP") {
      return std::make_shared<f2c::pp::ReedsSheppCurves>();
    }
    if (alg == "REEDS_SHEPP_HC") {
      return std::make_shared<f2c::pp::ReedsSheppCurvesHC>();
    }

    RCLCPP_WARN(get_logger(), "Unknown pp_alg '%s'. Falling back to DUBINS.", alg_name.c_str());
    return std::make_shared<f2c::pp::DubinsCurves>();
  }

  geometry_msgs::msg::PoseStamped to_pose(double x, double y, double yaw) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = frame_id_;
    pose.pose.position.x = x;
    pose.pose.position.y = y;
    pose.pose.position.z = 0.0;

    const double half_yaw = yaw * 0.5;
    pose.pose.orientation.x = 0.0;
    pose.pose.orientation.y = 0.0;
    pose.pose.orientation.z = std::sin(half_yaw);
    pose.pose.orientation.w = std::cos(half_yaw);

    return pose;
  }

  geometry_msgs::msg::Point to_point_msg(double x, double y, double z = 0.0) {
    geometry_msgs::msg::Point p;
    p.x = x;
    p.y = y;
    p.z = z;
    return p;
  }

  double planar_distance(
    const geometry_msgs::msg::PoseStamped &a, const geometry_msgs::msg::PoseStamped &b) {
    const double dx = a.pose.position.x - b.pose.position.x;
    const double dy = a.pose.position.y - b.pose.position.y;
    return std::hypot(dx, dy);
  }

  void append_cell_ring(
    visualization_msgs::msg::Marker &marker, const F2CCell &cell, bool close_ring = true) {
    const auto ring = cell.getExteriorRing();
    if (ring.size() == 0) {
      return;
    }

    for (size_t i = 0; i < ring.size(); ++i) {
      const auto p = ring.getGeometry(i);
      marker.points.push_back(to_point_msg(p.getX(), p.getY()));
    }

    if (close_ring) {
      const auto p0 = ring.getGeometry(0);
      marker.points.push_back(to_point_msg(p0.getX(), p0.getY()));
    }
  }

  void generate_waypoints() {
    F2CRobot robot(robot_width_, coverage_width_);
    if (max_curv_ > 0.0) {
      robot.max_icc = max_curv_;
    }
    robot.linear_curv_change = max_diff_curv_;

    field_cell_.importFromWkt(field_wkt_);
    if (field_cell_.isEmpty()) {
      throw std::runtime_error("Parsed field polygon is empty. Check field_wkt parameter.");
    }

    exclusion_cells_.clear();
    work_cells_ = F2CCells(field_cell_);

    for (const auto &exclude_wkt : exclude_wkts_) {
      F2CCell ex_cell;
      ex_cell.importFromWkt(exclude_wkt);
      if (ex_cell.isEmpty()) {
        RCLCPP_WARN(get_logger(), "Skipping empty exclusion polygon WKT.");
        continue;
      }
      exclusion_cells_.push_back(ex_cell);
      const auto diff_cells = work_cells_.Difference(F2CCells(ex_cell));
      if (diff_cells.size() > 0) {
        work_cells_ = diff_cells;
      } else {
        RCLCPP_WARN(
          get_logger(),
          "Ignoring an exclusion polygon because it would remove the whole working area.");
      }
    }

    f2c::sg::BruteForce swath_generator;
    f2c::obj::SwathLength swath_objective;
    f2c::rp::SnakeOrder route_generator;
    f2c::pp::PathPlanning path_planner;
    path_planner.turn_point_dist = std::max(0.05, waypoint_spacing_ * 0.5);
    auto turning_model = create_turning_algorithm(pp_alg_);

    F2CPath path;
    bool has_any_path = false;

    for (size_t i = 0; i < work_cells_.size(); ++i) {
      const auto work_cell = work_cells_.getGeometry(i);
      if (work_cell.isEmpty()) {
        continue;
      }

      auto swaths = swath_generator.generateBestSwaths(swath_objective, robot.op_width, work_cell);
      if (swaths.size() == 0) {
        RCLCPP_WARN(get_logger(), "No swaths generated for work cell %zu, skipping.", i);
        continue;
      }

      swaths = route_generator.genSortedSwaths(swaths);
      auto cell_path = path_planner.searchBestPath(robot, swaths, *turning_model);
      if (cell_path.size() == 0) {
        RCLCPP_WARN(get_logger(), "No path generated for work cell %zu, skipping.", i);
        continue;
      }

      if (!has_any_path) {
        path = cell_path;
        has_any_path = true;
      } else {
        path += cell_path;
      }
    }

    if (!has_any_path || path.size() == 0) {
      throw std::runtime_error("Fields2Cover returned an empty path.");
    }

    full_path_poses_.clear();
    waypoints_.clear();
    full_path_poses_.reserve(path.size());
    waypoints_.reserve(path.size());

    geometry_msgs::msg::PoseStamped last_kept;
    bool has_last_kept = false;
    size_t excluded_states = 0;

    for (const auto &state : path.states) {
      if (is_in_exclusion(state.point)) {
        ++excluded_states;
        continue;
      }

      auto pose = to_pose(state.point.getX(), state.point.getY(), state.angle);
      full_path_poses_.push_back(pose);

      if (!has_last_kept || planar_distance(last_kept, pose) >= waypoint_spacing_) {
        waypoints_.push_back(pose);
        last_kept = pose;
        has_last_kept = true;
      }
    }

    if (full_path_poses_.empty()) {
      throw std::runtime_error("All generated path states are inside exclusion polygons.");
    }

    if (!full_path_poses_.empty()) {
      const auto &final_pose = full_path_poses_.back();
      if (waypoints_.empty() || planar_distance(waypoints_.back(), final_pose) > 1e-3) {
        waypoints_.push_back(final_pose);
      }
    }

    RCLCPP_INFO(
      get_logger(),
      "Generated coverage path: %zu kept states (%zu excluded), %zu waypoint goals "
      "(spacing=%.2f m). Exclusions: %zu",
      full_path_poses_.size(), excluded_states, waypoints_.size(), waypoint_spacing_,
      exclusion_cells_.size());
  }

  bool is_in_exclusion(const f2c::types::Point &point) {
    for (const auto &cell : exclusion_cells_) {
      if (cell.isPointIn(point) || cell.isPointInBorder(point)) {
        return true;
      }
    }
    return false;
  }

  void publish_path() {
    nav_msgs::msg::Path path_msg;
    path_msg.header.stamp = now();
    path_msg.header.frame_id = frame_id_;
    path_msg.poses = full_path_poses_;

    for (auto &pose : path_msg.poses) {
      pose.header = path_msg.header;
    }

    path_pub_->publish(path_msg);
  }

  void publish_markers() {
    visualization_msgs::msg::MarkerArray marker_array;

    const auto stamp = now();

    visualization_msgs::msg::Marker field_marker;
    field_marker.header.frame_id = frame_id_;
    field_marker.header.stamp = stamp;
    field_marker.ns = "f2c_field";
    field_marker.id = 0;
    field_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    field_marker.action = visualization_msgs::msg::Marker::ADD;
    field_marker.scale.x = 0.04;
    field_marker.color.r = 0.1f;
    field_marker.color.g = 0.9f;
    field_marker.color.b = 0.1f;
    field_marker.color.a = 1.0f;
    append_cell_ring(field_marker, field_cell_);
    marker_array.markers.push_back(field_marker);

    for (size_t i = 0; i < work_cells_.size(); ++i) {
      visualization_msgs::msg::Marker work_marker;
      work_marker.header.frame_id = frame_id_;
      work_marker.header.stamp = stamp;
      work_marker.ns = "f2c_work_area";
      work_marker.id = static_cast<int>(i);
      work_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
      work_marker.action = visualization_msgs::msg::Marker::ADD;
      work_marker.scale.x = 0.03;
      work_marker.color.r = 0.1f;
      work_marker.color.g = 0.6f;
      work_marker.color.b = 1.0f;
      work_marker.color.a = 1.0f;
      append_cell_ring(work_marker, work_cells_.getGeometry(i));
      marker_array.markers.push_back(work_marker);
    }

    for (size_t i = 0; i < exclusion_cells_.size(); ++i) {
      visualization_msgs::msg::Marker ex_marker;
      ex_marker.header.frame_id = frame_id_;
      ex_marker.header.stamp = stamp;
      ex_marker.ns = "f2c_exclusion";
      ex_marker.id = static_cast<int>(i);
      ex_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
      ex_marker.action = visualization_msgs::msg::Marker::ADD;
      ex_marker.scale.x = 0.05;
      ex_marker.color.r = 1.0f;
      ex_marker.color.g = 0.15f;
      ex_marker.color.b = 0.15f;
      ex_marker.color.a = 1.0f;
      append_cell_ring(ex_marker, exclusion_cells_[i]);
      marker_array.markers.push_back(ex_marker);
    }

    visualization_msgs::msg::Marker waypoint_marker;
    waypoint_marker.header.frame_id = frame_id_;
    waypoint_marker.header.stamp = stamp;
    waypoint_marker.ns = "f2c_waypoints";
    waypoint_marker.id = 0;
    waypoint_marker.type = visualization_msgs::msg::Marker::POINTS;
    waypoint_marker.action = visualization_msgs::msg::Marker::ADD;
    waypoint_marker.scale.x = 0.06;
    waypoint_marker.scale.y = 0.06;
    waypoint_marker.color.r = 1.0f;
    waypoint_marker.color.g = 1.0f;
    waypoint_marker.color.b = 0.0f;
    waypoint_marker.color.a = 1.0f;
    waypoint_marker.points.reserve(waypoints_.size());
    for (const auto &pose : waypoints_) {
      waypoint_marker.points.push_back(
        to_point_msg(pose.pose.position.x, pose.pose.position.y, 0.03));
    }
    marker_array.markers.push_back(waypoint_marker);

    markers_pub_->publish(marker_array);
  }

  void try_send_waypoints() {
    if (goal_sent_) {
      return;
    }

    if (!action_client_->wait_for_action_server(std::chrono::milliseconds(1))) {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Waiting for Nav2 FollowWaypoints action server on '%s'...", action_name_.c_str());
      return;
    }

    send_waypoints_goal();

    if (send_timer_) {
      send_timer_->cancel();
    }
  }

  void handle_start_coverage(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
    if (goal_sent_) {
      response->success = false;
      response->message = "Coverage goal already sent.";
      return;
    }

    if (!action_client_->wait_for_action_server(std::chrono::seconds(1))) {
      response->success = false;
      response->message = "Nav2 FollowWaypoints action server is not available yet.";
      return;
    }

    send_waypoints_goal();
    response->success = true;
    response->message = "Coverage waypoints sent to Nav2.";
  }

  void send_waypoints_goal() {
    FollowWaypoints::Goal goal;
    goal.poses = waypoints_;

    auto options = rclcpp_action::Client<FollowWaypoints>::SendGoalOptions();

    options.goal_response_callback =
      [this](const GoalHandleFollowWaypoints::SharedPtr &goal_handle) {
        if (!goal_handle) {
          RCLCPP_ERROR(get_logger(), "FollowWaypoints goal was rejected by server.");
          goal_sent_ = false;
          return;
        }
        RCLCPP_INFO(get_logger(), "FollowWaypoints goal accepted.");
      };

    options.result_callback =
      [this](const GoalHandleFollowWaypoints::WrappedResult &result) {
        switch (result.code) {
          case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(get_logger(), "Coverage waypoints completed successfully.");
            break;
          case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(get_logger(), "Coverage waypoints aborted by Nav2.");
            break;
          case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_WARN(get_logger(), "Coverage waypoints canceled.");
            break;
          default:
            RCLCPP_ERROR(get_logger(), "Coverage waypoints finished with unknown result code.");
            break;
        }
      };

    goal_sent_ = true;
    action_client_->async_send_goal(goal, options);
    RCLCPP_INFO(get_logger(), "Sent %zu coverage waypoints to Nav2.", waypoints_.size());
  }

  std::string frame_id_;
  std::string field_wkt_;
  std::vector<std::string> exclude_wkts_;
  std::string action_name_;
  std::string pp_alg_;
  double robot_width_ {0.18};
  double coverage_width_ {0.18};
  double max_curv_ {1.0};
  double max_diff_curv_ {0.3};
  double waypoint_spacing_ {0.25};
  bool auto_start_ {false};
  bool goal_sent_ {false};

  F2CCell field_cell_;
  F2CCells work_cells_;
  std::vector<F2CCell> exclusion_cells_;

  std::vector<geometry_msgs::msg::PoseStamped> full_path_poses_;
  std::vector<geometry_msgs::msg::PoseStamped> waypoints_;

  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_pub_;
  rclcpp_action::Client<FollowWaypoints>::SharedPtr action_client_;
  rclcpp::TimerBase::SharedPtr send_timer_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_service_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  try {
    auto node = std::make_shared<F2CNav2WaypointNode>();
    rclcpp::spin(node);
  } catch (const std::exception &e) {
    RCLCPP_FATAL(rclcpp::get_logger("f2c_nav2_waypoint_node"), "%s", e.what());
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}
