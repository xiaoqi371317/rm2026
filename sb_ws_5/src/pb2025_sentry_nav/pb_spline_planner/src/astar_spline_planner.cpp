#include "pb_spline_planner/astar_spline_planner.hpp"

#include <algorithm>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "bipedal_wheel_planner/trajectory_generator/backend_tools/TrajectoryOptimizer.hpp"
#include "bipedal_wheel_planner/trajectory_generator/frontend_tools/Astar.hpp"
#include "nav2_core/exceptions.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace pb_spline_planner
{

namespace
{
geometry_msgs::msg::Quaternion yawToQuaternion(double yaw)
{
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, yaw);
  return tf2::toMsg(q);
}
}  // namespace

void AStarSplinePlanner::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, std::string name,
  std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  parent_ = parent;
  auto node = parent.lock();
  if (!node) {
    throw nav2_core::PlannerException("Unable to lock node in AStarSplinePlanner");
  }

  name_ = name;
  tf_ = tf;
  costmap_ros_ = costmap_ros;
  costmap_ = costmap_ros_->getCostmap();
  global_frame_ = costmap_ros_->getGlobalFrameID();
  clock_ = node->get_clock();
  logger_ = node->get_logger();

  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".safe_distance", rclcpp::ParameterValue(hard_safe_distance_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".hard_safe_distance", rclcpp::ParameterValue(hard_safe_distance_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".preferred_clearance", rclcpp::ParameterValue(preferred_clearance_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".astar_clearance_weight", rclcpp::ParameterValue(astar_clearance_weight_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".spline_clearance_weight", rclcpp::ParameterValue(spline_clearance_weight_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".collision_weight", rclcpp::ParameterValue(collision_weight_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".allow_unknown", rclcpp::ParameterValue(allow_unknown_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".lethal_cost_threshold", rclcpp::ParameterValue(lethal_cost_threshold_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".astar_timeout_ms", rclcpp::ParameterValue(astar_timeout_ms_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".max_velocity", rclcpp::ParameterValue(max_velocity_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".max_acceleration", rclcpp::ParameterValue(max_acceleration_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".trajectory_sample_dt", rclcpp::ParameterValue(trajectory_sample_dt_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".fallback_to_astar_path", rclcpp::ParameterValue(fallback_to_astar_path_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".enable_plan_cache", rclcpp::ParameterValue(enable_plan_cache_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".optimizer_max_iterations", rclcpp::ParameterValue(optimizer_max_iterations_));

  node->get_parameter(name_ + ".safe_distance", hard_safe_distance_);
  node->get_parameter(name_ + ".hard_safe_distance", hard_safe_distance_);
  node->get_parameter(name_ + ".preferred_clearance", preferred_clearance_);
  node->get_parameter(name_ + ".astar_clearance_weight", astar_clearance_weight_);
  node->get_parameter(name_ + ".spline_clearance_weight", spline_clearance_weight_);
  node->get_parameter(name_ + ".collision_weight", collision_weight_);
  node->get_parameter(name_ + ".allow_unknown", allow_unknown_);
  node->get_parameter(name_ + ".lethal_cost_threshold", lethal_cost_threshold_);
  node->get_parameter(name_ + ".astar_timeout_ms", astar_timeout_ms_);
  node->get_parameter(name_ + ".max_velocity", max_velocity_);
  node->get_parameter(name_ + ".max_acceleration", max_acceleration_);
  node->get_parameter(name_ + ".trajectory_sample_dt", trajectory_sample_dt_);
  node->get_parameter(name_ + ".fallback_to_astar_path", fallback_to_astar_path_);
  node->get_parameter(name_ + ".enable_plan_cache", enable_plan_cache_);
  node->get_parameter(name_ + ".optimizer_max_iterations", optimizer_max_iterations_);

  astar_path_pub_ = node->create_publisher<nav_msgs::msg::Path>(name_ + "/astar_path", 1);
  spline_path_pub_ = node->create_publisher<nav_msgs::msg::Path>(name_ + "/spline_path", 1);

  RCLCPP_INFO(
    logger_,
    "Configured %s with hard_safe_distance %.2f, preferred_clearance %.2f, "
    "astar_clearance_weight %.2f, allow_unknown %s, lethal_cost_threshold %d",
    name_.c_str(), hard_safe_distance_, preferred_clearance_, astar_clearance_weight_,
    allow_unknown_ ? "true" : "false", lethal_cost_threshold_);
}

void AStarSplinePlanner::cleanup()
{
  astar_path_pub_.reset();
  spline_path_pub_.reset();
}

void AStarSplinePlanner::activate()
{
  if (astar_path_pub_) {
    astar_path_pub_->on_activate();
  }
  if (spline_path_pub_) {
    spline_path_pub_->on_activate();
  }
}

void AStarSplinePlanner::deactivate()
{
  if (astar_path_pub_) {
    astar_path_pub_->on_deactivate();
  }
  if (spline_path_pub_) {
    spline_path_pub_->on_deactivate();
  }
}

nav_msgs::msg::Path AStarSplinePlanner::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal)
{
  if (!costmap_) {
    throw nav2_core::PlannerException("AStarSplinePlanner costmap is not initialized");
  }

  auto start_global = transformToGlobalFrame(start);
  auto goal_global = transformToGlobalFrame(goal);

  if (!isPoseValidInCostmap(start_global)) {
    throw nav2_core::PlannerException("Start pose is outside map bounds or in collision");
  }
  if (!isPoseValidInCostmap(goal_global)) {
    throw nav2_core::PlannerException("Goal pose is outside map bounds or in collision");
  }

  auto header = start_global.header;
  header.stamp = clock_->now();
  header.frame_id = global_frame_;

  auto grid_map = buildGridMapFromCostmap();

  path_planning::AStar astar(*grid_map, hard_safe_distance_);
  astar.setMaxVelocity(max_velocity_);
  astar.setMaxAcceleration(max_acceleration_);
  astar.setPreferredClearance(preferred_clearance_);
  astar.setClearanceWeight(astar_clearance_weight_);

  const Eigen::Vector2d start_pos(start_global.pose.position.x, start_global.pose.position.y);
  const Eigen::Vector2d goal_pos(goal_global.pose.position.x, goal_global.pose.position.y);
  auto astar_result = astar.planWithPostProcessing(start_pos, goal_pos, astar_timeout_ms_);

  if (astar_result.optimized_path.size() < 2) {
    throw nav2_core::PlannerException("A* failed to find a path");
  }

  auto astar_path = makePathMessage(astar_result.optimized_path, header, goal_global);
  if (astar_path_pub_ && astar_path_pub_->is_activated()) {
    astar_path_pub_->publish(astar_path);
  }

  TrajOpt::TrajectoryParams params;
  params.total_len = std::max(astar_result.total_length, costmap_->getResolution());
  params.total_time = std::max(astar_result.total_time, trajectory_sample_dt_);
  params.piece_len = std::max(costmap_->getResolution(), params.total_len / 8.0);
  params.max_v = max_velocity_;
  params.safe_threshold = hard_safe_distance_;
  params.preferred_clearance = preferred_clearance_;
  params.rho_clearance = spline_clearance_weight_;
  params.rho_collision = collision_weight_;
  params.max_iter = optimizer_max_iterations_;

  TrajOpt::TrajectoryOptimizer optimizer(grid_map, astar_result.optimized_path, params);
  optimizer.plan();
  auto spline_points = optimizer.sampleTrajectory(trajectory_sample_dt_);

  nav_msgs::msg::Path result;
  if (spline_points.size() >= 2) {
    result = makePathMessage(spline_points, header, goal_global);
  } else if (fallback_to_astar_path_) {
    RCLCPP_WARN(logger_, "Spline optimization failed; falling back to A* path");
    result = astar_path;
  } else {
    throw nav2_core::PlannerException("Spline optimization failed");
  }

  if (spline_path_pub_ && spline_path_pub_->is_activated()) {
    spline_path_pub_->publish(result);
  }

  return result;
}

geometry_msgs::msg::PoseStamped AStarSplinePlanner::transformToGlobalFrame(
  const geometry_msgs::msg::PoseStamped & pose) const
{
  if (pose.header.frame_id == global_frame_ || pose.header.frame_id.empty()) {
    auto transformed = pose;
    transformed.header.frame_id = global_frame_;
    return transformed;
  }

  try {
    return tf_->transform(pose, global_frame_, tf2::durationFromSec(0.1));
  } catch (const tf2::TransformException & ex) {
    throw nav2_core::PlannerException(
            std::string("Failed to transform pose to global frame: ") + ex.what());
  }
}

std::shared_ptr<grid_map::GridMap> AStarSplinePlanner::buildGridMapFromCostmap() const
{
  auto grid_map = std::make_shared<grid_map::GridMap>();
  const auto size_x = static_cast<int>(costmap_->getSizeInCellsX());
  const auto size_y = static_cast<int>(costmap_->getSizeInCellsY());
  const double resolution = costmap_->getResolution();
  const Eigen::Vector2d origin(costmap_->getOriginX(), costmap_->getOriginY());

  grid_map->init(size_x * resolution, size_y * resolution, resolution, origin);

  grid_map::RowMatrixXi occupancy(size_x, size_y);
  for (int x = 0; x < size_x; ++x) {
    for (int y = 0; y < size_y; ++y) {
      const auto cost = costmap_->getCost(x, y);
      const bool is_unknown = cost == nav2_costmap_2d::NO_INFORMATION;
      const bool blocked = is_unknown ? !allow_unknown_ : cost >= lethal_cost_threshold_;
      occupancy(x, y) = blocked ? 1 : 0;
    }
  }

  grid_map->setMap(occupancy);
  return grid_map;
}

bool AStarSplinePlanner::isPoseValidInCostmap(const geometry_msgs::msg::PoseStamped & pose) const
{
  unsigned int mx = 0;
  unsigned int my = 0;
  if (!costmap_->worldToMap(pose.pose.position.x, pose.pose.position.y, mx, my)) {
    return false;
  }

  const auto cost = costmap_->getCost(mx, my);
  if (cost == nav2_costmap_2d::NO_INFORMATION) {
    return allow_unknown_;
  }
  return cost < lethal_cost_threshold_;
}

nav_msgs::msg::Path AStarSplinePlanner::makePathMessage(
  const std::vector<Eigen::Vector2d> & points, const std_msgs::msg::Header & header,
  const geometry_msgs::msg::PoseStamped & goal) const
{
  nav_msgs::msg::Path path;
  path.header = header;
  path.poses.reserve(points.size());

  for (const auto & point : points) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = header;
    pose.pose.position.x = point.x();
    pose.pose.position.y = point.y();
    pose.pose.position.z = 0.0;
    pose.pose.orientation.w = 1.0;
    path.poses.push_back(pose);
  }

  updatePathOrientations(path, goal);
  return path;
}

void AStarSplinePlanner::updatePathOrientations(
  nav_msgs::msg::Path & path, const geometry_msgs::msg::PoseStamped & goal) const
{
  if (path.poses.empty()) {
    return;
  }

  for (size_t i = 0; i + 1 < path.poses.size(); ++i) {
    const auto & current = path.poses[i].pose.position;
    const auto & next = path.poses[i + 1].pose.position;
    const double yaw = std::atan2(next.y - current.y, next.x - current.x);
    path.poses[i].pose.orientation = yawToQuaternion(yaw);
  }

  path.poses.back().pose.orientation = goal.pose.orientation;
}

}  // namespace pb_spline_planner

PLUGINLIB_EXPORT_CLASS(pb_spline_planner::AStarSplinePlanner, nav2_core::GlobalPlanner)
