// Copyright 2025 Lihan Chen
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "pb_omni_pid_pursuit_controller/omni_mpc_pursuit_controller.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

#include "nav2_core/exceptions.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_util/node_utils.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/exceptions.h"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

using nav2_util::declare_parameter_if_not_declared;
using nav2_util::geometry_utils::euclidean_distance;

namespace pb_omni_pid_pursuit_controller
{

void OmniMpcPursuitController::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, std::string name,
  std::shared_ptr<tf2_ros::Buffer> tf, std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  auto node = parent.lock();
  node_ = parent;
  if (!node) {
    throw nav2_core::PlannerException("Unable to lock node!");
  }

  tf_ = tf;
  costmap_ros_ = costmap_ros;
  costmap_ = costmap_ros_->getCostmap();
  plugin_name_ = name;
  logger_ = node->get_logger();
  clock_ = node->get_clock();

  declare_parameter_if_not_declared(
    node, plugin_name_ + ".transform_tolerance", rclcpp::ParameterValue(0.2));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".horizon_steps", rclcpp::ParameterValue(12));
  declare_parameter_if_not_declared(node, plugin_name_ + ".model_dt", rclcpp::ParameterValue(0.05));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".reference_speed", rclcpp::ParameterValue(0.8));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".lookahead_dist", rclcpp::ParameterValue(0.8));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".max_robot_pose_search_dist", rclcpp::ParameterValue(3.0));

  declare_parameter_if_not_declared(node, plugin_name_ + ".vx_min", rclcpp::ParameterValue(-0.2));
  declare_parameter_if_not_declared(node, plugin_name_ + ".vx_max", rclcpp::ParameterValue(1.2));
  declare_parameter_if_not_declared(node, plugin_name_ + ".vy_min", rclcpp::ParameterValue(-0.45));
  declare_parameter_if_not_declared(node, plugin_name_ + ".vy_max", rclcpp::ParameterValue(0.45));
  declare_parameter_if_not_declared(node, plugin_name_ + ".wz_min", rclcpp::ParameterValue(-2.2));
  declare_parameter_if_not_declared(node, plugin_name_ + ".wz_max", rclcpp::ParameterValue(2.2));

  declare_parameter_if_not_declared(node, plugin_name_ + ".ax_max", rclcpp::ParameterValue(2.0));
  declare_parameter_if_not_declared(node, plugin_name_ + ".ay_max", rclcpp::ParameterValue(2.0));
  declare_parameter_if_not_declared(node, plugin_name_ + ".az_max", rclcpp::ParameterValue(3.0));

  declare_parameter_if_not_declared(node, plugin_name_ + ".vx_samples", rclcpp::ParameterValue(7));
  declare_parameter_if_not_declared(node, plugin_name_ + ".vy_samples", rclcpp::ParameterValue(7));
  declare_parameter_if_not_declared(node, plugin_name_ + ".wz_samples", rclcpp::ParameterValue(9));

  declare_parameter_if_not_declared(node, plugin_name_ + ".q_path_x", rclcpp::ParameterValue(2.0));
  declare_parameter_if_not_declared(node, plugin_name_ + ".q_path_y", rclcpp::ParameterValue(12.0));
  declare_parameter_if_not_declared(node, plugin_name_ + ".q_yaw", rclcpp::ParameterValue(6.0));
  declare_parameter_if_not_declared(node, plugin_name_ + ".q_terminal", rclcpp::ParameterValue(3.0));
  declare_parameter_if_not_declared(node, plugin_name_ + ".r_control", rclcpp::ParameterValue(0.2));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".r_delta_control", rclcpp::ParameterValue(2.0));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".collision_check_step", rclcpp::ParameterValue(2));

  double transform_tolerance = 0.2;
  node->get_parameter(plugin_name_ + ".transform_tolerance", transform_tolerance);
  node->get_parameter(plugin_name_ + ".horizon_steps", horizon_steps_);
  node->get_parameter(plugin_name_ + ".model_dt", model_dt_);
  node->get_parameter(plugin_name_ + ".reference_speed", reference_speed_);
  node->get_parameter(plugin_name_ + ".lookahead_dist", lookahead_dist_);
  node->get_parameter(plugin_name_ + ".max_robot_pose_search_dist", max_robot_pose_search_dist_);
  node->get_parameter(plugin_name_ + ".vx_min", vx_min_);
  node->get_parameter(plugin_name_ + ".vx_max", vx_max_);
  node->get_parameter(plugin_name_ + ".vy_min", vy_min_);
  node->get_parameter(plugin_name_ + ".vy_max", vy_max_);
  node->get_parameter(plugin_name_ + ".wz_min", wz_min_);
  node->get_parameter(plugin_name_ + ".wz_max", wz_max_);
  node->get_parameter(plugin_name_ + ".ax_max", ax_max_);
  node->get_parameter(plugin_name_ + ".ay_max", ay_max_);
  node->get_parameter(plugin_name_ + ".az_max", az_max_);
  node->get_parameter(plugin_name_ + ".vx_samples", vx_samples_);
  node->get_parameter(plugin_name_ + ".vy_samples", vy_samples_);
  node->get_parameter(plugin_name_ + ".wz_samples", wz_samples_);
  node->get_parameter(plugin_name_ + ".q_path_x", q_path_x_);
  node->get_parameter(plugin_name_ + ".q_path_y", q_path_y_);
  node->get_parameter(plugin_name_ + ".q_yaw", q_yaw_);
  node->get_parameter(plugin_name_ + ".q_terminal", q_terminal_);
  node->get_parameter(plugin_name_ + ".r_control", r_control_);
  node->get_parameter(plugin_name_ + ".r_delta_control", r_delta_control_);
  node->get_parameter(plugin_name_ + ".collision_check_step", collision_check_step_);

  double controller_frequency = 20.0;
  node->get_parameter("controller_frequency", controller_frequency);
  control_duration_ = controller_frequency > 0.0 ? 1.0 / controller_frequency : model_dt_;

  horizon_steps_ = std::max(1, horizon_steps_);
  model_dt_ = std::max(0.01, model_dt_);
  reference_speed_ = std::max(0.0, reference_speed_);
  lookahead_dist_ = std::max(0.05, lookahead_dist_);
  max_robot_pose_search_dist_ = std::max(0.1, max_robot_pose_search_dist_);
  ax_max_ = std::max(0.01, std::abs(ax_max_));
  ay_max_ = std::max(0.01, std::abs(ay_max_));
  az_max_ = std::max(0.01, std::abs(az_max_));
  vx_samples_ = std::max(1, vx_samples_);
  vy_samples_ = std::max(1, vy_samples_);
  wz_samples_ = std::max(1, wz_samples_);
  collision_check_step_ = std::max(1, collision_check_step_);
  transform_tolerance_ = tf2::durationFromSec(transform_tolerance);

  RCLCPP_INFO(
    logger_, "Configured Omni MPC pursuit controller %s with horizon %d x %.3fs",
    plugin_name_.c_str(), horizon_steps_, model_dt_);
}

void OmniMpcPursuitController::cleanup()
{
  RCLCPP_INFO(logger_, "Cleaning up controller: %s", plugin_name_.c_str());
}

void OmniMpcPursuitController::activate()
{
  last_cmd_ = Command{};
  have_last_cmd_ = false;
  RCLCPP_INFO(logger_, "Activating controller: %s", plugin_name_.c_str());
}

void OmniMpcPursuitController::deactivate()
{
  last_cmd_ = Command{};
  have_last_cmd_ = false;
  RCLCPP_INFO(logger_, "Deactivating controller: %s", plugin_name_.c_str());
}

geometry_msgs::msg::TwistStamped OmniMpcPursuitController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose, const geometry_msgs::msg::Twist & velocity,
  nav2_core::GoalChecker * /*goal_checker*/)
{
  std::lock_guard<std::mutex> lock(mutex_);

  auto * costmap = costmap_ros_->getCostmap();
  std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> costmap_lock(*(costmap->getMutex()));

  nav_msgs::msg::Path transformed_plan;
  try {
    transformed_plan = transformGlobalPlan(pose);
  } catch (const std::exception & ex) {
    RCLCPP_WARN_THROTTLE(
      logger_, *clock_, 1000, "MPC cannot transform plan yet: %s. Commanding zero velocity.",
      ex.what());
    last_cmd_ = Command{};
    have_last_cmd_ = true;
    return makeZeroCommand(pose);
  }

  auto command = chooseBestCommand(transformed_plan, velocity);
  last_cmd_ = command;
  have_last_cmd_ = true;
  return makeCommandMsg(pose, command);
}

void OmniMpcPursuitController::setPlan(const nav_msgs::msg::Path & path)
{
  std::lock_guard<std::mutex> lock(mutex_);
  global_plan_ = path;
}

void OmniMpcPursuitController::setSpeedLimit(
  const double & /*speed_limit*/, const bool & /*percentage*/)
{
  RCLCPP_WARN_THROTTLE(
    logger_, *clock_, 5000, "Speed limit is not implemented in OmniMpcPursuitController.");
}

nav_msgs::msg::Path OmniMpcPursuitController::transformGlobalPlan(
  const geometry_msgs::msg::PoseStamped & pose)
{
  if (global_plan_.poses.empty()) {
    throw nav2_core::PlannerException("Received plan with zero length");
  }

  geometry_msgs::msg::PoseStamped robot_pose;
  if (!transformPose(global_plan_.header.frame_id, pose, robot_pose)) {
    throw nav2_core::PlannerException("Unable to transform robot pose into global plan's frame");
  }

  const double max_costmap_extent = getCostmapMaxExtent();
  auto closest_pose_upper_bound = nav2_util::geometry_utils::first_after_integrated_distance(
    global_plan_.poses.begin(), global_plan_.poses.end(), max_robot_pose_search_dist_);

  auto transformation_begin = nav2_util::geometry_utils::min_by(
    global_plan_.poses.begin(), closest_pose_upper_bound,
    [&robot_pose](const geometry_msgs::msg::PoseStamped & ps) {
      return euclidean_distance(robot_pose, ps);
    });

  if (transformation_begin == global_plan_.poses.end()) {
    throw nav2_core::PlannerException("Cannot find closest pose on global plan");
  }

  auto transformation_end = std::find_if(
    transformation_begin, global_plan_.poses.end(),
    [&](const auto & path_pose) {
      return euclidean_distance(path_pose, robot_pose) > max_costmap_extent;
    });

  if (transformation_end == transformation_begin) {
    transformation_end = std::next(transformation_begin);
  }

  auto transform_global_pose_to_local = [&](const auto & global_plan_pose) {
    geometry_msgs::msg::PoseStamped stamped_pose;
    geometry_msgs::msg::PoseStamped transformed_pose;
    stamped_pose.header.frame_id = global_plan_.header.frame_id;
    stamped_pose.header.stamp = robot_pose.header.stamp;
    stamped_pose.pose = global_plan_pose.pose;
    if (!transformPose(costmap_ros_->getBaseFrameID(), stamped_pose, transformed_pose)) {
      throw nav2_core::PlannerException("Unable to transform path pose into robot frame");
    }
    transformed_pose.pose.position.z = 0.0;
    return transformed_pose;
  };

  nav_msgs::msg::Path transformed_plan;
  std::transform(
    transformation_begin, transformation_end, std::back_inserter(transformed_plan.poses),
    transform_global_pose_to_local);
  transformed_plan.header.frame_id = costmap_ros_->getBaseFrameID();
  transformed_plan.header.stamp = robot_pose.header.stamp;

  global_plan_.poses.erase(global_plan_.poses.begin(), transformation_begin);

  if (transformed_plan.poses.empty()) {
    throw nav2_core::PlannerException("Resulting plan has 0 poses in it.");
  }

  return transformed_plan;
}

bool OmniMpcPursuitController::transformPose(
  const std::string & frame, const geometry_msgs::msg::PoseStamped & in_pose,
  geometry_msgs::msg::PoseStamped & out_pose) const
{
  if (in_pose.header.frame_id == frame) {
    out_pose = in_pose;
    return true;
  }

  try {
    tf_->transform(in_pose, out_pose, frame, transform_tolerance_);
    return true;
  } catch (tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(logger_, *clock_, 1000, "Exception in transformPose: %s", ex.what());
  }
  return false;
}

double OmniMpcPursuitController::getCostmapMaxExtent() const
{
  const double max_costmap_dim_meters =
    std::max(costmap_->getSizeInMetersX(), costmap_->getSizeInMetersY());
  return max_costmap_dim_meters / 2.0;
}

std::vector<double> OmniMpcPursuitController::calculateCumulativeDistances(
  const nav_msgs::msg::Path & path) const
{
  std::vector<double> distances;
  distances.reserve(path.poses.size());
  distances.push_back(0.0);
  for (size_t i = 1; i < path.poses.size(); ++i) {
    const auto & prev = path.poses[i - 1].pose.position;
    const auto & curr = path.poses[i].pose.position;
    distances.push_back(distances.back() + std::hypot(curr.x - prev.x, curr.y - prev.y));
  }
  return distances;
}

OmniMpcPursuitController::Reference OmniMpcPursuitController::getReferenceAtDistance(
  const nav_msgs::msg::Path & path, const std::vector<double> & cumulative_distances,
  double target_distance) const
{
  if (path.poses.size() == 1) {
    const auto & pose = path.poses.front().pose;
    return {pose.position.x, pose.position.y, tf2::getYaw(pose.orientation)};
  }

  if (target_distance <= 0.0) {
    return makeReferenceFromSegment(path.poses[0], path.poses[1], 0.0);
  }

  for (size_t i = 1; i < path.poses.size(); ++i) {
    if (cumulative_distances[i] >= target_distance) {
      const double segment_length = cumulative_distances[i] - cumulative_distances[i - 1];
      const double ratio = segment_length > 1.0e-6
                             ? (target_distance - cumulative_distances[i - 1]) / segment_length
                             : 1.0;
      return makeReferenceFromSegment(path.poses[i - 1], path.poses[i], clampValue(ratio, 0.0, 1.0));
    }
  }

  return makeReferenceFromSegment(
    path.poses[path.poses.size() - 2], path.poses[path.poses.size() - 1], 1.0);
}

OmniMpcPursuitController::Reference OmniMpcPursuitController::makeReferenceFromSegment(
  const geometry_msgs::msg::PoseStamped & start, const geometry_msgs::msg::PoseStamped & end,
  double ratio) const
{
  const auto & p0 = start.pose.position;
  const auto & p1 = end.pose.position;
  const double x = p0.x + ratio * (p1.x - p0.x);
  const double y = p0.y + ratio * (p1.y - p0.y);
  const double dx = p1.x - p0.x;
  const double dy = p1.y - p0.y;
  const double yaw = std::hypot(dx, dy) > 1.0e-6 ? std::atan2(dy, dx) : tf2::getYaw(end.pose.orientation);
  return {x, y, yaw};
}

std::vector<double> OmniMpcPursuitController::sampleRange(
  double lower, double upper, int samples) const
{
  if (upper < lower) {
    std::swap(lower, upper);
  }

  if (samples <= 1 || std::abs(upper - lower) < 1.0e-9) {
    return {0.5 * (lower + upper)};
  }

  std::vector<double> values;
  values.reserve(samples);
  const double step = (upper - lower) / static_cast<double>(samples - 1);
  for (int i = 0; i < samples; ++i) {
    values.push_back(lower + step * static_cast<double>(i));
  }
  return values;
}

OmniMpcPursuitController::Command OmniMpcPursuitController::chooseBestCommand(
  const nav_msgs::msg::Path & transformed_plan, const geometry_msgs::msg::Twist & velocity)
{
  Command previous_cmd = have_last_cmd_
                           ? last_cmd_
                           : Command{velocity.linear.x, velocity.linear.y, velocity.angular.z};
  previous_cmd.vx = clampValue(previous_cmd.vx, vx_min_, vx_max_);
  previous_cmd.vy = clampValue(previous_cmd.vy, vy_min_, vy_max_);
  previous_cmd.wz = clampValue(previous_cmd.wz, wz_min_, wz_max_);

  const auto vx_values = sampleRange(
    std::max(vx_min_, previous_cmd.vx - ax_max_ * control_duration_),
    std::min(vx_max_, previous_cmd.vx + ax_max_ * control_duration_), vx_samples_);
  const auto vy_values = sampleRange(
    std::max(vy_min_, previous_cmd.vy - ay_max_ * control_duration_),
    std::min(vy_max_, previous_cmd.vy + ay_max_ * control_duration_), vy_samples_);
  const auto wz_values = sampleRange(
    std::max(wz_min_, previous_cmd.wz - az_max_ * control_duration_),
    std::min(wz_max_, previous_cmd.wz + az_max_ * control_duration_), wz_samples_);

  const auto cumulative_distances = calculateCumulativeDistances(transformed_plan);
  double best_score = std::numeric_limits<double>::infinity();
  Command best_command{};
  bool found_valid_command = false;

  for (const auto vx : vx_values) {
    for (const auto vy : vy_values) {
      for (const auto wz : wz_values) {
        const Command command{vx, vy, wz};
        bool collision_free = true;
        const double score =
          scoreCommand(command, transformed_plan, cumulative_distances, previous_cmd, collision_free);
        if (collision_free && score < best_score) {
          best_score = score;
          best_command = command;
          found_valid_command = true;
        }
      }
    }
  }

  if (!found_valid_command) {
    RCLCPP_WARN_THROTTLE(
      logger_, *clock_, 1000, "MPC found no collision-free candidate. Commanding zero velocity.");
    return Command{};
  }

  return best_command;
}

double OmniMpcPursuitController::scoreCommand(
  const Command & command, const nav_msgs::msg::Path & transformed_plan,
  const std::vector<double> & cumulative_distances, const Command & previous_command,
  bool & collision_free) const
{
  State state;
  double cost = r_control_ * (command.vx * command.vx + command.vy * command.vy +
                              0.25 * command.wz * command.wz);
  cost += r_delta_control_ *
          ((command.vx - previous_command.vx) * (command.vx - previous_command.vx) +
           (command.vy - previous_command.vy) * (command.vy - previous_command.vy) +
           0.25 * (command.wz - previous_command.wz) * (command.wz - previous_command.wz));

  for (int step = 1; step <= horizon_steps_; ++step) {
    state = predictNextState(state, command);
    if ((step % collision_check_step_ == 0 || step == horizon_steps_) && !isStateCollisionFree(state)) {
      collision_free = false;
      return std::numeric_limits<double>::infinity();
    }

    const double target_distance =
      std::min(lookahead_dist_, reference_speed_ * model_dt_ * static_cast<double>(step));
    const auto ref = getReferenceAtDistance(transformed_plan, cumulative_distances, target_distance);

    const double dx = state.x - ref.x;
    const double dy = state.y - ref.y;
    const double cos_yaw = std::cos(ref.yaw);
    const double sin_yaw = std::sin(ref.yaw);
    const double longitudinal_error = cos_yaw * dx + sin_yaw * dy;
    const double lateral_error = -sin_yaw * dx + cos_yaw * dy;
    const double yaw_error = normalizeAngle(state.yaw - ref.yaw);
    const double terminal_scale = step == horizon_steps_ ? q_terminal_ : 1.0;

    cost += terminal_scale *
            (q_path_x_ * longitudinal_error * longitudinal_error +
             q_path_y_ * lateral_error * lateral_error + q_yaw_ * yaw_error * yaw_error);
  }

  collision_free = true;
  return cost;
}

OmniMpcPursuitController::State OmniMpcPursuitController::predictNextState(
  const State & state, const Command & command) const
{
  State next;
  next.x = state.x + model_dt_ * (std::cos(state.yaw) * command.vx - std::sin(state.yaw) * command.vy);
  next.y = state.y + model_dt_ * (std::sin(state.yaw) * command.vx + std::cos(state.yaw) * command.vy);
  next.yaw = normalizeAngle(state.yaw + model_dt_ * command.wz);
  return next;
}

bool OmniMpcPursuitController::isStateCollisionFree(const State & state) const
{
  geometry_msgs::msg::PoseStamped local_pose;
  geometry_msgs::msg::PoseStamped costmap_pose;
  local_pose.header.frame_id = costmap_ros_->getBaseFrameID();
  local_pose.header.stamp.sec = 0;
  local_pose.header.stamp.nanosec = 0;
  local_pose.pose.position.x = state.x;
  local_pose.pose.position.y = state.y;
  local_pose.pose.position.z = 0.0;
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, state.yaw);
  local_pose.pose.orientation = tf2::toMsg(q);

  if (!transformPose(costmap_ros_->getGlobalFrameID(), local_pose, costmap_pose)) {
    return false;
  }

  unsigned int mx = 0;
  unsigned int my = 0;
  if (!costmap_->worldToMap(costmap_pose.pose.position.x, costmap_pose.pose.position.y, mx, my)) {
    return false;
  }

  return costmap_->getCost(mx, my) < nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
}

geometry_msgs::msg::TwistStamped OmniMpcPursuitController::makeCommandMsg(
  const geometry_msgs::msg::PoseStamped & pose, const Command & command) const
{
  geometry_msgs::msg::TwistStamped cmd_vel;
  cmd_vel.header = pose.header;
  cmd_vel.twist.linear.x = command.vx;
  cmd_vel.twist.linear.y = command.vy;
  cmd_vel.twist.angular.z = command.wz;
  return cmd_vel;
}

geometry_msgs::msg::TwistStamped OmniMpcPursuitController::makeZeroCommand(
  const geometry_msgs::msg::PoseStamped & pose) const
{
  return makeCommandMsg(pose, Command{});
}

double OmniMpcPursuitController::normalizeAngle(double angle) const
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

double OmniMpcPursuitController::clampValue(double value, double lower, double upper) const
{
  if (upper < lower) {
    std::swap(lower, upper);
  }
  return std::min(std::max(value, lower), upper);
}

}  // namespace pb_omni_pid_pursuit_controller

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  pb_omni_pid_pursuit_controller::OmniMpcPursuitController, nav2_core::Controller)
