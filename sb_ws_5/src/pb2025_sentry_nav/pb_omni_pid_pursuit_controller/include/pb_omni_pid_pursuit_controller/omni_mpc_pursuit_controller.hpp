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

#ifndef PB_OMNI_PID_PURSUIT_CONTROLLER__OMNI_MPC_PURSUIT_CONTROLLER_HPP_
#define PB_OMNI_PID_PURSUIT_CONTROLLER__OMNI_MPC_PURSUIT_CONTROLLER_HPP_

#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav2_core/controller.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "tf2/time.h"
#include "tf2_ros/buffer.h"

namespace pb_omni_pid_pursuit_controller
{

class OmniMpcPursuitController : public nav2_core::Controller
{
public:
  OmniMpcPursuitController() = default;
  ~OmniMpcPursuitController() override = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void cleanup() override;
  void activate() override;
  void deactivate() override;

  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose, const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * goal_checker) override;

  void setPlan(const nav_msgs::msg::Path & path) override;
  void setSpeedLimit(const double & speed_limit, const bool & percentage) override;

private:
  struct Command
  {
    double vx{0.0};
    double vy{0.0};
    double wz{0.0};
  };

  struct State
  {
    double x{0.0};
    double y{0.0};
    double yaw{0.0};
  };

  struct Reference
  {
    double x{0.0};
    double y{0.0};
    double yaw{0.0};
  };

  nav_msgs::msg::Path transformGlobalPlan(const geometry_msgs::msg::PoseStamped & pose);

  bool transformPose(
    const std::string & frame, const geometry_msgs::msg::PoseStamped & in_pose,
    geometry_msgs::msg::PoseStamped & out_pose) const;

  double getCostmapMaxExtent() const;
  std::vector<double> calculateCumulativeDistances(const nav_msgs::msg::Path & path) const;
  Reference getReferenceAtDistance(
    const nav_msgs::msg::Path & path, const std::vector<double> & cumulative_distances,
    double target_distance) const;
  Reference makeReferenceFromSegment(
    const geometry_msgs::msg::PoseStamped & start, const geometry_msgs::msg::PoseStamped & end,
    double ratio) const;

  std::vector<double> sampleRange(double lower, double upper, int samples) const;
  Command chooseBestCommand(
    const nav_msgs::msg::Path & transformed_plan, const geometry_msgs::msg::Twist & velocity);
  double scoreCommand(
    const Command & command, const nav_msgs::msg::Path & transformed_plan,
    const std::vector<double> & cumulative_distances, const Command & previous_command,
    bool & collision_free) const;

  State predictNextState(const State & state, const Command & command) const;
  bool isStateCollisionFree(const State & state) const;
  geometry_msgs::msg::TwistStamped makeCommandMsg(
    const geometry_msgs::msg::PoseStamped & pose, const Command & command) const;
  geometry_msgs::msg::TwistStamped makeZeroCommand(
    const geometry_msgs::msg::PoseStamped & pose) const;

  double normalizeAngle(double angle) const;
  double clampValue(double value, double lower, double upper) const;

  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  nav2_costmap_2d::Costmap2D * costmap_{nullptr};
  std::string plugin_name_;
  rclcpp::Logger logger_{rclcpp::get_logger("OmniMpcPursuitController")};
  rclcpp::Clock::SharedPtr clock_;
  mutable std::mutex mutex_;

  nav_msgs::msg::Path global_plan_;
  Command last_cmd_;
  bool have_last_cmd_{false};

  tf2::Duration transform_tolerance_{tf2::durationFromSec(0.2)};
  double control_duration_{0.05};
  double max_robot_pose_search_dist_{3.0};

  int horizon_steps_{12};
  double model_dt_{0.05};
  double reference_speed_{0.8};
  double lookahead_dist_{0.8};

  double vx_min_{-0.2};
  double vx_max_{1.2};
  double vy_min_{-0.45};
  double vy_max_{0.45};
  double wz_min_{-2.2};
  double wz_max_{2.2};

  double ax_max_{2.0};
  double ay_max_{2.0};
  double az_max_{3.0};

  int vx_samples_{7};
  int vy_samples_{7};
  int wz_samples_{9};

  double q_path_x_{2.0};
  double q_path_y_{12.0};
  double q_yaw_{6.0};
  double q_terminal_{3.0};
  double r_control_{0.2};
  double r_delta_control_{2.0};

  int collision_check_step_{2};
};

}  // namespace pb_omni_pid_pursuit_controller

#endif  // PB_OMNI_PID_PURSUIT_CONTROLLER__OMNI_MPC_PURSUIT_CONTROLLER_HPP_
