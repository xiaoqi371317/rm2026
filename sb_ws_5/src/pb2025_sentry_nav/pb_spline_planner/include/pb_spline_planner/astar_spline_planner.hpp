#ifndef PB_SPLINE_PLANNER__ASTAR_SPLINE_PLANNER_HPP_
#define PB_SPLINE_PLANNER__ASTAR_SPLINE_PLANNER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_core/global_planner.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "std_msgs/msg/header.hpp"
#include "tf2_ros/buffer.h"

#include "bipedal_wheel_planner/trajectory_generator/perception_tools/GridMap.hpp"

namespace pb_spline_planner
{

class AStarSplinePlanner : public nav2_core::GlobalPlanner
{
public:
  AStarSplinePlanner() = default;
  ~AStarSplinePlanner() override = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void cleanup() override;
  void activate() override;
  void deactivate() override;

  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal) override;

private:
  geometry_msgs::msg::PoseStamped transformToGlobalFrame(
    const geometry_msgs::msg::PoseStamped & pose) const;

  std::shared_ptr<grid_map::GridMap> buildGridMapFromCostmap() const;

  bool isPoseValidInCostmap(const geometry_msgs::msg::PoseStamped & pose) const;

  nav_msgs::msg::Path makePathMessage(
    const std::vector<Eigen::Vector2d> & points, const std_msgs::msg::Header & header,
    const geometry_msgs::msg::PoseStamped & goal) const;

  void updatePathOrientations(nav_msgs::msg::Path & path, const geometry_msgs::msg::PoseStamped & goal)
    const;

  rclcpp_lifecycle::LifecycleNode::WeakPtr parent_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  nav2_costmap_2d::Costmap2D * costmap_{nullptr};

  rclcpp::Clock::SharedPtr clock_;
  rclcpp::Logger logger_{rclcpp::get_logger("pb_spline_planner")};
  std::string name_;
  std::string global_frame_;

  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr astar_path_pub_;
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr spline_path_pub_;

  double hard_safe_distance_{0.12};
  double preferred_clearance_{0.45};
  double astar_clearance_weight_{6.0};
  double spline_clearance_weight_{5000.0};
  double collision_weight_{100000.0};
  bool allow_unknown_{false};
  int lethal_cost_threshold_{252};
  int astar_timeout_ms_{2000};
  double max_velocity_{3.0};
  double max_acceleration_{3.0};
  double trajectory_sample_dt_{0.1};
  bool fallback_to_astar_path_{true};
  bool enable_plan_cache_{true};
  int optimizer_max_iterations_{500};
};

}  // namespace pb_spline_planner

#endif  // PB_SPLINE_PLANNER__ASTAR_SPLINE_PLANNER_HPP_
