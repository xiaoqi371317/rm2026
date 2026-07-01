#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <vector>
#include "robo_bridge/my_listener.h"
#include "robo_bridge/robo_bridge.h"

RoboBridge bridge;

namespace robo_bridge
{
class RoboBridgeNode : public rclcpp::Node
{
public:
  explicit RoboBridgeNode(const rclcpp::NodeOptions & options) : Node("robo_bridge", options)
  {
    RCLCPP_INFO(this->get_logger(), "Starting RoboBridgeNode!");
    bridge.init((rclcpp::Node *)this, 500000);
  }
  
  ~RoboBridgeNode() override
  {
    RCLCPP_INFO(this->get_logger(), "RoboBridgeNode destroyed!");
  }
private:

};
} // namespace robo_bridge

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(robo_bridge::RoboBridgeNode)