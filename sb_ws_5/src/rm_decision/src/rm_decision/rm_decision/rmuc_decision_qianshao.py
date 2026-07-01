from dataclasses import dataclass
from enum import Enum, auto
import math
from typing import Dict, Optional

from action_msgs.msg import GoalStatus
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion
from nav2_msgs.action import NavigateToPose
from robo_utils.msg import ReferenceDataMini as RefereeData
from std_msgs.msg import String


SELF_IS_RED = True
AUTO_TUNNEL_ROUTE_ID = 'auto_tunnel'


@dataclass(frozen=True)
class Target:
    x: float
    y: float
    yaw: float = 0.0
    route_id: str = 'normal'


# Replace these map coordinates with your real points.
# route_id controls whether the target should use tunnel follow-yaw mode:
#   normal      -> tunnel manager publishes /if_follow_yaw=false
#   auto_tunnel -> tunnel manager may publish /if_follow_yaw=true near any configured tunnel
RED_TARGETS: Dict[str, Target] = {
    'home': Target(-2.30, -3.65, 0.0, AUTO_TUNNEL_ROUTE_ID),
    'enemy_outpost': Target(11.13, 3.59, 0.0, AUTO_TUNNEL_ROUTE_ID),
    'self_outpost': Target(7.68, -4.55, 0.0, AUTO_TUNNEL_ROUTE_ID),
    'fortress': Target(2.69, 0.25, 0.0, AUTO_TUNNEL_ROUTE_ID),
    'center_cruise_0': Target(4.69, -3.18, 0.0, AUTO_TUNNEL_ROUTE_ID),
    'center_cruise_1': Target(3.15, -2.60, 0.0, AUTO_TUNNEL_ROUTE_ID),
    'self_cruise_0': Target(10.02, -2.62, 0.0, AUTO_TUNNEL_ROUTE_ID),
    'self_cruise_1': Target(8.14, -4.12, 0.0, AUTO_TUNNEL_ROUTE_ID),
}

BLUE_TARGETS: Dict[str, Target] = {
    'home': Target(22.01, 2.50, 0.0, AUTO_TUNNEL_ROUTE_ID),
    'enemy_outpost': Target(9.25, -4.74, 0.0, AUTO_TUNNEL_ROUTE_ID),
    'self_outpost': Target(11.71, 3.13, 0.0, AUTO_TUNNEL_ROUTE_ID),
    'fortress': Target(17.39, -1.35, 0.0, AUTO_TUNNEL_ROUTE_ID),
    'center_cruise_0': Target(16.24, 1.01, 0.0, AUTO_TUNNEL_ROUTE_ID),
    'center_cruise_1': Target(15.35, 1.10, 0.0, AUTO_TUNNEL_ROUTE_ID),
    'self_cruise_0': Target(15.16, 1.61, 0.0, AUTO_TUNNEL_ROUTE_ID),
    'self_cruise_1': Target(16.99, 1.37, 0.0, AUTO_TUNNEL_ROUTE_ID),
}


class RobotState(Enum):
    IDLE = auto()
    GOING_TO_OUTPOST = auto()
    GOING_TO_HOME = auto()
    CENTER_CRUISING = auto()
    HOME_CRUISING = auto()
    GOING_TO_FORTRESS = auto()


class NavigationController:
    def __init__(self, node: Node, action_server_name: str):
        self.node = node
        self.action_client = ActionClient(node, NavigateToPose, action_server_name)
        self.current_goal_handle = None
        self.active_target_name: Optional[str] = None

    def send_target(self, target_name: str, target: Target) -> bool:
        if self.active_target_name == target_name:
            return True

        if not self.action_client.wait_for_server(timeout_sec=1.0):
            self.node.get_logger().error('NavigateToPose action server not available')
            return False

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = self._create_pose_stamped(target)
        self.active_target_name = target_name

        self.node.get_logger().info(
            f'Sending target {target_name}: '
            f'x={target.x:.3f}, y={target.y:.3f}, yaw={target.yaw:.3f}, '
            f'route_id={target.route_id}'
        )
        future = self.action_client.send_goal_async(goal_msg)
        future.add_done_callback(self._goal_response_callback)
        return True

    def cancel_current_goal(self) -> None:
        if self.current_goal_handle is not None:
            self.node.get_logger().info('Canceling current goal...')
            self.current_goal_handle.cancel_goal_async()
            self.current_goal_handle = None
            self.active_target_name = None

    def _create_pose_stamped(self, target: Target) -> PoseStamped:
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.node.get_clock().now().to_msg()
        half_yaw = target.yaw * 0.5
        pose.pose = Pose(
            position=Point(x=target.x, y=target.y, z=0.0),
            orientation=Quaternion(
                x=0.0,
                y=0.0,
                z=math.sin(half_yaw),
                w=math.cos(half_yaw),
            ),
        )
        return pose

    def _goal_response_callback(self, future) -> None:
        self.current_goal_handle = future.result()
        if not self.current_goal_handle.accepted:
            self.node.get_logger().warning('Goal rejected')
            self.current_goal_handle = None
            self.active_target_name = None
            return

        self.node.get_logger().info('Goal accepted')
        self.current_goal_handle.get_result_async().add_done_callback(self._result_callback)

    def _result_callback(self, future) -> None:
        status = None
        try:
            wrapped_result = future.result()
            status = wrapped_result.status
            result = wrapped_result.result
            self.node.get_logger().info(
                f'Navigation completed with status={status}, result: {result}'
            )
        except Exception as exc:
            self.node.get_logger().error(f'Failed to get navigation result: {exc}')
        finally:
            self.current_goal_handle = None
            self.active_target_name = None
            if (
                self.node.reset_route_on_goal_result
                and status == GoalStatus.STATUS_SUCCEEDED
            ):
                self.node.set_tunnel_route_id('normal')
            elif self.node.reset_route_on_goal_result and status is not None:
                self.node.get_logger().warning(
                    'Navigation did not succeed; keeping current tunnel route id'
                )


class StateMachine:
    def __init__(self, node: Node, targets: Dict[str, Target]):
        self.node = node
        self.targets = targets
        self.current_state = RobotState.IDLE
        self.current_target_name: Optional[str] = None
        self.center_cruise_index = 0
        self.self_cruise_index = 0

    def update(self, msg: RefereeData) -> None:
        if msg.game_state != 3:
            return

        if self._handle_low_health(msg.self_health):
            return

        if self._handle_low_bullet(msg.remaining_bullet):
            return

        if self._handle_outpost(msg.remaining_time, msg.enermy_outpost_hp):
            return

        if self._handle_center_cruise(msg.remaining_time):
            return

        self._handle_self_cruise()

    def _handle_low_health(self, self_health: int) -> bool:
        if self.current_state == RobotState.GOING_TO_HOME:
            if self_health >= 300:
                self.current_state = RobotState.IDLE
                self.current_target_name = None
                return False
            return True

        if self_health <= 100:
            self.node.get_logger().warning(f'Emergency return: health={self_health}')
            self._transition_to(RobotState.GOING_TO_HOME, 'home')
            return True

        return False

    def _handle_low_bullet(self, remaining_bullet: int) -> bool:
        if self.current_state == RobotState.GOING_TO_HOME:
            if remaining_bullet >= 50:
                self.current_state = RobotState.IDLE
                self.current_target_name = None
                return False
            return True

        if remaining_bullet <= 10:
            self.node.get_logger().warning(
                f'Emergency return: remaining_bullet={remaining_bullet}'
            )
            self._transition_to(RobotState.GOING_TO_HOME, 'home')
            return True

        return False

    def _handle_outpost(self, remaining_time: int, enemy_outpost_hp: int) -> bool:
        if remaining_time > 150 and enemy_outpost_hp > 0:
            self._transition_to(RobotState.GOING_TO_OUTPOST, 'enemy_outpost')
            return True

        if self.current_state == RobotState.GOING_TO_OUTPOST:
            self.current_state = RobotState.IDLE
            self.current_target_name = None

        return False

    def _handle_center_cruise(self, remaining_time: int) -> bool:
        if remaining_time <= 0:
            return False

        target_names = ['center_cruise_0', 'center_cruise_1']
        target_name = target_names[self.center_cruise_index % len(target_names)]
        self._transition_to(RobotState.CENTER_CRUISING, target_name)
        return True

    def _handle_self_cruise(self) -> bool:
        target_names = ['self_cruise_0', 'self_cruise_1']
        target_name = target_names[self.self_cruise_index % len(target_names)]
        self._transition_to(RobotState.HOME_CRUISING, target_name)
        return True

    def _transition_to(self, new_state: RobotState, target_name: str) -> None:
        if self.current_state == new_state and self.current_target_name == target_name:
            return

        target = self.targets[target_name]
        if self.current_state != new_state:
            self.node.get_logger().info(f'State transition: {self.current_state} -> {new_state}')
        self.current_state = new_state
        self.current_target_name = target_name
        self.node.set_tunnel_route_id(target.route_id)
        self.node.navigation_controller.send_target(target_name, target)


class Nav2DecisionNode(Node):
    def __init__(self):
        super().__init__('nav_decision_node')

        self.action_server_name = self.declare_parameter(
            'action_server_name',
            '/navigate_to_pose',
        ).value
        self.referee_topic = self.declare_parameter(
            'referee_topic',
            '/offboardlink/reference_data_mini',
        ).value
        self.route_topic = self.declare_parameter(
            'route_topic',
            '/tunnel_route_id',
        ).value
        self.reset_route_on_goal_result = bool(
            self.declare_parameter('reset_route_on_goal_result', True).value
        )

        self.targets = RED_TARGETS if SELF_IS_RED else BLUE_TARGETS
        self.latest_referee_msg: Optional[RefereeData] = None
        self.current_tunnel_route_id = 'normal'

        self.tunnel_route_id_pub = self.create_publisher(String, self.route_topic, 10)
        self.navigation_controller = NavigationController(self, self.action_server_name)
        self.state_machine = StateMachine(self, self.targets)

        self.ref_data_sub = self.create_subscription(
            RefereeData,
            self.referee_topic,
            self._ref_data_callback,
            10,
        )
        self.create_timer(0.2, self._publish_tunnel_route_id)
        self.create_timer(0.5, self._update)

        self.get_logger().info(
            f'Qianshao decision started. action_server={self.action_server_name}, '
            f'referee_topic={self.referee_topic}, route_topic={self.route_topic}'
        )

    def set_tunnel_route_id(self, route_id: str) -> None:
        route_id = route_id or 'normal'
        if route_id != self.current_tunnel_route_id:
            self.get_logger().info(
                f'Tunnel route id: {self.current_tunnel_route_id} -> {route_id}'
            )
        self.current_tunnel_route_id = route_id
        self._publish_tunnel_route_id()

    def _publish_tunnel_route_id(self) -> None:
        msg = String()
        msg.data = self.current_tunnel_route_id
        self.tunnel_route_id_pub.publish(msg)

    def _ref_data_callback(self, msg: RefereeData) -> None:
        self.latest_referee_msg = msg

    def _update(self) -> None:
        if self.latest_referee_msg is None:
            return
        self.state_machine.update(self.latest_referee_msg)


def main(args=None):
    rclpy.init(args=args)
    node = Nav2DecisionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down...')
    finally:
        node.navigation_controller.cancel_current_goal()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
