from dataclasses import dataclass
from enum import Enum, auto
import math
from typing import Dict, List, Optional

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion
from nav2_msgs.action import NavigateToPose
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from robo_utils.msg import ReferenceDataMini as RefereeData
from std_msgs.msg import String, UInt8


SELF_IS_RED = True

AUTO_TUNNEL_ROUTE_ID = 'auto_tunnel'
NORMAL_ROUTE_ID = 'normal'

POSTURE_NORMAL = 1
POSTURE_DEFENSE = 2
POSTURE_ATTACK = 3


@dataclass(frozen=True)
class Target:
    x: float
    y: float
    yaw: float = 0.0
    route_id: str = AUTO_TUNNEL_ROUTE_ID
    posture: int = POSTURE_DEFENSE


# Replace these map coordinates with your real points.
RED_TARGETS: Dict[str, Target] = {
    'home': Target(-2.30, -3.65),
    'enemy_outpost': Target(11.13, 3.59, posture=POSTURE_ATTACK),
    'enemy_rough_road_0': Target(3.81, -2.72, posture=POSTURE_ATTACK),
    'enemy_rough_road_1': Target(4.40, 1.03, posture=POSTURE_ATTACK),
    'center_patrol_0': Target(4.69, -3.18),
    'center_patrol_1': Target(3.15, -2.60),
}

BLUE_TARGETS: Dict[str, Target] = {
    'home': Target(22.01, 2.50),
    'enemy_outpost': Target(9.25, -4.74, posture=POSTURE_ATTACK),
    'enemy_rough_road_0': Target(15.53, 2.47, posture=POSTURE_ATTACK),
    'enemy_rough_road_1': Target(15.54, -1.52, posture=POSTURE_ATTACK),
    'center_patrol_0': Target(16.24, 1.01),
    'center_patrol_1': Target(15.35, 1.10),
}


class RobotState(Enum):
    WAIT_START = auto()
    ENEMY_ROUGH_ROAD_GUERRILLA = auto()
    ATTACK_OUTPOST = auto()
    CENTER_PATROL = auto()
    RETURN_HOME = auto()


class NavigationController:
    def __init__(self, node: Node, action_server_name: str):
        self.node = node
        self.action_client = ActionClient(node, NavigateToPose, action_server_name)
        self.current_goal_handle = None
        self.active_target_name: Optional[str] = None

    def send_target(self, target_name: str, target: Target) -> bool:
        if self.active_target_name == target_name:
            return True

        if self.current_goal_handle is not None:
            self.cancel_current_goal()

        if not self.action_client.wait_for_server(timeout_sec=1.0):
            self.node.get_logger().error('NavigateToPose action server not available')
            return False

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = self._create_pose_stamped(target)
        self.active_target_name = target_name

        self.node.get_logger().info(
            f'Sending target {target_name}: '
            f'x={target.x:.3f}, y={target.y:.3f}, yaw={target.yaw:.3f}, '
            f'route_id={target.route_id}, posture={target.posture}'
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
            target_name = self.active_target_name
            self.node.get_logger().warning(f'Goal rejected: {target_name}')
            self.current_goal_handle = None
            self.active_target_name = None
            self.node.on_navigation_result(target_name, GoalStatus.STATUS_ABORTED)
            return

        self.node.get_logger().info(f'Goal accepted: {self.active_target_name}')
        self.current_goal_handle.get_result_async().add_done_callback(self._result_callback)

    def _result_callback(self, future) -> None:
        target_name = self.active_target_name
        status = GoalStatus.STATUS_UNKNOWN
        try:
            wrapped_result = future.result()
            status = wrapped_result.status
            self.node.get_logger().info(
                f'Navigation result target={target_name}, status={status}'
            )
        except Exception as exc:
            self.node.get_logger().error(f'Failed to get navigation result: {exc}')
        finally:
            self.current_goal_handle = None
            self.active_target_name = None
            self.node.on_navigation_result(target_name, status)


class DelayedOutpostDecisionNode(Node):
    def __init__(self):
        super().__init__('qianshao_delayed_outpost_decision_node')

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
        self.posture_topic = self.declare_parameter(
            'posture_topic',
            '/sentry/status/posture',
        ).value
        self.self_is_red = bool(self.declare_parameter('self_is_red', SELF_IS_RED).value)

        self.delayed_attack_sec = float(
            self.declare_parameter('delayed_attack_sec', 60.0).value
        )
        self.outpost_attack_dwell_sec = float(
            self.declare_parameter('outpost_attack_dwell_sec', 20.0).value
        )
        self.outpost_attack_timeout_sec = float(
            self.declare_parameter('outpost_attack_timeout_sec', 60.0).value
        )
        self.patrol_hold_sec = float(self.declare_parameter('patrol_hold_sec', 4.0).value)
        self.low_health = int(self.declare_parameter('low_health', 100).value)
        self.recover_health = int(self.declare_parameter('recover_health', 250).value)
        self.low_bullet = int(self.declare_parameter('low_bullet', 10).value)
        self.recover_bullet = int(self.declare_parameter('recover_bullet', 30).value)

        self.targets = RED_TARGETS if self.self_is_red else BLUE_TARGETS
        self.enemy_rough_road_targets = ['enemy_rough_road_0', 'enemy_rough_road_1']
        self.center_patrol_targets = ['center_patrol_0', 'center_patrol_1']

        self.navigation_controller = NavigationController(self, self.action_server_name)
        self.tunnel_route_id_pub = self.create_publisher(String, self.route_topic, 10)
        self.posture_pub = self.create_publisher(UInt8, self.posture_topic, 10)
        self.ref_data_sub = self.create_subscription(
            RefereeData,
            self.referee_topic,
            self._ref_data_callback,
            10,
        )

        self.latest_referee_msg: Optional[RefereeData] = None
        self.current_state = RobotState.WAIT_START
        self.previous_state_before_return: Optional[RobotState] = None
        self.return_due_to_health = False
        self.return_due_to_bullet = False
        self.current_route_id = NORMAL_ROUTE_ID
        self.current_posture = POSTURE_DEFENSE
        self.match_start_time: Optional[float] = None
        self.enemy_rough_road_index = 0
        self.center_index = 0
        self.patrol_hold_until: Optional[float] = None
        self.outpost_phase_start_time: Optional[float] = None
        self.outpost_arrival_time: Optional[float] = None

        self.create_timer(0.2, self._publish_mode_topics)
        self.create_timer(0.2, self._update)

        self.get_logger().info(
            f'Delayed-outpost qianshao decision started. action_server={self.action_server_name}, '
            f'referee_topic={self.referee_topic}, route_topic={self.route_topic}, '
            f'posture_topic={self.posture_topic}'
        )

    def on_navigation_result(self, target_name: Optional[str], status: int) -> None:
        if status != GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().warning(f'Navigation failed target={target_name}, status={status}')
            return

        now = self._now_sec()
        if self.current_state == RobotState.ATTACK_OUTPOST and target_name == 'enemy_outpost':
            self.outpost_arrival_time = now
            self.get_logger().info('Arrived at enemy_outpost; starting attack dwell')
            return

        if (
            self.current_state == RobotState.ENEMY_ROUGH_ROAD_GUERRILLA
            and target_name in self.enemy_rough_road_targets
        ):
            self.patrol_hold_until = now + self.patrol_hold_sec
            return

        if self.current_state == RobotState.CENTER_PATROL and target_name in self.center_patrol_targets:
            self.patrol_hold_until = now + self.patrol_hold_sec
            return

        if self.current_state == RobotState.RETURN_HOME and target_name == 'home':
            self.get_logger().info('Arrived home; waiting for recovery')

    def set_tunnel_route_id(self, route_id: str) -> None:
        route_id = route_id or NORMAL_ROUTE_ID
        if route_id != self.current_route_id:
            self.get_logger().info(f'Tunnel route id: {self.current_route_id} -> {route_id}')
        self.current_route_id = route_id

    def set_posture(self, posture: int) -> None:
        posture = int(posture)
        if posture != self.current_posture:
            self.get_logger().info(f'Sentry posture: {self.current_posture} -> {posture}')
        self.current_posture = posture

    def _ref_data_callback(self, msg: RefereeData) -> None:
        self.latest_referee_msg = msg

    def _update(self) -> None:
        msg = self.latest_referee_msg
        if msg is None:
            return

        if msg.game_state != 3:
            self._handle_wait_start()
            return

        if self.match_start_time is None:
            self.match_start_time = self._now_sec()
            self._transition_to(RobotState.ENEMY_ROUGH_ROAD_GUERRILLA)

        if self._handle_emergency(msg):
            return

        if self.current_state == RobotState.ENEMY_ROUGH_ROAD_GUERRILLA:
            self._handle_enemy_rough_road_guerrilla()
        elif self.current_state == RobotState.ATTACK_OUTPOST:
            self._handle_attack_outpost()
        elif self.current_state == RobotState.CENTER_PATROL:
            self._handle_center_patrol()
        elif self.current_state == RobotState.RETURN_HOME:
            self._handle_return_home(msg)
        else:
            self._transition_to(RobotState.ENEMY_ROUGH_ROAD_GUERRILLA)

    def _handle_wait_start(self) -> None:
        self.set_tunnel_route_id(NORMAL_ROUTE_ID)
        self.set_posture(POSTURE_DEFENSE)

    def _handle_emergency(self, msg: RefereeData) -> bool:
        if self.current_state == RobotState.RETURN_HOME:
            self._handle_return_home(msg)
            return True

        need_health_return = msg.self_health <= self.low_health
        need_bullet_return = msg.remaining_bullet <= self.low_bullet
        if not need_health_return and not need_bullet_return:
            return False

        self.previous_state_before_return = self.current_state
        self.return_due_to_health = need_health_return
        self.return_due_to_bullet = need_bullet_return
        self.get_logger().warning(
            f'Emergency return: health={msg.self_health}, bullet={msg.remaining_bullet}'
        )
        self._transition_to(RobotState.RETURN_HOME)
        return True

    def _handle_return_home(self, msg: RefereeData) -> None:
        self._send_named_target('home', POSTURE_DEFENSE)

        health_ready = (not self.return_due_to_health) or msg.self_health >= self.recover_health
        bullet_ready = (not self.return_due_to_bullet) or msg.remaining_bullet >= self.recover_bullet
        if not health_ready or not bullet_ready:
            return

        resume_state = self.previous_state_before_return or RobotState.ENEMY_ROUGH_ROAD_GUERRILLA
        self.return_due_to_health = False
        self.return_due_to_bullet = False
        self.previous_state_before_return = None
        self.get_logger().info(f'Recovered; resume state {resume_state}')
        self._transition_to(resume_state)

    def _handle_enemy_rough_road_guerrilla(self) -> None:
        if self.match_start_time is not None:
            elapsed = self._now_sec() - self.match_start_time
            if elapsed >= self.delayed_attack_sec:
                self._transition_to(RobotState.ATTACK_OUTPOST)
                return

        self._run_patrol(
            self.enemy_rough_road_targets,
            'enemy_rough_road_index',
            POSTURE_ATTACK,
        )

    def _handle_attack_outpost(self) -> None:
        now = self._now_sec()
        if self.outpost_phase_start_time is None:
            self.outpost_phase_start_time = now

        if self.outpost_arrival_time is not None:
            if now - self.outpost_arrival_time >= self.outpost_attack_dwell_sec:
                self.get_logger().info('Outpost attack dwell finished')
                self._transition_to(RobotState.CENTER_PATROL)
            return

        if now - self.outpost_phase_start_time >= self.outpost_attack_timeout_sec:
            self.get_logger().warning('Outpost attack timeout; entering center patrol')
            self._transition_to(RobotState.CENTER_PATROL)
            return

        self._send_named_target('enemy_outpost', POSTURE_ATTACK)

    def _handle_center_patrol(self) -> None:
        self._run_patrol(self.center_patrol_targets, 'center_index', POSTURE_DEFENSE)

    def _run_patrol(self, target_names: List[str], index_attr: str, posture: int) -> None:
        now = self._now_sec()
        if self.patrol_hold_until is not None:
            if now < self.patrol_hold_until:
                return
            setattr(self, index_attr, getattr(self, index_attr) + 1)
            self.patrol_hold_until = None

        index = getattr(self, index_attr)
        target_name = target_names[index % len(target_names)]
        self._send_named_target(target_name, posture)

    def _transition_to(self, new_state: RobotState) -> None:
        if self.current_state == new_state:
            return

        self.get_logger().info(f'State transition: {self.current_state} -> {new_state}')
        self.current_state = new_state
        self.patrol_hold_until = None

        if new_state == RobotState.ATTACK_OUTPOST:
            self.outpost_phase_start_time = self._now_sec()
            self.outpost_arrival_time = None
        else:
            self.outpost_phase_start_time = None
            self.outpost_arrival_time = None

        self.navigation_controller.active_target_name = None

    def _send_named_target(self, target_name: str, posture: Optional[int] = None) -> None:
        target = self.targets[target_name]
        self.set_tunnel_route_id(target.route_id)
        self.set_posture(target.posture if posture is None else posture)
        self.navigation_controller.send_target(target_name, target)

    def _publish_mode_topics(self) -> None:
        route_msg = String()
        route_msg.data = self.current_route_id
        self.tunnel_route_id_pub.publish(route_msg)

        posture_msg = UInt8()
        posture_msg.data = self.current_posture
        self.posture_pub.publish(posture_msg)

    def _now_sec(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9


def main(args=None):
    rclpy.init(args=args)
    node = DelayedOutpostDecisionNode()
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
