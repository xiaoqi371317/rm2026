"""
Rule-aware autonomous sentry decision node for RMUC 2026.

This file is intended to replace rmuc_decision_qianshao_delayed_outpost.py.
It keeps the same external dependencies/topics used by the original node:
  - Referee data subscription: /offboardlink/reference_data_mini
  - Nav2 action: /navigate_to_pose
  - Tunnel route selector: /tunnel_route_id
  - Sentry posture selector: /sentry/status/posture

Main behavior:
  1. Start directly in home patrol.
  2. At 3 minutes remaining, patrol center highland for 40 seconds once, then return home.
  3. Recovery resumes the interrupted home/center phase.
  3. Enemy deep patrol is disabled by default.
  4. Return home for low HP or low bullets.
  5. Uses referee remaining_time for strategic phase timing when available.

Important integration notes:
  - POSTURE_ATTACK=1, POSTURE_DEFENSE=2, POSTURE_MOVE=3 follow your serial protocol.
  - Tunnel traversal is split into approach -> selected gate -> final target segments.
  - The current serial ReferenceDataMini does not include outpost/base HP; outpost
    pressure is time-based instead of HP-based.
  - Target coordinates are inherited from your current package where possible.
    Please verify every coordinate on the real match map before competition.
  - This node only publishes posture and route targets. It does not send referee-system
    economic commands, because your current package does not expose those command topics.
  - Attack posture is gated by recent bullet consumption during patrol/combat states.
"""

from __future__ import annotations

from collections import defaultdict, deque
from dataclasses import dataclass
from enum import Enum, auto
import math
from typing import Deque, Dict, List, Optional, Tuple

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion
from nav2_msgs.action import NavigateToPose
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import String, UInt8

try:
    # Keep the same import used by your current code.
    from robo_utils.msg import ReferenceDataMini as RefereeData
except ImportError:  # pragma: no cover - fallback for local interface builds.
    from rm_decision_interfaces.msg import RefereeData  # type: ignore


SELF_IS_RED = True

NORMAL_ROUTE_ID = 'normal'
# Must match route_ids configured by tunnel_mode_manager.yaml.
SELF_LEFT_GATE_ROUTE_ID = 'self_left_upper_gate'
SELF_RIGHT_GATE_ROUTE_ID = 'self_right_upper_gate'
ENEMY_LEFT_GATE_ROUTE_ID = 'enemy_left_upper_gate'
ENEMY_RIGHT_GATE_ROUTE_ID = 'enemy_right_upper_gate'

# Keep the numeric mapping used by your current sentry controller.
POSTURE_ATTACK = 1
POSTURE_DEFENSE = 2
POSTURE_MOVE = 3

MATCH_TOTAL_SEC = 420.0
AUTO_SENTRY_MAX_HP = 400
SENTRY_HEAT_LIMIT = 260


@dataclass(frozen=True)
class Target:
    x: float
    y: float
    yaw: float = 0.0
    route_id: str = NORMAL_ROUTE_ID
    default_posture: int = POSTURE_MOVE
    zone: str = 'home'
    allow_tunnel_sequence: bool = True


@dataclass(frozen=True)
class TransitionConfig:
    name: str
    source_zone: str
    target_zone: str
    side: str
    route_id: str
    approach_step: str
    exit_step: str


VALID_ZONES = ('home', 'center', 'enemy')
VALID_TUNNEL_SIDES = ('left', 'right')
TRANSITION_DEFINITIONS: Tuple[Tuple[str, str, str], ...] = (
    ('home_to_center', 'home', 'center'),
    ('center_to_home', 'center', 'home'),
    ('center_to_enemy', 'center', 'enemy'),
    ('enemy_to_center', 'enemy', 'center'),
)
DEFAULT_TARGET_NAMES = [
    'home',
    'home_patrol_0',
    'home_patrol_1',
    'center_patrol_0',
    'center_patrol_1',
    'enemy_patrol_0',
    'enemy_patrol_1',
]


# Coordinates are inherited from your submitted decision files where possible.
# Verify them on the real field / map. A wrong target is more dangerous than a simple policy.
RED_TARGETS: Dict[str, Target] = {
    'home': Target(0.0, 0.0, route_id=NORMAL_ROUTE_ID, default_posture=POSTURE_DEFENSE, zone='home'),
    'home_patrol_0': Target(7.68, -4.55, route_id=NORMAL_ROUTE_ID, default_posture=POSTURE_ATTACK, zone='home'),
    'home_patrol_1': Target(0.0, 0.0, route_id=NORMAL_ROUTE_ID, default_posture=POSTURE_ATTACK, zone='home'),
    'center_patrol_0': Target(4.69, -3.18, route_id=NORMAL_ROUTE_ID, default_posture=POSTURE_ATTACK, zone='center'),
    'center_patrol_1': Target(3.15, -2.60, route_id=NORMAL_ROUTE_ID, default_posture=POSTURE_ATTACK, zone='center'),
    'enemy_patrol_0': Target(5.78, -7.63, route_id=NORMAL_ROUTE_ID, default_posture=POSTURE_ATTACK, zone='enemy'),
    'enemy_patrol_1': Target(5.78, -7.63, route_id=NORMAL_ROUTE_ID, default_posture=POSTURE_ATTACK, zone='enemy'),
}

BLUE_TARGETS: Dict[str, Target] = {
    'home': Target(0.0, 0.0, route_id=NORMAL_ROUTE_ID, default_posture=POSTURE_DEFENSE, zone='home'),
    'home_patrol_0': Target(11.71, 3.13, route_id=NORMAL_ROUTE_ID, default_posture=POSTURE_ATTACK, zone='home'),
    'home_patrol_1': Target(0.0, 0.0, route_id=NORMAL_ROUTE_ID, default_posture=POSTURE_ATTACK, zone='home'),
    'center_patrol_0': Target(16.24, 1.01, route_id=NORMAL_ROUTE_ID, default_posture=POSTURE_ATTACK, zone='center'),
    'center_patrol_1': Target(15.35, 1.10, route_id=NORMAL_ROUTE_ID, default_posture=POSTURE_ATTACK, zone='center'),
    'enemy_patrol_0': Target(5.78, -7.63, route_id=NORMAL_ROUTE_ID, default_posture=POSTURE_ATTACK, zone='enemy'),
    'enemy_patrol_1': Target(5.78, -7.63, route_id=NORMAL_ROUTE_ID, default_posture=POSTURE_ATTACK, zone='enemy'),
}


class RobotState(Enum):
    WAIT_START = auto()
    WAIT_RESPAWN = auto()
    SUPPLY_RECOVERY = auto()
    OPENING_GUERRILLA = auto()
    ATTACK_OUTPOST = auto()
    CENTER_PATROL = auto()
    HOME_PATROL = auto()


@dataclass
class ReturnContext:
    previous_state: Optional[RobotState] = None
    due_to_health: bool = False
    due_to_bullet: bool = False
    due_to_energy: bool = False
    due_to_respawn: bool = False
    patrol_phase_elapsed: float = 0.0
    home_patrol_index: int = 0
    center_index: int = 0
    started_at: float = 0.0

    def clear(self) -> None:
        self.previous_state = None
        self.due_to_health = False
        self.due_to_bullet = False
        self.due_to_energy = False
        self.due_to_respawn = False
        self.patrol_phase_elapsed = 0.0
        self.home_patrol_index = 0
        self.center_index = 0
        self.started_at = 0.0


class NavigationController:
    def __init__(self, node: Node, action_server_name: str):
        self.node = node
        self.action_client = ActionClient(node, NavigateToPose, action_server_name)
        self.current_goal_handle = None
        self.active_target_name: Optional[str] = None
        self.last_goal_sent_time = 0.0

    def send_target(self, target_name: str, target: Target, force: bool = False) -> bool:
        if self.active_target_name == target_name and not force:
            return True

        if self.current_goal_handle is not None and self.active_target_name != target_name:
            self.cancel_current_goal()

        if not self.action_client.wait_for_server(timeout_sec=0.8):
            self.node.get_logger().error('NavigateToPose action server not available')
            return False

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = self._create_pose_stamped(target)
        self.active_target_name = target_name
        self.last_goal_sent_time = self.node.get_clock().now().nanoseconds * 1e-9

        self.node.get_logger().info(
            f'Sending target {target_name}: '
            f'x={target.x:.3f}, y={target.y:.3f}, yaw={target.yaw:.3f}, '
            f'route_id={target.route_id}, posture_hint={target.default_posture}'
        )
        future = self.action_client.send_goal_async(goal_msg)
        future.add_done_callback(self._goal_response_callback)
        return True

    def cancel_current_goal(self) -> None:
        if self.current_goal_handle is not None:
            self.node.get_logger().info(f'Canceling current goal: {self.active_target_name}')
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
        target_name = self.active_target_name
        try:
            self.current_goal_handle = future.result()
        except Exception as exc:
            self.node.get_logger().error(f'Goal response failed target={target_name}: {exc}')
            self.current_goal_handle = None
            self.active_target_name = None
            self.node.on_navigation_result(target_name, GoalStatus.STATUS_ABORTED)
            return

        if not self.current_goal_handle.accepted:
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


class SmartQianshaoDecisionNode(Node):
    def __init__(self):
        super().__init__('smart_qianshao_decision_node')

        self.action_server_name = self.declare_parameter('action_server_name', '/navigate_to_pose').value
        self.referee_topic = self.declare_parameter('referee_topic', '/offboardlink/reference_data_mini').value
        self.route_topic = self.declare_parameter('route_topic', '/tunnel_route_id').value
        self.posture_topic = self.declare_parameter('posture_topic', '/sentry/status/posture').value
        self.debug_topic = self.declare_parameter('debug_topic', '/sentry/decision_state').value
        self.self_is_red = bool(self.declare_parameter('self_is_red', SELF_IS_RED).value)
        self.playing_game_state = int(self.declare_parameter('playing_game_state', 1).value)
        playing_state_defaults = [self.playing_game_state]
        if self.playing_game_state == 1:
            playing_state_defaults.append(2)
        self.playing_game_states = tuple(
            self._declare_int_list_parameter('playing_game_states', playing_state_defaults)
        )

        # High-level tactical parameters.
        self.delayed_attack_sec = float(self.declare_parameter('delayed_attack_sec', 80.0).value)
        self.outpost_attack_dwell_sec = float(self.declare_parameter('outpost_attack_dwell_sec', 60.0).value)
        self.outpost_attack_timeout_sec = float(self.declare_parameter('outpost_attack_timeout_sec', 150.0).value)
        self.enable_enemy_patrol_attack = bool(
            self.declare_parameter('enable_enemy_patrol_attack', False).value
        )
        self.center_patrol_phase_sec = max(
            0.0,
            float(self.declare_parameter('center_patrol_phase_sec', 120.0).value),
        )
        self.scheduled_highland_trigger_remaining_sec = max(
            0.0,
            float(self.declare_parameter('scheduled_highland_trigger_remaining_sec', 180.0).value),
        )
        self.scheduled_highland_patrol_sec = max(
            0.0,
            float(self.declare_parameter('scheduled_highland_patrol_sec', 40.0).value),
        )
        self.patrol_hold_sec = float(self.declare_parameter('patrol_hold_sec', 8.0).value)
        self.combat_fire_hold_sec = max(
            0.0,
            float(self.declare_parameter('combat_fire_hold_sec', 3.0).value),
        )

        # Safety thresholds for an automatic sentry: max HP=400, heat limit=260.
        self.low_health = int(self.declare_parameter('low_health', 150).value)
        self.critical_health = int(self.declare_parameter('critical_health', 90).value)
        self.recover_health = int(self.declare_parameter('recover_health', 320).value)
        self.rapid_retreat_health = int(self.declare_parameter('rapid_retreat_health', 280).value)
        self.low_bullet = int(self.declare_parameter('low_bullet', 55).value)
        self.recover_bullet = int(self.declare_parameter('recover_bullet', 130).value)
        self.low_energy_percent = int(self.declare_parameter('low_energy_percent', 15).value)
        self.recover_energy_percent = int(self.declare_parameter('recover_energy_percent', 30).value)
        self.heat_high = int(self.declare_parameter('heat_high', 210).value)
        self.heat_critical = int(self.declare_parameter('heat_critical', 245).value)

        # Damage timing is used only for posture decisions, not for automatic retreat.
        self.under_attack_hold_sec = float(self.declare_parameter('under_attack_hold_sec', 4.0).value)

        # Rule constraints and housekeeping.
        self.posture_switch_cooldown_sec = float(self.declare_parameter('posture_switch_cooldown_sec', 5.0).value)
        self.nav_failure_backoff_base_sec = float(
            self.declare_parameter('nav_failure_backoff_base_sec', 3.0).value
        )
        self.nav_failure_backoff_max_sec = float(
            self.declare_parameter('nav_failure_backoff_max_sec', 15.0).value
        )
        self.max_nav_failures_before_fallback = int(
            self.declare_parameter('max_nav_failures_before_fallback', 3).value
        )

        self.default_targets = RED_TARGETS if self.self_is_red else BLUE_TARGETS
        self.targets = self._load_targets()
        self.transition_step_target_names: set[str] = set()
        self.transitions = self._load_transition_configs()
        self.home_patrol_targets = self._load_target_name_list(
            'home_patrol_targets',
            ['home_patrol_0', 'home_patrol_1'],
        )
        self.center_patrol_targets = self._load_target_name_list(
            'center_patrol_targets',
            ['center_patrol_0', 'center_patrol_1'],
        )
        self.enemy_patrol_targets = self._load_target_name_list(
            'enemy_patrol_targets',
            ['enemy_patrol_0', 'enemy_patrol_1'],
        )
        self.initial_zone = self._declare_zone_parameter('initial_zone', 'home')

        self.navigation_controller = NavigationController(self, self.action_server_name)
        self.tunnel_route_id_pub = self.create_publisher(String, self.route_topic, 10)
        self.posture_pub = self.create_publisher(UInt8, self.posture_topic, 10)
        self.debug_pub = self.create_publisher(String, self.debug_topic, 10)
        self.ref_data_sub = self.create_subscription(RefereeData, self.referee_topic, self._ref_data_callback, 10)

        self.latest_referee_msg: Optional[RefereeData] = None
        self.current_state = RobotState.WAIT_START
        self.state_entry_time = self._now_sec()
        self.return_context = ReturnContext()
        self.current_zone = self.initial_zone
        self.sequence_target_zone: Optional[str] = None
        self.sequence_active_transition: Optional[str] = None
        self.sequence_active_side: Optional[str] = None
        self.sequence_final_target: Optional[str] = None
        self.sequence_steps: List[str] = []
        self.sequence_index = 0

        self.current_route_id = NORMAL_ROUTE_ID
        self.current_posture = POSTURE_MOVE
        self.posture_since = self._now_sec()
        self.last_posture_switch_time = -999.0
        self.posture_cumulative: Dict[int, float] = defaultdict(float)

        self.match_start_wall_time: Optional[float] = None
        self.last_remaining_time: Optional[int] = None
        self.home_patrol_index = 0
        self.center_index = 0
        self.enemy_patrol_index = 0
        self.patrol_hold_until: Optional[float] = None
        self.patrol_phase_started_at: Optional[float] = None
        self.pending_patrol_resume_state: Optional[RobotState] = None
        self.pending_patrol_resume_elapsed = 0.0
        self.outpost_phase_start_time: Optional[float] = None
        self.outpost_arrival_time: Optional[float] = None
        self.outpost_attack_done = False
        self.scheduled_highland_done = False

        self.health_history: Deque[Tuple[float, int]] = deque(maxlen=80)
        self.last_health: Optional[int] = None
        self.last_bullet: Optional[int] = None
        self.last_damage_time = -999.0
        self.last_fire_time = -999.0
        self.under_attack_until = -999.0
        self.was_dead = False

        self.nav_fail_count: Dict[str, int] = defaultdict(int)
        self.nav_backoff_until: Dict[str, float] = defaultdict(float)

        self.create_timer(0.2, self._publish_periodic_topics)
        self.create_timer(0.2, self._update)

        self.get_logger().info(
            f'Smart qianshao decision started. side={"RED" if self.self_is_red else "BLUE"}, '
            f'action_server={self.action_server_name}, referee_topic={self.referee_topic}, '
            f'route_topic={self.route_topic}, posture_topic={self.posture_topic}, '
            f'initial_zone={self.current_zone}, playing_game_states={self.playing_game_states}'
        )

    def _load_targets(self) -> Dict[str, Target]:
        target_names_value = self.declare_parameter('target_names', DEFAULT_TARGET_NAMES).value
        target_names = [str(name) for name in target_names_value]
        targets: Dict[str, Target] = {}
        for target_name in target_names:
            default_target = self.default_targets.get(target_name, Target(0.0, 0.0))
            pose = self._declare_pose_parameter(
                f'target.{target_name}.pose',
                [default_target.x, default_target.y, default_target.yaw],
            )
            zone = self._declare_zone_parameter(f'target.{target_name}.zone', default_target.zone)
            targets[target_name] = Target(
                pose[0],
                pose[1],
                yaw=pose[2],
                route_id=NORMAL_ROUTE_ID,
                default_posture=default_target.default_posture,
                zone=zone,
                allow_tunnel_sequence=True,
            )
        return targets

    def _load_target_name_list(self, parameter_name: str, default_names: List[str]) -> List[str]:
        value = self.declare_parameter(parameter_name, default_names).value
        target_names = [str(name) for name in value]
        valid_names = [name for name in target_names if name in self.targets]
        missing_names = [name for name in target_names if name not in self.targets]
        if missing_names:
            self.get_logger().warning(
                f'{parameter_name} contains unknown targets: {", ".join(missing_names)}'
            )
        if not valid_names:
            self.get_logger().warning(f'{parameter_name} has no valid targets')
        return valid_names

    def _declare_int_list_parameter(self, name: str, default: List[int]) -> List[int]:
        value = self.declare_parameter(name, default).value
        raw_values = value if isinstance(value, (list, tuple)) else [value]
        result: List[int] = []
        for raw_value in raw_values:
            try:
                state = int(raw_value)
            except (TypeError, ValueError):
                self.get_logger().warning(f'Invalid {name} entry={raw_value}; ignoring')
                continue
            if state not in result:
                result.append(state)
        if not result:
            self.get_logger().warning(f'{name} has no valid states; using {default}')
            return default
        return result

    def _load_transition_configs(self) -> Dict[Tuple[str, str], TransitionConfig]:
        transitions: Dict[Tuple[str, str], TransitionConfig] = {}
        for transition_name, source_zone, target_zone in TRANSITION_DEFINITIONS:
            side = str(
                self.declare_parameter(f'transition.{transition_name}.side', 'left').value
            ).strip().lower()
            if side not in VALID_TUNNEL_SIDES:
                self.get_logger().warning(
                    f'Invalid transition.{transition_name}.side={side}; using left'
                )
                side = 'left'

            selected_steps: Dict[str, Tuple[str, str]] = {}
            for side_name in VALID_TUNNEL_SIDES:
                approach_pose = self._declare_pose_parameter(
                    f'transition.{transition_name}.{side_name}.approach_pose',
                    [0.0, 0.0, 0.0],
                )
                exit_pose = self._declare_pose_parameter(
                    f'transition.{transition_name}.{side_name}.exit_pose',
                    [0.0, 0.0, 0.0],
                )
                approach_step = f'__transition_{transition_name}_{side_name}_approach'
                exit_step = f'__transition_{transition_name}_{side_name}_exit'
                route_id = self._route_id_for_transition(source_zone, target_zone, side_name)

                self.targets[approach_step] = Target(
                    approach_pose[0],
                    approach_pose[1],
                    yaw=approach_pose[2],
                    route_id=NORMAL_ROUTE_ID,
                    default_posture=POSTURE_MOVE,
                    zone=source_zone,
                    allow_tunnel_sequence=False,
                )
                self.targets[exit_step] = Target(
                    exit_pose[0],
                    exit_pose[1],
                    yaw=exit_pose[2],
                    route_id=route_id,
                    default_posture=POSTURE_MOVE,
                    zone=target_zone,
                    allow_tunnel_sequence=False,
                )
                self.transition_step_target_names.update((approach_step, exit_step))
                selected_steps[side_name] = (approach_step, exit_step)

            approach_step, exit_step = selected_steps[side]
            transitions[(source_zone, target_zone)] = TransitionConfig(
                name=transition_name,
                source_zone=source_zone,
                target_zone=target_zone,
                side=side,
                route_id=self._route_id_for_transition(source_zone, target_zone, side),
                approach_step=approach_step,
                exit_step=exit_step,
            )
        return transitions

    def _declare_pose_parameter(self, name: str, default: List[float]) -> Tuple[float, float, float]:
        value = self.declare_parameter(name, default).value
        if len(value) == 2:
            return float(value[0]), float(value[1]), 0.0
        if len(value) == 3:
            return float(value[0]), float(value[1]), float(value[2])
        self.get_logger().warning(f'Parameter {name} must be [x, y] or [x, y, yaw]; using default')
        return float(default[0]), float(default[1]), float(default[2])

    def _declare_zone_parameter(self, name: str, default: str) -> str:
        zone = str(self.declare_parameter(name, default).value).strip().lower()
        if zone not in VALID_ZONES:
            self.get_logger().warning(f'Invalid {name}={zone}; using {default}')
            return default
        return zone

    def _route_id_for_transition(self, source_zone: str, target_zone: str, side: str) -> str:
        if {source_zone, target_zone} == {'home', 'center'}:
            return SELF_LEFT_GATE_ROUTE_ID if side == 'left' else SELF_RIGHT_GATE_ROUTE_ID
        if {source_zone, target_zone} == {'center', 'enemy'}:
            return ENEMY_LEFT_GATE_ROUTE_ID if side == 'left' else ENEMY_RIGHT_GATE_ROUTE_ID
        return NORMAL_ROUTE_ID

    # ----------------------------- ROS callbacks -----------------------------

    def _ref_data_callback(self, msg: RefereeData) -> None:
        self.latest_referee_msg = msg
        self._update_histories(msg)

    def _is_game_playing(self, msg: RefereeData) -> bool:
        return int(msg.game_state) in self.playing_game_states

    def on_navigation_result(self, target_name: Optional[str], status: int) -> None:
        if target_name is None:
            return

        now = self._now_sec()
        self._reset_tunnel_route_after_navigation()
        if status != GoalStatus.STATUS_SUCCEEDED:
            self._clear_tunnel_sequence_if_current_step_failed(target_name)
            self.nav_fail_count[target_name] += 1
            fail_count = self.nav_fail_count[target_name]
            backoff = min(self.nav_failure_backoff_base_sec * fail_count, self.nav_failure_backoff_max_sec)
            self.nav_backoff_until[target_name] = now + backoff
            self.get_logger().warning(
                f'Navigation failed target={target_name}, status={status}, '
                f'fail_count={fail_count}, backoff={backoff:.1f}s'
            )
            self._handle_navigation_failure(target_name, fail_count)
            return

        self.nav_fail_count[target_name] = 0
        self.nav_backoff_until[target_name] = 0.0

        if self._handle_tunnel_sequence_success(target_name):
            return

        self._update_current_zone_from_target(target_name)

        if (
            target_name in self.home_patrol_targets
            or target_name in self.center_patrol_targets
            or target_name in self.enemy_patrol_targets
        ):
            self.patrol_hold_until = now + self.patrol_hold_sec
            return

        if self.current_state == RobotState.SUPPLY_RECOVERY and target_name == 'home':
            self.get_logger().info('Arrived home/supply; waiting for recovery conditions')
            return

    # ----------------------------- Main update loop -----------------------------

    def _update(self) -> None:
        msg = self.latest_referee_msg
        if msg is None:
            return

        if not self._is_game_playing(msg):
            self._handle_wait_start()
            return

        if self.match_start_wall_time is None:
            self.match_start_wall_time = self._now_sec()
            self._transition_to(RobotState.HOME_PATROL, 'match_start_home_patrol')

        if self._handle_dead_or_respawn(msg):
            self._apply_posture_policy(msg)
            return

        if self._handle_safety_priority(msg):
            self._apply_posture_policy(msg)
            return

        if self._handle_attack_outpost(msg):
            self._apply_posture_policy(msg)
            return

        if self._handle_scheduled_highland_patrol(msg):
            self._apply_posture_policy(msg)
            return

        if self._handle_center_home_cycle(msg):
            self._apply_posture_policy(msg)
            return

        # Fallback should be reached rarely.
        self._transition_to(RobotState.HOME_PATROL, 'fallback_home_patrol')
        self._handle_home_patrol()
        self._apply_posture_policy(msg)

    # ----------------------------- History and inference -----------------------------

    def _update_histories(self, msg: RefereeData) -> None:
        now = self._now_sec()
        health = int(msg.self_health)
        bullet = int(msg.remaining_bullet)

        if health > 0:
            self.health_history.append((now, health))

        if self.last_health is not None:
            if 0 < health < self.last_health:
                self.last_damage_time = now
                self.under_attack_until = now + self.under_attack_hold_sec
            elif self.last_health == 0 and health > 0:
                self.was_dead = True

        if self.last_bullet is not None:
            if bullet < self.last_bullet:
                self.last_fire_time = now

        self.last_health = health
        self.last_bullet = bullet

    def _is_under_attack(self) -> bool:
        return self._now_sec() < self.under_attack_until

    def _is_out_of_combat(self) -> bool:
        now = self._now_sec()
        return now - self.last_fire_time >= 6.0 and now - self.last_damage_time >= 6.0

    def _has_recent_fire(self) -> bool:
        return self._now_sec() - self.last_fire_time < self.combat_fire_hold_sec

    def _last_fire_age_sec(self) -> float:
        if self.last_fire_time < 0.0:
            return float('inf')
        return max(0.0, self._now_sec() - self.last_fire_time)

    def _elapsed_sec(self, msg: RefereeData) -> float:
        return self._match_elapsed_sec(msg)

    def _match_elapsed_sec(self, msg: Optional[RefereeData] = None) -> float:
        msg = msg or self.latest_referee_msg
        if msg is not None:
            remaining = int(msg.remaining_time)
            if 0 < remaining <= int(MATCH_TOTAL_SEC):
                return max(0.0, MATCH_TOTAL_SEC - float(remaining))

        if self.match_start_wall_time is None:
            return 0.0
        return max(0.0, self._now_sec() - self.match_start_wall_time)

    def _has_valid_referee_time(self, msg: Optional[RefereeData] = None) -> bool:
        msg = msg or self.latest_referee_msg
        if msg is None:
            return False
        remaining = int(msg.remaining_time)
        return 0 < remaining <= int(MATCH_TOTAL_SEC)

    def _remaining_sec(self, msg: RefereeData) -> float:
        remaining = int(msg.remaining_time)
        if self._has_valid_referee_time(msg):
            return float(remaining)
        return max(0.0, MATCH_TOTAL_SEC - self._elapsed_sec(msg))

    def _at_supply(self, msg: RefereeData) -> bool:
        return bool(int(msg.at_supply_area_unoverlap) or int(msg.at_supply_area_overlap))

    def _at_castle(self, msg: RefereeData) -> bool:
        return bool(int(msg.at_castle_area))

    def _health_need_return(self, msg: RefereeData) -> bool:
        return 0 < int(msg.self_health) <= self.low_health

    def _bullet_need_return(self, msg: RefereeData) -> bool:
        return int(msg.remaining_bullet) <= self.low_bullet

    def _energy_need_return(self, msg: RefereeData) -> bool:
        # If energy_percent is not populated in your system, it often stays at 0.
        # Treat only 1..low_energy as a valid low-energy signal.
        energy = int(msg.energy_percent)
        return 0 < energy <= self.low_energy_percent

    # ----------------------------- Priority handlers -----------------------------

    def _handle_wait_start(self) -> None:
        self.match_start_wall_time = None
        self.last_remaining_time = None
        self.return_context.clear()
        self.outpost_phase_start_time = None
        self.outpost_arrival_time = None
        self.outpost_attack_done = False
        self.scheduled_highland_done = False
        self.current_zone = self.initial_zone
        self.patrol_phase_started_at = None
        self.pending_patrol_resume_state = None
        self.pending_patrol_resume_elapsed = 0.0
        self.home_patrol_index = 0
        self.center_index = 0
        self._clear_tunnel_sequence()
        self.set_tunnel_route_id(NORMAL_ROUTE_ID)
        self._set_posture(POSTURE_MOVE, reason='wait_start_default_move', force=True)
        if self.current_state != RobotState.WAIT_START:
            self._transition_to(RobotState.WAIT_START, 'game_not_playing')

    def _handle_dead_or_respawn(self, msg: RefereeData) -> bool:
        health = int(msg.self_health)
        if health <= 0:
            if self.current_state != RobotState.WAIT_RESPAWN:
                self.navigation_controller.cancel_current_goal()
                self._clear_tunnel_sequence()
                self._transition_to(RobotState.WAIT_RESPAWN, 'self_dead')
            return True

        if self.current_state == RobotState.WAIT_RESPAWN or self.was_dead:
            self.was_dead = False
            self.current_zone = 'home'
            self._clear_tunnel_sequence()
            self.return_context.previous_state = RobotState.HOME_PATROL
            self.return_context.due_to_respawn = True
            self.return_context.due_to_health = True
            self.return_context.started_at = self._now_sec()
            self._transition_to(RobotState.SUPPLY_RECOVERY, 'respawn_clear_weak_and_recover')
            self._handle_supply_recovery(msg)
            return True

        return False

    def _handle_safety_priority(self, msg: RefereeData) -> bool:
        if self.current_state == RobotState.SUPPLY_RECOVERY:
            self._handle_supply_recovery(msg)
            return True

        need_health = self._health_need_return(msg)
        need_bullet = self._bullet_need_return(msg)
        need_energy = False
        heat_critical = int(msg.shoot_heat) >= self.heat_critical

        if need_health or need_bullet or need_energy:
            self._start_supply_recovery(
                previous_state=self.current_state,
                due_to_health=need_health,
                due_to_bullet=need_bullet,
                due_to_energy=need_energy,
                reason=f'supply_needed hp={int(msg.self_health)} bullet={int(msg.remaining_bullet)}',
            )
            self._handle_supply_recovery(msg)
            return True

        # If heat is close to permanent-lock range, do not initiate new deep attack.
        # Posture policy will prefer attack posture to cool faster.
        if heat_critical and self.current_state in (RobotState.OPENING_GUERRILLA, RobotState.ATTACK_OUTPOST):
            self.get_logger().warning(f'Heat critical={int(msg.shoot_heat)}; falling back to home patrol')
            self._transition_to(RobotState.HOME_PATROL, 'critical_heat_home_patrol')
            self._handle_home_patrol()
            return True

        return False

    def _start_supply_recovery(
        self,
        previous_state: RobotState,
        due_to_health: bool,
        due_to_bullet: bool,
        due_to_energy: bool,
        reason: str,
    ) -> None:
        resume_state = previous_state
        if resume_state not in (RobotState.CENTER_PATROL, RobotState.HOME_PATROL):
            resume_state = RobotState.HOME_PATROL

        self.return_context.previous_state = resume_state
        self.return_context.due_to_health = due_to_health
        self.return_context.due_to_bullet = due_to_bullet
        self.return_context.due_to_energy = due_to_energy
        self.return_context.patrol_phase_elapsed = self._current_patrol_phase_elapsed(resume_state)
        self.return_context.home_patrol_index = self.home_patrol_index
        self.return_context.center_index = self.center_index
        self.return_context.started_at = self._now_sec()
        self._transition_to(RobotState.SUPPLY_RECOVERY, reason)

    def _handle_supply_recovery(self, msg: RefereeData) -> None:
        self._send_named_target('home')

        health = int(msg.self_health)
        bullet = int(msg.remaining_bullet)
        energy = int(msg.energy_percent)
        remaining = self._remaining_sec(msg)
        at_supply = self._at_supply(msg)
        stayed = self._now_sec() - self.return_context.started_at

        health_ready = (not self.return_context.due_to_health) or health >= self.recover_health
        bullet_ready = (not self.return_context.due_to_bullet) or bullet >= self.recover_bullet
        if self.return_context.due_to_energy:
            energy_ready = energy == 0 or energy >= self.recover_energy_percent
        else:
            energy_ready = True

        # Late game: a half-recovered sentry still has value; do not over-stay in supply.
        late_game_override = remaining <= 45.0 and health >= max(self.low_health + 45, 190)
        max_stay_override = at_supply and stayed >= 18.0 and health >= max(self.low_health + 70, 220)
        if (
            health_ready and bullet_ready and energy_ready
        ) or late_game_override or max_stay_override:
            resume_state = self.return_context.previous_state or RobotState.HOME_PATROL
            if resume_state not in (RobotState.CENTER_PATROL, RobotState.HOME_PATROL):
                resume_state = RobotState.HOME_PATROL
            resume_elapsed = self.return_context.patrol_phase_elapsed
            resume_home_index = self.return_context.home_patrol_index
            resume_center_index = self.return_context.center_index
            self.get_logger().info(
                f'Recovery complete: hp={health}, bullet={bullet}, energy={energy}; '
                f'resume={resume_state.name}, phase_elapsed={resume_elapsed:.1f}s'
            )
            self.return_context.clear()
            self._transition_to(resume_state, 'recovery_complete_resume_previous_phase')
            self.home_patrol_index = resume_home_index
            self.center_index = resume_center_index
            if resume_state in (RobotState.CENTER_PATROL, RobotState.HOME_PATROL):
                self.pending_patrol_resume_state = resume_state
                self.pending_patrol_resume_elapsed = resume_elapsed

    def _handle_attack_outpost(self, msg: RefereeData) -> bool:
        if not self.enable_enemy_patrol_attack:
            return False

        if self.outpost_attack_done:
            return False

        if self.current_state == RobotState.HOME_PATROL:
            return False

        elapsed = self._elapsed_sec(msg)
        remaining = self._remaining_sec(msg)
        health_ok = int(msg.self_health) > self.low_health
        bullet_ok = int(msg.remaining_bullet) > self.low_bullet
        heat_ok = int(msg.shoot_heat) < self.heat_critical

        if self.current_state == RobotState.ATTACK_OUTPOST:
            now = self._now_sec()
            if self.outpost_phase_start_time is None:
                self.outpost_phase_start_time = now

            if not health_ok or not bullet_ok:
                self._transition_to(RobotState.HOME_PATROL, 'attack_resources_low_home_patrol')
                self._handle_home_patrol()
                return True

            attack_elapsed = now - self.outpost_phase_start_time
            if (
                attack_elapsed >= self.outpost_attack_dwell_sec
                or attack_elapsed >= self.outpost_attack_timeout_sec
            ):
                self.get_logger().info('Enemy patrol attack phase finished; entering home patrol')
                self.outpost_attack_done = True
                self._transition_to(RobotState.HOME_PATROL, 'enemy_patrol_phase_finished')
                self._handle_home_patrol()
                return True

            self._handle_enemy_patrol()
            return True

        if elapsed >= self.delayed_attack_sec and remaining > 95.0 and health_ok and bullet_ok and heat_ok:
            self._transition_to(RobotState.ATTACK_OUTPOST, f'delayed_attack_elapsed={elapsed:.1f}')
            self._handle_enemy_patrol()
            return True

        return False

    def _handle_scheduled_highland_patrol(self, msg: RefereeData) -> bool:
        if self.scheduled_highland_done:
            return False

        if self.current_state == RobotState.CENTER_PATROL:
            elapsed = self._patrol_phase_elapsed('center')
            if elapsed >= self.scheduled_highland_patrol_sec:
                self.scheduled_highland_done = True
                self._transition_to(RobotState.HOME_PATROL, 'scheduled_highland_finished_go_home')
                self._handle_home_patrol()
                return True

            self._handle_center_patrol()
            return True

        remaining = self._remaining_sec(msg)
        if remaining <= self.scheduled_highland_trigger_remaining_sec:
            self._transition_to(
                RobotState.CENTER_PATROL,
                f'scheduled_highland_start remaining={remaining:.1f}',
            )
            self._handle_center_patrol()
            return True

        return False

    def _handle_center_home_cycle(self, msg: RefereeData) -> bool:
        if self.current_state != RobotState.HOME_PATROL:
            self._transition_to(RobotState.HOME_PATROL, 'default_home_patrol')
        self._patrol_phase_elapsed('home')
        self._handle_home_patrol()
        return True

    def _patrol_phase_elapsed(self, zone: str) -> float:
        if self.current_zone != zone:
            self.patrol_phase_started_at = None
            return 0.0

        self._apply_pending_patrol_resume(zone)

        now = self._match_elapsed_sec()
        if self.patrol_phase_started_at is None:
            self.patrol_phase_started_at = now
            return 0.0

        return max(0.0, now - self.patrol_phase_started_at)

    def _current_patrol_phase_elapsed(self, state: RobotState) -> float:
        zone = 'center' if state == RobotState.CENTER_PATROL else 'home'
        if state not in (RobotState.CENTER_PATROL, RobotState.HOME_PATROL):
            return 0.0
        if self.current_zone != zone or self.patrol_phase_started_at is None:
            return 0.0
        return max(0.0, self._match_elapsed_sec() - self.patrol_phase_started_at)

    def _apply_pending_patrol_resume(self, zone: str) -> None:
        if self.pending_patrol_resume_state is None:
            return

        expected_zone = 'center' if self.pending_patrol_resume_state == RobotState.CENTER_PATROL else 'home'
        if self.current_state != self.pending_patrol_resume_state or zone != expected_zone:
            return
        if self.current_zone != expected_zone:
            return

        elapsed = max(0.0, self.pending_patrol_resume_elapsed)
        self.patrol_phase_started_at = self._match_elapsed_sec() - elapsed
        self.pending_patrol_resume_state = None
        self.pending_patrol_resume_elapsed = 0.0
        self.get_logger().info(
            f'Resumed {expected_zone} patrol phase with elapsed={elapsed:.1f}s'
        )

    def _handle_home_patrol(self) -> None:
        self._run_patrol(self.home_patrol_targets, 'home_patrol_index')

    def _handle_center_patrol(self) -> None:
        self._run_patrol(self.center_patrol_targets, 'center_index')

    def _handle_enemy_patrol(self) -> None:
        self._run_patrol(self.enemy_patrol_targets, 'enemy_patrol_index')

    # ----------------------------- Patrol and transitions -----------------------------

    def _run_patrol(self, target_names: List[str], index_attr: str) -> None:
        if not target_names:
            self.get_logger().warning(f'No patrol targets configured for {index_attr}')
            return

        now = self._now_sec()
        if self.patrol_hold_until is not None:
            if now < self.patrol_hold_until:
                return
            setattr(self, index_attr, getattr(self, index_attr) + 1)
            self.patrol_hold_until = None

        index = getattr(self, index_attr)
        target_name = target_names[index % len(target_names)]
        self._send_named_target(target_name)

    def _transition_to(self, new_state: RobotState, reason: str) -> None:
        if self.current_state == new_state:
            return

        self.get_logger().info(f'State transition: {self.current_state.name} -> {new_state.name}, reason={reason}')
        self.current_state = new_state
        self.state_entry_time = self._now_sec()
        self.patrol_hold_until = None
        self.patrol_phase_started_at = None
        if self.pending_patrol_resume_state is not None and new_state != self.pending_patrol_resume_state:
            self.pending_patrol_resume_state = None
            self.pending_patrol_resume_elapsed = 0.0

        if new_state == RobotState.CENTER_PATROL:
            self.center_index = 0
        elif new_state == RobotState.HOME_PATROL:
            self.home_patrol_index = 0

        if new_state == RobotState.ATTACK_OUTPOST:
            self.outpost_phase_start_time = self._now_sec()
            self.outpost_arrival_time = None
        else:
            self.outpost_phase_start_time = None
            self.outpost_arrival_time = None

    def _send_named_target(self, target_name: str, force: bool = False) -> bool:
        target = self.targets[target_name]
        if self._should_use_tunnel_sequence(target_name, target):
            return self._start_or_continue_tunnel_sequence(target_name, force=force)

        if self.sequence_final_target is not None and self.sequence_final_target != target_name:
            self._clear_tunnel_sequence()

        return self._send_direct_target(target_name, target, force=force)

    def _send_direct_target(self, target_name: str, target: Target, force: bool = False) -> bool:
        now = self._now_sec()
        backoff_until = self.nav_backoff_until.get(target_name, 0.0)
        if backoff_until > now and not force:
            self.get_logger().warning(
                f'Target {target_name} in nav backoff for {backoff_until - now:.1f}s; not resending yet'
            )
            return False

        self.set_tunnel_route_id(target.route_id)
        return self.navigation_controller.send_target(target_name, target, force=force)

    def _should_use_tunnel_sequence(self, target_name: str, target: Target) -> bool:
        if target_name in self.transition_step_target_names:
            return False
        if not target.allow_tunnel_sequence:
            return False
        if target.zone not in VALID_ZONES:
            return False
        return target.zone != self.current_zone

    def _start_or_continue_tunnel_sequence(self, target_name: str, force: bool = False) -> bool:
        if self.sequence_final_target != target_name:
            self.sequence_final_target = target_name
            self.sequence_target_zone = self.targets[target_name].zone
            self.sequence_steps = self._build_tunnel_sequence(target_name)
            self.sequence_index = 0
            self._update_active_sequence_debug()
            self.get_logger().info(
                f'Tunnel sequence current_zone={self.current_zone}, '
                f'target_zone={self.sequence_target_zone}, final={target_name}, '
                f'steps={" -> ".join(self.sequence_steps)}'
            )

        return self._send_current_tunnel_sequence_step(force=force)

    def _build_tunnel_sequence(self, final_target_name: str) -> List[str]:
        final_target = self.targets[final_target_name]
        zone_path = self._build_zone_path(self.current_zone, final_target.zone)
        if not zone_path:
            return [final_target_name]

        steps: List[str] = []
        for source_zone, target_zone in zone_path:
            transition = self.transitions.get((source_zone, target_zone))
            if transition is None:
                self.get_logger().error(
                    f'Missing transition {source_zone}_to_{target_zone}; '
                    f'sending final target {final_target_name} directly'
                )
                return [final_target_name]
            steps.extend([transition.approach_step, transition.exit_step])
        steps.append(final_target_name)
        return steps

    def _build_zone_path(self, source_zone: str, target_zone: str) -> List[Tuple[str, str]]:
        if source_zone == target_zone:
            return []
        if (source_zone, target_zone) in self.transitions:
            return [(source_zone, target_zone)]
        if source_zone == 'home' and target_zone == 'enemy':
            return [('home', 'center'), ('center', 'enemy')]
        if source_zone == 'enemy' and target_zone == 'home':
            return [('enemy', 'center'), ('center', 'home')]
        self.get_logger().error(f'Unsupported zone transition: {source_zone} -> {target_zone}')
        return []

    def _send_current_tunnel_sequence_step(self, force: bool = False) -> bool:
        if not self.sequence_steps:
            return False
        if self.sequence_index >= len(self.sequence_steps):
            self._clear_tunnel_sequence()
            return False

        step_name = self.sequence_steps[self.sequence_index]
        self._update_active_sequence_debug()
        return self._send_direct_target(step_name, self.targets[step_name], force=force)

    def _handle_tunnel_sequence_success(self, target_name: str) -> bool:
        if not self.sequence_steps or self.sequence_index >= len(self.sequence_steps):
            return False

        expected_step = self.sequence_steps[self.sequence_index]
        if target_name != expected_step:
            return False

        if self.sequence_index < len(self.sequence_steps) - 1:
            self._update_current_zone_from_target(target_name)
            self.sequence_index += 1
            self._send_current_tunnel_sequence_step(force=True)
            return True

        self._update_current_zone_from_target(target_name)
        self._clear_tunnel_sequence()
        return False

    def _clear_tunnel_sequence_if_current_step_failed(self, target_name: str) -> None:
        if not self.sequence_steps or self.sequence_index >= len(self.sequence_steps):
            return
        if target_name != self.sequence_steps[self.sequence_index]:
            return
        self.get_logger().warning(
            f'Tunnel sequence failed at step={target_name}, final={self.sequence_final_target}'
        )
        self._clear_tunnel_sequence()

    def _clear_tunnel_sequence(self) -> None:
        self.sequence_final_target = None
        self.sequence_target_zone = None
        self.sequence_active_transition = None
        self.sequence_active_side = None
        self.sequence_steps = []
        self.sequence_index = 0

    def _update_active_sequence_debug(self) -> None:
        self.sequence_active_transition = None
        self.sequence_active_side = None
        if not self.sequence_steps or self.sequence_index >= len(self.sequence_steps):
            return

        step_name = self.sequence_steps[self.sequence_index]
        for transition in self.transitions.values():
            if step_name in (transition.approach_step, transition.exit_step):
                self.sequence_active_transition = transition.name
                self.sequence_active_side = transition.side
                return

    def _update_current_zone_from_target(self, target_name: str) -> None:
        target = self.targets.get(target_name)
        if target is None or target.zone not in VALID_ZONES:
            return
        if target.zone != self.current_zone:
            self.get_logger().info(f'Current zone: {self.current_zone} -> {target.zone}, target={target_name}')
            self.current_zone = target.zone

    def _handle_navigation_failure(self, target_name: str, fail_count: int) -> None:
        if fail_count < self.max_nav_failures_before_fallback:
            return

        if target_name in self.transition_step_target_names:
            self.get_logger().warning(
                f'Transition target {target_name} failed repeatedly; fallback to home patrol'
            )
            self._transition_to(RobotState.HOME_PATROL, 'tunnel_sequence_nav_failure')
            return

        if target_name in self.enemy_patrol_targets:
            self.get_logger().warning(f'High-risk target {target_name} failed repeatedly; fallback to home patrol')
            self._transition_to(RobotState.HOME_PATROL, 'nav_failure_fallback')
            return

        if target_name == 'home':
            self.get_logger().warning('home target failed repeatedly; holding current position in defense posture')
            return

    # ----------------------------- Posture policy -----------------------------

    def _apply_posture_policy(self, msg: RefereeData) -> None:
        desired, reason = self._desired_posture(msg)
        self._set_posture(desired, reason=reason)

    def _desired_posture(self, msg: RefereeData) -> Tuple[int, str]:
        health = int(msg.self_health)
        under_attack = self._is_under_attack()

        if self.current_state == RobotState.WAIT_START:
            return POSTURE_MOVE, 'opening_default_move'

        if self.current_state == RobotState.WAIT_RESPAWN:
            return POSTURE_DEFENSE, 'respawn_wait_defense'

        # Critical survivability beats cooling and mobility.
        if health <= self.critical_health:
            return POSTURE_DEFENSE, 'critical_health_defense'

        if self.current_state == RobotState.SUPPLY_RECOVERY:
            if under_attack or health <= self.low_health:
                return POSTURE_DEFENSE, 'recovery_under_attack_or_low_hp'
            return POSTURE_DEFENSE, 'supply_heal_defense'

        if self.current_state == RobotState.ATTACK_OUTPOST:
            if under_attack and health <= self.rapid_retreat_health:
                return POSTURE_DEFENSE, 'attack_under_damage_defense'
            return self._desired_combat_posture('outpost')

        if self.current_state in (RobotState.OPENING_GUERRILLA, RobotState.CENTER_PATROL):
            if self.current_zone != 'center':
                return POSTURE_MOVE, 'go_center_move'
            return self._desired_combat_posture('center_highland')

        if self.current_state == RobotState.HOME_PATROL:
            return self._desired_combat_posture('home_patrol')

        return POSTURE_DEFENSE, 'default_defense'

    def _desired_combat_posture(self, context: str) -> Tuple[int, str]:
        if self._has_recent_fire():
            return POSTURE_ATTACK, f'combat_{context}_fire_recent_attack'
        return POSTURE_DEFENSE, f'combat_{context}_idle_defense'

    def _set_posture(self, posture: int, reason: str, force: bool = False) -> None:
        posture = int(posture)
        now = self._now_sec()

        if posture == self.current_posture:
            return

        if not force and now - self.last_posture_switch_time < self.posture_switch_cooldown_sec:
            return

        used = self._posture_used_sec(posture)
        self.posture_cumulative[self.current_posture] += max(0.0, now - self.posture_since)
        old = self.current_posture
        self.current_posture = posture
        self.posture_since = now
        self.last_posture_switch_time = now
        self.get_logger().info(f'Sentry posture: {old} -> {posture}, reason={reason}, used={used:.1f}s')

    def _posture_used_sec(self, posture: int) -> float:
        used = self.posture_cumulative.get(posture, 0.0)
        if posture == self.current_posture:
            used += max(0.0, self._now_sec() - self.posture_since)
        return used

    # ----------------------------- Publishers and utilities -----------------------------

    def set_tunnel_route_id(self, route_id: str) -> None:
        route_id = route_id or NORMAL_ROUTE_ID
        if route_id != self.current_route_id:
            self.get_logger().info(f'Tunnel route id: {self.current_route_id} -> {route_id}')
        self.current_route_id = route_id

    def _reset_tunnel_route_after_navigation(self) -> None:
        if self.current_route_id != NORMAL_ROUTE_ID:
            self.set_tunnel_route_id(NORMAL_ROUTE_ID)

    def _publish_periodic_topics(self) -> None:
        route_msg = String()
        route_msg.data = self.current_route_id
        self.tunnel_route_id_pub.publish(route_msg)

        posture_msg = UInt8()
        posture_msg.data = int(self.current_posture)
        self.posture_pub.publish(posture_msg)

        self._publish_debug_state()

    def _publish_debug_state(self) -> None:
        msg = self.latest_referee_msg
        debug = String()
        phase_name, phase_elapsed, phase_limit = self._patrol_phase_debug()
        fire_age = self._last_fire_age_sec()
        recent_fire = int(self._has_recent_fire())
        time_source = 'referee' if self._has_valid_referee_time(msg) else 'local'
        highland_elapsed = phase_elapsed if self.current_state == RobotState.CENTER_PATROL else 0.0
        if msg is None:
            debug.data = (
                f'state={self.current_state.name}, posture={self.current_posture}, '
                f'route={self.current_route_id}, current_zone={self.current_zone}, '
                f'target_zone={self.sequence_target_zone}, transition={self.sequence_active_transition}, '
                f'side={self.sequence_active_side}, sequence_final={self.sequence_final_target}, '
                f'sequence_index={self.sequence_index}, patrol_phase={phase_name}, '
                f'phase_elapsed={phase_elapsed:.1f}, phase_limit={phase_limit:.1f}, '
                f'highland_done={int(self.scheduled_highland_done)}, '
                f'highland_elapsed={highland_elapsed:.1f}, time_source={time_source}, '
                f'fire_age={fire_age:.1f}, recent_fire={recent_fire}'
            )
        else:
            debug.data = (
                f'state={self.current_state.name}, posture={self.current_posture}, route={self.current_route_id}, '
                f'game_state={int(msg.game_state)}, playing_states={list(self.playing_game_states)}, '
                f'hp={int(msg.self_health)}, bullet={int(msg.remaining_bullet)}, heat={int(msg.shoot_heat)}, '
                f'remaining={int(msg.remaining_time)}, active_target={self.navigation_controller.active_target_name}, '
                f'current_zone={self.current_zone}, target_zone={self.sequence_target_zone}, '
                f'transition={self.sequence_active_transition}, side={self.sequence_active_side}, '
                f'sequence_final={self.sequence_final_target}, sequence_index={self.sequence_index}, '
                f'patrol_phase={phase_name}, phase_elapsed={phase_elapsed:.1f}, phase_limit={phase_limit:.1f}, '
                f'highland_done={int(self.scheduled_highland_done)}, '
                f'highland_elapsed={highland_elapsed:.1f}, time_source={time_source}, '
                f'fire_age={fire_age:.1f}, recent_fire={recent_fire}'
            )
        self.debug_pub.publish(debug)

    def _patrol_phase_debug(self) -> Tuple[str, float, float]:
        if self.current_state == RobotState.CENTER_PATROL:
            phase_name = 'center'
            phase_limit = self.scheduled_highland_patrol_sec
        elif self.current_state == RobotState.HOME_PATROL:
            phase_name = 'home'
            phase_limit = math.inf
        else:
            return '', 0.0, 0.0

        if self.patrol_phase_started_at is None:
            return phase_name, 0.0, phase_limit
        return phase_name, max(0.0, self._match_elapsed_sec() - self.patrol_phase_started_at), phase_limit

    def _now_sec(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9


def main(args=None):
    rclpy.init(args=args)
    node = SmartQianshaoDecisionNode()
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
