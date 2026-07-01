from dataclasses import dataclass
import math
from typing import Dict, List, Optional, Tuple

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformException, TransformListener


@dataclass(frozen=True)
class TunnelRoute:
    route_id: str
    center: Tuple[float, float]
    trigger_radius: float
    release_radius: float
    min_follow_yaw_sec: float
    max_follow_yaw_sec: float


class TunnelModeManager(Node):
    def __init__(self) -> None:
        super().__init__('tunnel_mode_manager')

        self.map_frame = self.declare_parameter('map_frame', 'map').value
        self.robot_frame = self.declare_parameter('robot_frame', 'base_footprint').value
        self.route_topic = self.declare_parameter('route_topic', '/tunnel_route_id').value
        self.follow_yaw_topic = self.declare_parameter('follow_yaw_topic', '/if_follow_yaw').value
        self.status_topic = self.declare_parameter(
            'status_topic',
            '/tunnel_mode_manager/status',
        ).value
        self.publish_rate = float(self.declare_parameter('publish_rate', 10.0).value)
        self.route_timeout = float(self.declare_parameter('route_timeout', 1.0).value)
        self.tf_timeout = float(self.declare_parameter('tf_timeout', 0.1).value)

        route_ids_value = self.declare_parameter(
            'route_ids',
            [
                'self_left_upper_gate',
                'self_right_upper_gate',
                'enemy_left_upper_gate',
                'enemy_right_upper_gate',
            ],
        ).value
        self.route_ids = [str(route_id) for route_id in route_ids_value]
        self.routes = self._load_routes(self.route_ids)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.current_route_id = 'normal'
        self.last_route_stamp: Optional[Time] = None
        self.last_follow_yaw: Optional[bool] = None
        self.follow_yaw_latched = False
        self.follow_yaw_started_sec: Optional[float] = None
        self.route_completed = False
        self.active_route_id: Optional[str] = None
        self.last_center_distance: Optional[float] = None
        self.last_release_reason = 'idle'

        self.route_sub = self.create_subscription(
            String,
            self.route_topic,
            self._route_callback,
            10,
        )
        self.follow_yaw_pub = self.create_publisher(Bool, self.follow_yaw_topic, 10)
        self.status_pub = self.create_publisher(String, self.status_topic, 10)

        timer_period = 1.0 / max(self.publish_rate, 0.1)
        self.timer = self.create_timer(timer_period, self._timer_callback)

        self.get_logger().info(
            f'Loaded tunnel routes: {", ".join(self.routes.keys()) or "(none)"}'
        )

    def _load_routes(self, route_ids: List[str]) -> Dict[str, TunnelRoute]:
        routes: Dict[str, TunnelRoute] = {}
        for route_id in route_ids:
            # entry / exit are kept as backward-compatible defaults for center.
            entry = self._declare_float_array(f'{route_id}.entry', [0.0, 0.0])
            exit_point = self._declare_float_array(f'{route_id}.exit', [0.5, 0.0])
            default_center = [
                (entry[0] + exit_point[0]) * 0.5,
                (entry[1] + exit_point[1]) * 0.5,
            ]
            center = self._declare_float_array(f'{route_id}.center', default_center)

            legacy_trigger_radius = float(
                self.declare_parameter(f'{route_id}.pre_trigger_distance', 1.2).value
            )
            trigger_radius = float(
                self.declare_parameter(
                    f'{route_id}.trigger_radius',
                    legacy_trigger_radius,
                ).value
            )
            release_radius = float(
                self.declare_parameter(
                    f'{route_id}.release_radius',
                    trigger_radius + 0.2,
                ).value
            )
            if release_radius < trigger_radius:
                self.get_logger().warn(
                    f'Route "{route_id}" release_radius should be no smaller than '
                    'trigger_radius; clamping it'
                )
                release_radius = trigger_radius
            min_follow_yaw_sec = float(
                self.declare_parameter(f'{route_id}.min_follow_yaw_sec', 0.0).value
            )
            max_follow_yaw_sec = float(
                self.declare_parameter(f'{route_id}.max_follow_yaw_sec', 0.0).value
            )
            if min_follow_yaw_sec < 0.0:
                self.get_logger().warn(
                    f'Route "{route_id}" min_follow_yaw_sec must be non-negative; clamping it'
                )
                min_follow_yaw_sec = 0.0
            if max_follow_yaw_sec < 0.0:
                self.get_logger().warn(
                    f'Route "{route_id}" max_follow_yaw_sec must be non-negative; disabling it'
                )
                max_follow_yaw_sec = 0.0
            if 0.0 < max_follow_yaw_sec < min_follow_yaw_sec:
                self.get_logger().warn(
                    f'Route "{route_id}" max_follow_yaw_sec is smaller than '
                    'min_follow_yaw_sec; clamping max to min'
                )
                max_follow_yaw_sec = min_follow_yaw_sec

            route = TunnelRoute(
                route_id=route_id,
                center=(center[0], center[1]),
                trigger_radius=trigger_radius,
                release_radius=release_radius,
                min_follow_yaw_sec=min_follow_yaw_sec,
                max_follow_yaw_sec=max_follow_yaw_sec,
            )
            if trigger_radius <= 0.0:
                self.get_logger().error(
                    f'Ignoring route "{route_id}": trigger_radius must be positive'
                )
                continue
            if release_radius <= 0.0:
                self.get_logger().error(
                    f'Ignoring route "{route_id}": release_radius must be positive'
                )
                continue
            routes[route_id] = route
        return routes

    def _declare_float_array(self, name: str, default: List[float]) -> List[float]:
        value = self.declare_parameter(name, default).value
        if len(value) != 2:
            raise ValueError(f'Parameter "{name}" must contain exactly 2 numbers')
        return [float(value[0]), float(value[1])]

    def _route_callback(self, msg: String) -> None:
        route_id = msg.data.strip() or 'normal'
        if route_id != self.current_route_id:
            self.get_logger().info(f'Tunnel route id: {self.current_route_id} -> {route_id}')
            self._reset_route_state('route_changed')
        self.current_route_id = route_id
        self.last_route_stamp = self.get_clock().now()

    def _timer_callback(self) -> None:
        follow_yaw = self._should_follow_yaw()
        msg = Bool()
        msg.data = follow_yaw
        self.follow_yaw_pub.publish(msg)
        self._publish_status(follow_yaw)

        if self.last_follow_yaw is None or self.last_follow_yaw != follow_yaw:
            self.get_logger().info(f'Publishing {self.follow_yaw_topic}={follow_yaw}')
            self.last_follow_yaw = follow_yaw

    def _should_follow_yaw(self) -> bool:
        if self._route_is_stale():
            self._reset_route_state('route_stale')
            return False

        route = self.routes.get(self.current_route_id)
        if route is None:
            self._reset_route_state('route_inactive')
            return False

        return self._update_route_state(route)

    def _route_is_stale(self) -> bool:
        if self.route_timeout <= 0.0:
            return False
        if self.last_route_stamp is None:
            return True
        age = (self.get_clock().now() - self.last_route_stamp).nanoseconds * 1.0e-9
        return age > self.route_timeout

    def _lookup_robot_position(self) -> Optional[Tuple[float, float]]:
        try:
            transform = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.robot_frame,
                Time(),
                timeout=Duration(seconds=self.tf_timeout),
            )
        except TransformException as exc:
            self.get_logger().warn(
                f'Failed to lookup {self.map_frame}->{self.robot_frame}: {exc}',
                throttle_duration_sec=1.0,
            )
            return None

        translation = transform.transform.translation
        return float(translation.x), float(translation.y)

    def _reset_route_state(self, reason: str = 'reset') -> None:
        self.follow_yaw_latched = False
        self.follow_yaw_started_sec = None
        self.route_completed = False
        self.active_route_id = None
        self.last_center_distance = None
        self.last_release_reason = reason

    @staticmethod
    def _distance(first: Tuple[float, float], second: Tuple[float, float]) -> float:
        return math.hypot(first[0] - second[0], first[1] - second[1])

    def _update_route_state(
        self,
        route: TunnelRoute,
    ) -> bool:
        if not self.follow_yaw_latched or self.active_route_id != route.route_id:
            self.follow_yaw_latched = True
            self.follow_yaw_started_sec = self._now_sec()
            self.active_route_id = route.route_id
            self.route_completed = False
            self.last_release_reason = 'active'
            self.last_center_distance = None
            self.get_logger().info(f'Activated follow-yaw by route_id={route.route_id}')
            return True

        self.last_release_reason = 'active'
        return True

    def _publish_status(self, follow_yaw: bool) -> None:
        status = String()
        distance = 'nan'
        if self.last_center_distance is not None:
            distance = f'{self.last_center_distance:.3f}'
        status.data = (
            f'route_id={self.current_route_id}, '
            f'active_tunnel={self.active_route_id or ""}, '
            f'follow_yaw={int(follow_yaw)}, '
            f'latched={int(self.follow_yaw_latched)}, '
            f'completed={int(self.route_completed)}, '
            f'distance={distance}, '
            f'trigger_mode=route_id, '
            f'elapsed={self._active_elapsed_sec():.2f}, '
            f'reason={self.last_release_reason}'
        )
        self.status_pub.publish(status)

    def _active_elapsed_sec(self) -> float:
        if self.follow_yaw_started_sec is None:
            return 0.0
        return max(0.0, self._now_sec() - self.follow_yaw_started_sec)

    def _now_sec(self) -> float:
        return self.get_clock().now().nanoseconds * 1.0e-9


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TunnelModeManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
