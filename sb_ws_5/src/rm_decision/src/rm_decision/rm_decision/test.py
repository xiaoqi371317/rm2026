import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from robo_utils.msg import ReferenceDataMini as RefereeData
from enum import Enum, auto
from typing import Dict, List, Tuple
from std_msgs.msg import UInt8


class RobotState(Enum):
    IDLE = auto()  # 空闲状态
    GOING_TO_OUTPOST = auto()  # 推前哨
    GOING_TO_HOME = auto()  # 回家补血/补弹
    CENTER_CRUISING = auto()  # 中央巡航
    HOME_CRUISING = auto()  # 己方半场巡航
    GOING_FORTRESS = auto()  # 去堡垒


class NavigationController:
    """封装导航相关逻辑"""

    def __init__(self, node: Node, action_server_name: str):
        self.node = node
        self.action_client = ActionClient(node, NavigateToPose, action_server_name)
        self.current_goal_handle = None

    def send_goal(self, position: Tuple[float, float]) -> None:
        """发送导航目标"""
        goal_pose = self._create_pose_stamped(*position)

        if not self.action_client.wait_for_server(timeout_sec=1.0):
            self.node.get_logger().error("Action server not available")
            return

        goal_msg = NavigateToPose.Goal(pose=goal_pose)
        send_goal_future = self.action_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self._goal_response_callback)

    def cancel_current_goal(self) -> None:
        """取消当前导航目标"""
        if self.current_goal_handle:
            self.node.get_logger().info("Canceling current goal...")
            cancel_future = self.current_goal_handle.cancel_goal_async()
            cancel_future.add_done_callback(self._cancel_done_callback)
            self.current_goal_handle = None

    def _create_pose_stamped(self, x: float, y: float) -> PoseStamped:
        """创建PoseStamped消息"""
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.node.get_clock().now().to_msg()
        pose.pose = Pose(
            position=Point(x=x, y=y, z=0.0),
            orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        )
        return pose

    def _goal_response_callback(self, future) -> None:
        """目标响应回调"""
        self.current_goal_handle = future.result()
        if not self.current_goal_handle.accepted:
            self.node.get_logger().warning("Goal rejected")
            return
        self.node.get_logger().info("Goal accepted")
        self.current_goal_handle.get_result_async().add_done_callback(self._result_callback)

    def _result_callback(self, future) -> None:
        """导航结果回调"""
        try:
            result = future.result().result
            self.node.get_logger().info(f"Navigation completed with result: {result}")
        except Exception as e:
            self.node.get_logger().error(f"Failed to get result: {str(e)}")

    def _cancel_done_callback(self, future) -> None:
        """取消操作回调"""
        try:
            response = future.result()
            if response.goals_canceled:
                self.node.get_logger().info("Goal canceled successfully")
            else:
                self.node.get_logger().warning("Cancel request failed")
        except Exception as e:
            self.node.get_logger().error(f"Cancel failed: {str(e)}")


class StateMachine:
    """状态机管理"""

    def __init__(self, node: Node):
        self.node = node
        self.current_state = RobotState.IDLE
        self.go_home_flag = 0
        self.cruise_index = 0  # 己方巡航索引
        self.posture = 0  # 哨兵姿态（导航目标类型）
        self.combat_posture = 3  # 战斗姿态：1=进攻, 2=防御, 3=移动
        self.is_red = None  # 己方颜色：None=未配置, True=红方, False=蓝方
        #self.is_red = False 
        self.self_cruising_list = []  # 己方巡航列表，待颜色配置后填充
        self.center_cruising_list = []  # 中心巡航列表，待颜色配置后填充
        self.stay_timer = None
        self.self_stay_timer = None

        self.stay_duration = 12  # 巡航停留时间
        self.self_stay_duration = 12  # 巡航停留时间

        # 初始化位置占位符
        self.home_position = [0.0, 0.0]
        self.enemy_outpost_position = [0.0, 0.0]
        self.fortress_position = [0.0, 0.0]

    def set_color(self, color: int) -> None:
        """根据己方颜色配置场地坐标 (0=红方, 1=蓝方)"""
        is_red = (color == 0)
        if self.is_red == is_red:
            return  # 颜色未变化，无需重新配置
        
        self.is_red = is_red

        if self.is_red:
            # 红方坐标配置
            self.self_cruising_list = [(16.20, 3.13), (13.62, -4.23),(19.84, -0.85),(19.95, 3.02)] # 己方巡航列表
            self.center_cruising_list = [(9.77, -1.78), (9.58, 4.18)]  # 中心巡航列表
            self.home_position = [21.62, 5.40]  # 补血点
            self.enemy_outpost_position = [6.80, 5.79]  # 敌方前哨站位置
            self.fortress_position = [2.69, 0.25]  # 堡垒点
            self.node.get_logger().info(f"Configured for RED side")
            self.node.get_logger().info(f"  Self cruising: {self.self_cruising_list}")
            self.node.get_logger().info(f"  Center cruising: {self.center_cruising_list}")
            self.node.get_logger().info(f"  Home: {self.home_position}")
        else:
            # 蓝方坐标配置
            self.self_cruising_list = [(16.20, 3.13), (13.62, -4.23),(19.84, -0.85),(19.95, 3.02)]  # 己方巡航列表
            self.center_cruising_list = [(8.67, 4.07), (8.82, -2.11)]  # 中心巡航列表
            self.home_position = [21.62, 5.40]  # 补血点
            self.enemy_outpost_position = [13.96, -3.39]  # 敌方前哨站位置
            self.fortress_position = [17.39, -1.35]  # 堡垒点
            self.node.get_logger().info(f"Configured for BLUE side")
            self.node.get_logger().info(f"  Self cruising: {self.self_cruising_list}")
            self.node.get_logger().info(f"  Center cruising: {self.center_cruising_list}")
            self.node.get_logger().info(f"  Home: {self.home_position}")

    def update(self, game_state, remaining_time, self_health,
               auto_aiming_status, remaining_bullet, at_castle_area) -> None:
        """更新状态机"""
        # 如果颜色还未配置，不执行任何导航操作
        if self.is_red is None:
            self.node.get_logger().debug("Waiting for color configuration...")
            return

        # 新消息定义：0-比赛未开始 1-比赛开始且为RMUC 2-比赛开始且为3V3
        if game_state > 0:
            # 回家补血
            if self._emergency_check(self_health):
                return

            # 回家补弹
            if self._emergency_check_remaining_bullet(remaining_bullet):
                return

            # TODO 上堡垒(比赛只有150秒或者己方基地少于4000血)
            # if self._handle_fortress_situation(remaining_time, self_base_hp):
            #     return

            # 推前哨站(比赛开始后的前60秒，即 remaining_time > 360)
            # if self._handle_outpost_situation(remaining_time):
                # return

            # 中央巡逻（比赛多于200秒）
            # if self._hander_center_cruise(remaining_time):
            #     return

            # 己方半场巡逻（保守策略）
            if self._hander_self_cruise(remaining_time):
                return
        else:
            pass

    def _emergency_check(self, health: int) -> bool:
        """紧急状态检查"""
        if health <= 100 and self.current_state != RobotState.GOING_TO_HOME:
            self.node.get_logger().warning(f"Emergency return! Health: {health}")
            self._transition_to(RobotState.GOING_TO_HOME, self.home_position)
            self.go_home_flag = 1
            self.posture = 1
            return True
        elif self.current_state == RobotState.GOING_TO_HOME:
            if health >= 400:
                self.current_state = RobotState.IDLE
                return False
            else:  # 血量比较少，继续回家状态
                return True
        return False

    def _emergency_check_remaining_bullet(self, remaining_bullet) -> bool:
        """回家补弹"""
        if remaining_bullet <= 10 and self.current_state != RobotState.GOING_TO_HOME:
            self.node.get_logger().warning(f"Emergency return! remaining_bullet: {remaining_bullet}")
            self._transition_to(RobotState.GOING_TO_HOME, self.home_position)
            self.go_home_flag = 1
            self.posture = 1
            return True
        elif self.current_state == RobotState.GOING_TO_HOME:
            if remaining_bullet >= 300:
                self.current_state = RobotState.IDLE
                return False
            else:
                return True
        return False

    def _handle_outpost_situation(self, remaining_time):
        """前60秒推前哨站（假设RMUC 7分钟比赛，remaining_time > 360为前60秒）"""
        if remaining_time > 300 and remaining_time < 360:
            if self.current_state != RobotState.GOING_TO_OUTPOST:
                self._transition_to(RobotState.GOING_TO_OUTPOST, self.enemy_outpost_position)
                self.posture = 2
                return True
            elif self.current_state == RobotState.GOING_TO_OUTPOST:
                return True
        else:
            # 60秒过后，如果还在推前哨，切回IDLE以便进入巡航
            if self.current_state == RobotState.GOING_TO_OUTPOST:
                self.current_state = RobotState.IDLE
        return False

    # TODO
    def _handle_fortress_situation(self, remaining_time, self_base_hp):
        """上堡垒"""
        if (remaining_time < 150 or self_base_hp < 4000):
            if (self.current_state != RobotState.GOING_FORTRESS):
                self._transition_to(RobotState.GOING_FORTRESS, self.fortress_position)
                return True
            elif self.current_state == RobotState.GOING_FORTRESS:
                return True
        return False

    def _hander_center_cruise(self, remaining_time):
        """中央巡航"""
        if (remaining_time > 0):
            if (self.current_state != RobotState.CENTER_CRUISING):
                self._start_center_cruise()
                self._start_stay_timer()
                return True
            elif self.current_state == RobotState.CENTER_CRUISING:
                if (remaining_time < 0):
                    self.current_state = RobotState.IDLE
                    return False
                return True

    def _hander_self_cruise(self, remaining_time):
        """己方半场巡航"""
        # 检查巡航列表是否为空
        if not self.self_cruising_list:
            self.node.get_logger().warn("self_cruising_list is empty, cannot start self cruise")
            return False
        
        if self.current_state != RobotState.HOME_CRUISING:
            self._start_self_cruise()
            self._start_self_stay_timer()
            self.posture = 1
            return True
        return True  # 已在巡航中，保持当前状态

    def _start_self_cruise(self) -> None:
        """开始己方巡航模式"""
        # 安全检查：确保列表不为空
        if not self.self_cruising_list:
            self.node.get_logger().error("Cannot start self cruise: self_cruising_list is empty")
            self.current_state = RobotState.IDLE
            return
        
        self.cruise_index = self.cruise_index % len(self.self_cruising_list)
        self.node.get_logger().info(f"Starting self cruise to point {self.cruise_index}: {self.self_cruising_list[self.cruise_index]}")
        self._transition_to(RobotState.HOME_CRUISING, self.self_cruising_list[self.cruise_index])

    def _start_self_stay_timer(self) -> None:
        """启动己方停留定时器"""
        if self.self_stay_timer:
            self.self_stay_timer.cancel()
        self.self_stay_timer = self.node.create_timer(self.self_stay_duration, self._handle_self_stay_complete)

    def _handle_self_stay_complete(self) -> None:
        try:
            # 取消当前定时器
            if self.self_stay_timer:
                self.self_stay_timer.cancel()
                self.self_stay_timer = None
            
            # 只有在巡航状态且列表非空时才继续
            if self.current_state != RobotState.HOME_CRUISING:
                self.node.get_logger().debug("Not in HOME_CRUISING state, stopping cruise")
                return
            
            if not self.self_cruising_list:
                self.node.get_logger().error("self_cruising_list is empty, stopping cruise")
                self.current_state = RobotState.IDLE
                return
            
            # 切换到下一个点
            old_index = self.cruise_index
            self.cruise_index = (self.cruise_index + 1) % len(self.self_cruising_list)
            target_point = self.self_cruising_list[self.cruise_index]
            
            self.node.get_logger().info(f"Self cruise: point {old_index} -> {self.cruise_index}, target: {target_point}")
            
            # 发送新目标
            self.node.navigation_controller.send_goal(target_point)
            
            # 重新启动定时器
            self.self_stay_timer = self.node.create_timer(self.self_stay_duration, self._handle_self_stay_complete)
            
        except Exception as e:
            self.node.get_logger().error(f"Error in _handle_self_stay_complete: {str(e)}")
            self.current_state = RobotState.IDLE

    def _handle_normal_operation(self, game_state: int, health: int) -> None:
        """处理正常操作"""
        if self.current_state == RobotState.IDLE and game_state == 1:
            target_state = RobotState.GOING_TO_HOME if health <= 150 else RobotState.GOING_TO_FIRST
            self._transition_to(target_state, self.targets['home' if health <= 150 else 'first_point'])

    def _update_combat_posture(self, game_state, self_health, remaining_bullet, auto_aiming_status,
                               energy_percent) -> None:
        """更新战斗姿态：1=进攻姿态, 2=防御姿态, 3=移动姿态"""
        if game_state == 0 or game_state == 0xff:
            self.combat_posture = 3
            return

        is_emergency = (self_health <= 100) or (remaining_bullet <= 10)

        if is_emergency:
            if energy_percent > 30:
                self.combat_posture = 2  # 防御姿态
            else:
                self.combat_posture = 3  # 移动姿态
        else:
            if auto_aiming_status != 0 and energy_percent > 30:
                self.combat_posture = 1  # 进攻姿态
            else:
                self.combat_posture = 3  # 移动姿态

    def _transition_to(self, new_state: RobotState, target: Tuple[float, float]) -> None:
        """执行状态转换"""
        self.node.get_logger().info(f"State transition: {self.current_state} -> {new_state}, target: {target}")
        # 切换状态前取消旧导航目标，避免 Nav2 同时存在多个活跃 goal
        self.node.navigation_controller.cancel_current_goal()
        self.current_state = new_state
        self.node.navigation_controller.send_goal(target)

    def _start_center_cruise(self) -> None:
        """开始中心巡航模式"""
        # 安全检查：确保列表不为空
        if not self.center_cruising_list:
            self.node.get_logger().error("Cannot start center cruise: center_cruising_list is empty")
            self.current_state = RobotState.IDLE
            return
        
        self.center_cruise_index = self.center_cruise_index % len(self.center_cruising_list)
        self._transition_to(RobotState.CENTER_CRUISING, self.center_cruising_list[self.center_cruise_index])

    def _start_stay_timer(self) -> None:
        """启动停留定时器"""
        if self.stay_timer:
            self.stay_timer.cancel()
        self.stay_timer = self.node.create_timer(self.stay_duration, self._handle_stay_complete)

    def _handle_stay_complete(self) -> None:
        """处理中心巡航停留结束"""
        if self.current_state == RobotState.CENTER_CRUISING:
            # 取消定时器，防止周期性重复触发
            if self.stay_timer:
                self.stay_timer.cancel()
                self.stay_timer = None
            
            # 安全检查：确保列表不为空
            if not self.center_cruising_list:
                self.node.get_logger().error("Cannot continue center cruise: center_cruising_list is empty")
                self.current_state = RobotState.IDLE
                return
            
            self.center_cruise_index = (self.center_cruise_index + 1) % len(self.center_cruising_list)
            self._transition_to(RobotState.CENTER_CRUISING, self.center_cruising_list[self.center_cruise_index])


class Nav2DecisionNode(Node):
    def __init__(self):
        super().__init__('nav_decision_node')
        self._declare_parameters()

        # 初始化子系统
        self.navigation_controller = NavigationController(self, '/navigate_to_pose')
        self.state_machine = StateMachine(self)

        # 订阅器
        self.ref_data_sub = self.create_subscription(
            RefereeData,
            '/offboardlink/reference_data_mini',
            self._ref_data_callback,
            10
        )

        self.posture_pub = self.create_publisher(
            UInt8,
            '/change_posture',
            10
        )
        
        # 状态更新定时器
        self.create_timer(0.5, self._update)

    def _declare_parameters(self) -> None:
        """声明所有参数"""
        # 声明参数（实际不会使用，因为坐标由 set_color 硬编码）
        self.declare_parameter('home_position', [21.00, 6.25])
        self.declare_parameter('enemy_outpost_position', [9.75, -1.76])
        self.declare_parameter('self_outpost_position', [7.68, -4.55])
        self.declare_parameter('fortress_position', [2.69, 0.25])
        self.declare_parameter('stay_duration', 2.0)

        self.current_self_color = 0xff  # 0=红方, 1=蓝方, 0xff=未知
        self.game_state = 0xff  # 比赛状态
        self.remaining_time = 0xff  # 剩余时间
        self.self_health = 0xff  # 自身血量
        self.auto_aiming_status = 0xff  # 是否处于自瞄状态
        self.remaining_bullet = 0xff  # 剩余子弹
        self.energy_percent = 0xff  # 剩余能量百分比
        self.at_castle_area = 0xff  # 是否在堡垒

    def _ref_data_callback(self, msg: RefereeData) -> None:
        """裁判系统数据回调"""
        self.game_state = msg.game_state
        self.remaining_time = msg.remaining_time

        # 根据 self_color 动态配置红蓝方坐标 (0=红方, 1=蓝方)
        if msg.self_color != self.current_self_color:
            self.current_self_color = msg.self_color
            self.state_machine.set_color(msg.self_color)
            self.get_logger().info(f"Color configured: {'RED' if msg.self_color == 0 else 'BLUE'}")

        self.self_health = msg.self_health
        self.auto_aiming_status = msg.auto_aiming_status
        self.remaining_bullet = msg.remaining_bullet
        self.energy_percent = msg.energy_percent
        self.at_castle_area = msg.at_castle_area

    def _update(self) -> None:
        """定时状态更新"""
        self.state_machine.update(
            game_state=self.game_state,
            remaining_time=self.remaining_time,
            self_health=self.self_health,
            auto_aiming_status=self.auto_aiming_status,
            remaining_bullet=self.remaining_bullet,
            at_castle_area=self.at_castle_area
        )
        
        # 更新战斗姿态（进攻/防御/移动），独立于导航状态机运行
        self.state_machine._update_combat_posture(
            game_state=self.game_state,
            self_health=self.self_health,
            remaining_bullet=self.remaining_bullet,
            auto_aiming_status=self.auto_aiming_status,
            energy_percent=self.energy_percent
        )
        
        msg = UInt8()
        msg.data = self.state_machine.combat_posture
        self.posture_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = Nav2DecisionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down...")
    finally:
        node.navigation_controller.cancel_current_goal()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main() 