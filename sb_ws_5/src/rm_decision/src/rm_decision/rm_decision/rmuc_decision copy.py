import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
# from rm_decision_interfaces.msg import RefereeData
from enum import Enum, auto
from typing import Dict, List, Tuple

""" 家里测试的版本 """

class RobotState(Enum):
    IDLE = auto()               # 空闲状态
    GOING_TO_OUTPOST = auto()   # 推前哨
    GOING_TO_HOME = auto()      # 回家补血/补弹
    CENTER_CRUISING = auto()    # 中央巡航
    HOME_CRUISING = auto()      # 己方半场巡航
    GOING_FORTRESS = auto()     # 去堡垒

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
        self.cruise_index = 0
        self.center_cruise_index = 0
        self.center_cruising_list = [(11.7, -1.83),(11.47, 2.92),(9.59, 2.34),(10.37, -0.39)]
        self.stay_timer = None

        self.stay_duration = 12
        
        # 配置参数
        self._load_configuration()

        

    def _load_configuration(self) -> None:
        """加载配置参数"""

        self.home_position = self.node.get_parameter('home_position').value
        # self.cruise_points = self.node.get_parameter('cruise_points').value
        # self.center_gain_points = self.node.get_parameter('center_gain_points').value
        # self.stay_duration = self.node.get_parameter('stay_duration').value

        self.enemy_outpost_position = self.node.get_parameter('enemy_outpost_position').value

        self.fortress_position = self.node.get_parameter('fortress_position').value

    def update(self, game_state, remaining_time, self_health,
                auto_aiming_status, remaining_bullet, enemy_outpost_hp,
                 self_outpost_hp, self_base_hp) -> None:
        
        """更新状态机"""
        if(game_state == 1):
            # 回家咯
            if self._emergency_check(self_health):
                return
            
            # 上堡垒(比赛只有150秒或者己方基地少于4000血)
            if self._handle_fortress_situation(remaining_time, self_base_hp):
                return
            
            # 推前哨站(比赛时间多于300秒 且敌方前哨站有血量)
            if self._handle_outpost_situation(remaining_time, enemy_outpost_hp):
                return

            
            
            # 中央巡逻（比赛多于200秒）
            if self._hander_center_cruise(remaining_time):
                return
            
            # TODO 己方半场巡逻（保守策略）
            


            # if self._handle_center_gain(center_status, health):
            #     return
            
            # self._handle_normal_operation(game_state, health)
        else:
            pass

    def _emergency_check(self, health: int) -> bool:
        """紧急状态检查"""
        if health <= 100 and self.current_state != RobotState.GOING_TO_HOME:
            self.node.get_logger().warning(f"Emergency return! Health: {health}")
            self._transition_to(RobotState.GOING_TO_HOME, self.home_position)
            return True
        elif self.current_state == RobotState.GOING_TO_HOME:
            if health >=300:
                self.current_state == RobotState.IDLE
                return False
            else:  #血量比较少，继续回家状态
                return True
        return False

    def _handle_outpost_situation(self,remaining_time,enemy_outpost_hp):
        "推前哨站"
        if(remaining_time > 300 and enemy_outpost_hp > 0):
            if self.current_state != RobotState.GOING_TO_OUTPOST:
                self._transition_to(RobotState.GOING_TO_OUTPOST, 
                                        self.enemy_outpost_position)
                return True
            elif self.current_state == RobotState.GOING_TO_OUTPOST:
                if remaining_time > 300 and enemy_outpost_hp > 0:
                    return True
                else:
                    self.current_state == RobotState.IDLE
                    return False
        return False
    
    def _handle_fortress_situation(self,remaining_time, self_base_hp):
        "上堡垒"
        if(remaining_time < 150 or self_base_hp < 4000):
            if(self.current_state != RobotState.GOING_FORTRESS):
                self._transition_to(RobotState.GOING_FORTRESS, self.fortress_position)
                return True
            elif self.current_state == RobotState.GOING_FORTRESS:
                return True
        return False
    
    def _hander_center_cruise(self,remaining_time):
        "中央巡航"
        if(remaining_time > 200):
            if(self.current_state != RobotState.CENTER_CRUISING):
                self._start_center_cruise()
                self._start_stay_timer()
                return True
            elif self.current_state == RobotState.CENTER_CRUISING:
                if(remaining_time < 200):
                    self.current_state == RobotState.IDLE
                    return False
                return True



    def _handle_normal_operation(self, game_state: int, health: int) -> None:
        """处理正常操作"""
        if self.current_state == RobotState.IDLE and game_state == 1:
            target_state = RobotState.GOING_TO_HOME if health <= 150 else RobotState.GOING_TO_FIRST
            self._transition_to(target_state, self.targets['home' if health <=150 else 'first_point'])
        

    def _transition_to(self, new_state: RobotState, target: Tuple[float, float]) -> None:
        """执行状态转换"""
        self.node.get_logger().info(f"State transition: {self.current_state} -> {new_state}")
        self.current_state = new_state
        self.node.navigation_controller.send_goal(target)

    # def handle_navigation_complete(self) -> None:
    #     """处理导航完成事件"""
    #     if self.current_state == RobotState.GOING_TO_FIRST:
    #         self._start_cruise()
    #     elif self.current_state in [RobotState.CRUISING, RobotState.CENTER_GAIN_CRUISE]:
    #         self._start_stay_timer()

    def _start_center_cruise(self) -> None:
        """开始中心巡航模式"""
        self.center_cruise_index = self.center_cruise_index % 4
        self._transition_to(RobotState.CENTER_CRUISING, self.center_cruising_list[self.center_cruise_index])

    def _start_stay_timer(self) -> None:
        """启动停留定时器"""
        if self.stay_timer:
            self.stay_timer.cancel()
        self.stay_timer = self.node.create_timer(self.stay_duration, self._handle_stay_complete)

    def _handle_stay_complete(self) -> None:
        """处理中心巡航停留结束"""
        # self.stay_timer.cancel()
        if self.current_state == RobotState.CENTER_CRUISING:
            self.center_cruise_index = (self.center_cruise_index + 1) % len(self.center_cruising_list)
            self._transition_to(RobotState.CENTER_CRUISING, self.center_cruising_list[self.center_cruise_index])
        # elif self.current_state == RobotState.CENTER_GAIN_CRUISE:
        #     self.center_cruise_index = (self.center_cruise_index + 1) % len(self.center_gain_points)
        #     self._transition_to(RobotState.CENTER_GAIN_CRUISE, self.center_gain_points[self.center_cruise_index])

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
            '/offboardlink/reference_data_fake',
            self._ref_data_callback,
            10
        )
        
        # 状态更新定时器
        self.create_timer(0.5, self._update)

    def _declare_parameters(self) -> None:
        """声明所有参数"""
        self.declare_parameter('home_position', [-1.11, -4.06])  # 补血点
        self.declare_parameter('enemy_outpost_position', [11.71, 5.04])  # 敌方前哨站位置
        self.declare_parameter('self_outpost_position', [7.45, -3.09])  # 己方前哨站(保护)
        # self.declare_parameter('center_cruisiong', [
        #     [12.07, -4.59], 
        #     [12.08, 1.62],  
        #     [8.78, -0.40],  
        #     [9.69, -5.08]  
        # ])
        self.declare_parameter('fortress_position', [0.33, 1.88])  # 堡垒点

        self.declare_parameter('stay_duration', 2.0)

        self.game_state = 0xff
        self.remaining_time = 0xff
        self.self_health = 0xff
        self.auto_aiming_status = 0xff
        self.remaining_bullet = 0xff
        self.enemy_outpost_hp = 0xff
        self.self_outpost_hp = 0xff
        self.self_base_hp = 0xff

        

    def _ref_data_callback(self, msg: RefereeData) -> None:
        """裁判系统数据回调"""
        self.game_state = msg.game_state
        self.remaining_time = msg.remaining_time
        self.self_health = msg.self_health
        self.auto_aiming_status = msg.auto_aiming_status
        self.remaining_bullet = msg.remaining_bullet
        self.enemy_outpost_hp = msg.enemy_outpost_hp
        self.self_outpost_hp = msg.self_outpost_hp
        self.self_base_hp = msg.self_base_hp

    def _update(self) -> None:
        """定时状态更新"""
        self.state_machine.update(
            game_state=self.game_state,
            remaining_time = self.remaining_time,
            self_health = self.self_health,
            auto_aiming_status = self.auto_aiming_status,
            remaining_bullet = self.remaining_bullet,
            enemy_outpost_hp = self.enemy_outpost_hp,
            self_outpost_hp = self.self_outpost_hp,
            self_base_hp = self.self_base_hp,
        )

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