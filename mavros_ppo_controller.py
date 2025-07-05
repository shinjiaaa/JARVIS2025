#!/usr/bin/env python3
"""
MAVROS PPO Controller - MAVROS를 통한 드론 제어
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import State, OverrideRCIn
from mavros_msgs.srv import CommandBool, CommandTOL, SetMode
from std_msgs.msg import Bool, String
import time

class MavrosPPOController(Node):
    def __init__(self):
        super().__init__('mavros_ppo_controller')
        
        # MAVROS 상태 구독
        self.state_sub = self.create_subscription(
            State, '/mavros/state', self.state_callback, 10)
        
        # 로컬 위치 발행
        self.local_pos_pub = self.create_publisher(
            PoseStamped, '/mavros/setpoint_position/local', 10)
        
        # 속도 명령 발행
        self.vel_pub = self.create_publisher(
            TwistStamped, '/mavros/setpoint_velocity/cmd_vel', 10)
        
        # 드론 상태 발행
        self.status_pub = self.create_publisher(String, '/drone_status', 10)
        
        # 서비스 클라이언트
        self.arming_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.takeoff_client = self.create_client(CommandTOL, '/mavros/cmd/takeoff')
        self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')
        
        # 변수
        self.current_state = State()
        self.armed = False
        self.offboard_enabled = False
        
        # 타이머
        self.status_timer = self.create_timer(1.0, self.publish_status)
        
        self.get_logger().info('MAVROS PPO Controller initialized')
    
    def state_callback(self, msg):
        """MAVROS 상태 콜백"""
        self.current_state = msg
        self.armed = msg.armed
        self.offboard_enabled = msg.mode == "OFFBOARD"
    
    def publish_status(self):
        """드론 상태 발행"""
        status_msg = String()
        status_msg.data = f"Armed: {self.armed}, Mode: {self.current_state.mode}, Connected: {self.current_state.connected}"
        self.status_pub.publish(status_msg)
    
    def arm_drone(self):
        """드론 ARM"""
        if not self.arming_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('Arming service not available')
            return False
        
        request = CommandBool.Request()
        request.value = True
        
        future = self.arming_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() is not None:
            self.get_logger().info('Drone armed successfully')
            return True
        else:
            self.get_logger().error('Failed to arm drone')
            return False
    
    def takeoff(self, altitude=2.0):
        """드론 이륙"""
        if not self.takeoff_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('Takeoff service not available')
            return False
        
        request = CommandTOL.Request()
        request.altitude = altitude
        
        future = self.takeoff_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() is not None:
            self.get_logger().info(f'Takeoff to {altitude}m initiated')
            return True
        else:
            self.get_logger().error('Failed to takeoff')
            return False
    
    def set_mode(self, mode):
        """드론 모드 설정"""
        if not self.set_mode_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('Set mode service not available')
            return False
        
        request = SetMode.Request()
        request.custom_mode = mode
        
        future = self.set_mode_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() is not None:
            self.get_logger().info(f'Mode set to {mode}')
            return True
        else:
            self.get_logger().error(f'Failed to set mode to {mode}')
            return False

def main(args=None):
    rclpy.init(args=args)
    node = MavrosPPOController()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 