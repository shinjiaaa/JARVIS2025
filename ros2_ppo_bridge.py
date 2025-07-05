#!/usr/bin/env python3
"""
ROS2 PPO Bridge - PPO 알고리즘과 ROS2 간의 브릿지 역할
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Twist
from std_msgs.msg import Bool, String
from sensor_msgs.msg import Image
import numpy as np
import time

class PPOBridge(Node):
    def __init__(self):
        super().__init__('ppo_bridge')
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.status_pub = self.create_publisher(String, '/drone_status', 10)
        
        # Subscribers
        self.ppo_enable_sub = self.create_subscription(
            Bool, '/ppo_enable', self.ppo_enable_callback, 10)
        self.ppo_target_sub = self.create_subscription(
            Point, '/ppo_target', self.ppo_target_callback, 10)
        
        # Variables
        self.ppo_enabled = False
        self.target_position = Point()
        self.current_position = Point()
        
        # Timer for control loop
        self.control_timer = self.create_timer(0.1, self.control_loop)
        
        self.get_logger().info('PPO Bridge initialized')
    
    def ppo_enable_callback(self, msg):
        """PPO 활성화/비활성화 콜백"""
        self.ppo_enabled = msg.data
        status = "PPO Enabled" if self.ppo_enabled else "PPO Disabled"
        self.get_logger().info(f'PPO Status: {status}')
        
        # 상태 발행
        status_msg = String()
        status_msg.data = status
        self.status_pub.publish(status_msg)
    
    def ppo_target_callback(self, msg):
        """PPO 목표 위치 콜백"""
        self.target_position = msg
        self.get_logger().info(f'Target position: x={msg.x:.2f}, y={msg.y:.2f}, z={msg.z:.2f}')
    
    def control_loop(self):
        """제어 루프"""
        if not self.ppo_enabled:
            return
        
        # 간단한 위치 제어 (실제 PPO 알고리즘 대신)
        cmd_vel = Twist()
        
        # 목표 위치로 이동하는 간단한 제어
        error_x = self.target_position.x - self.current_position.x
        error_y = self.target_position.y - self.current_position.y
        error_z = self.target_position.z - self.current_position.z
        
        # 간단한 P 제어
        kp = 0.5
        cmd_vel.linear.x = kp * error_x
        cmd_vel.linear.y = kp * error_y
        cmd_vel.linear.z = kp * error_z
        
        # 속도 제한
        max_vel = 2.0
        cmd_vel.linear.x = max(-max_vel, min(max_vel, cmd_vel.linear.x))
        cmd_vel.linear.y = max(-max_vel, min(max_vel, cmd_vel.linear.y))
        cmd_vel.linear.z = max(-max_vel, min(max_vel, cmd_vel.linear.z))
        
        self.cmd_vel_pub.publish(cmd_vel)

def main(args=None):
    rclpy.init(args=args)
    node = PPOBridge()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 