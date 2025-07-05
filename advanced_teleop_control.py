#!/usr/bin/env python3
"""
🚁 고급 쿼드콥터 텔레오퍼레이션 시스템
실시간 키보드 제어, 연속적인 움직임, 향상된 UI를 제공합니다.

주요 기능:
- 실시간 키보드 입력 감지
- 연속적인 움직임 (키를 누르고 있으면 계속 이동)
- 속도 조절 기능
- 안전 기능 (고도 제한, 경계 체크)
- 실시간 상태 모니터링
- 스무스한 조작감

조작법:
====================
이동 제어:
- W/S     : 앞/뒤 이동 (Pitch)
- A/D     : 왼쪽/오른쪽 이동 (Roll)  
- Q/E     : 회전 (Yaw)
- R/F     : 상승/하강 (Throttle)
- 방향키  : 세밀한 위치 조정

비행 제어:
- SPACE   : 이륙/호버링
- X       : 착륙
- H       : 홈 포지션으로 복귀

시스템 제어:
- T       : 제어 ON/OFF 토글
- 1-5     : 속도 레벨 (1=매우느림, 5=빠름)
- I       : 상태 정보 토글
- ESC/Z   : 긴급 정지 및 종료

안전 기능:
- 고도 제한 (0.3m ~ 15m)
- 경계 영역 체크 (-20m ~ 20m)
- 긴급 정지 기능
- 자동 착륙 보호
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Twist, Vector3
from std_msgs.msg import Bool, String, Float32
import sys
import termios
import tty
import select
import threading
import time
import math
import os
import signal
from collections import defaultdict

class AdvancedTeleopController(Node):
    def __init__(self):
        super().__init__('advanced_teleop_controller')
        
        # ROS2 Publishers
        self.target_publisher = self.create_publisher(Point, '/ppo_target', 10)
        self.enable_publisher = self.create_publisher(Bool, '/ppo_enable', 10)
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.status_publisher = self.create_publisher(String, '/teleop_status', 10)
        
        # ROS2 Subscribers
        self.drone_status_sub = self.create_subscription(
            String, '/drone_status', self.drone_status_callback, 10)
        self.position_sub = self.create_subscription(
            Point, '/drone_position', self.position_callback, 10)
        
        # 드론 상태
        self.current_position = [0.0, 0.0, 0.0]  # [x, y, z]
        self.target_position = [0.0, 0.0, 0.0]   # [x, y, z]
        self.home_position = [0.0, 0.0, 0.0]     # 홈 포지션
        self.current_yaw = 0.0                   # 현재 방향
        self.target_yaw = 0.0                    # 목표 방향
        
        # 제어 상태
        self.is_enabled = False
        self.is_flying = False
        self.emergency_stop = False
        self.auto_mode = False
        
        # 이동 설정
        self.speed_levels = {
            1: {'linear': 0.3, 'vertical': 0.2, 'angular': 0.2},   # 매우 느림
            2: {'linear': 0.6, 'vertical': 0.3, 'angular': 0.3},   # 느림
            3: {'linear': 1.0, 'vertical': 0.5, 'angular': 0.5},   # 보통
            4: {'linear': 1.5, 'vertical': 0.7, 'angular': 0.7},   # 빠름
            5: {'linear': 2.0, 'vertical': 1.0, 'angular': 1.0}    # 매우 빠름
        }
        self.current_speed_level = 3
        self.speed = self.speed_levels[self.current_speed_level]
        
        # 안전 설정
        self.min_altitude = 0.3      # 최소 고도
        self.max_altitude = 15.0     # 최대 고도
        self.max_distance = 20.0     # 홈으로부터 최대 거리
        self.takeoff_height = 2.0    # 기본 이륙 높이
        
        # UI 설정
        self.show_info = True
        self.last_key_time = time.time()
        self.key_timeout = 0.1       # 키 입력 타임아웃
        
        # 현재 누르고 있는 키들
        self.active_keys = set()
        self.key_pressed = defaultdict(bool)
        
        # 터미널 설정
        self.old_settings = termios.tcgetattr(sys.stdin)
        tty.setraw(sys.stdin.fileno())
        
        # 제어 루프 타이머
        self.control_timer = self.create_timer(0.05, self.control_loop)  # 20Hz
        
        # 상태 표시 스레드
        self.running = True
        self.display_thread = threading.Thread(target=self.display_loop)
        self.display_thread.daemon = True
        self.display_thread.start()
        
        # 키 입력 스레드
        self.key_thread = threading.Thread(target=self.key_input_loop)
        self.key_thread.daemon = True
        self.key_thread.start()
        
        # 신호 처리
        signal.signal(signal.SIGINT, self.signal_handler)
        
        # 초기화
        self.clear_screen()
        self.print_welcome()
        self.get_logger().info("🚁 고급 텔레오퍼레이션 시스템 시작!")

    def signal_handler(self, signum, frame):
        """신호 처리 (Ctrl+C)"""
        self.cleanup()
        sys.exit(0)

    def clear_screen(self):
        """화면 클리어"""
        os.system('cls' if os.name == 'nt' else 'clear')
    
    def print_welcome(self):
        """환영 메시지 출력"""
        print("╔════════════════════════════════════════════════════════════════╗")
        print("║                🚁 고급 쿼드콥터 텔레오퍼레이션                 ║")
        print("╠════════════════════════════════════════════════════════════════╣")
        print("║  이동: W/A/S/D  상하: R/F  회전: Q/E  방향키: 세밀조정        ║")
        print("║  비행: SPACE(이륙) X(착륙) H(홈복귀)  제어: T(토글)           ║") 
        print("║  속도: 1-5  정보: I  긴급정지: ESC/Z                          ║")
        print("╚════════════════════════════════════════════════════════════════╝")
        print()

    def drone_status_callback(self, msg):
        """드론 상태 콜백"""
        try:
            status_str = msg.data
            if "제어:" in status_str:
                self.is_enabled = "ON" in status_str.split("제어:")[1]
        except:
            pass

    def position_callback(self, msg):
        """위치 정보 콜백"""
        self.current_position = [msg.x, msg.y, msg.z]

    def key_input_loop(self):
        """키 입력 감지 루프"""
        while self.running and rclpy.ok():
            try:
                key = self.get_key_nonblocking()
                if key:
                    self.process_key(key)
                time.sleep(0.01)  # 100Hz 키 체크
            except:
                break

    def get_key_nonblocking(self):
        """비차단 키 입력"""
        if select.select([sys.stdin], [], [], 0)[0]:
            key = sys.stdin.read(1)
            
            # ESC 시퀀스 처리 (방향키)
            if ord(key) == 27:
                try:
                    next1 = sys.stdin.read(1)
                    next2 = sys.stdin.read(1)
                    if ord(next1) == 91:  # '['
                        if ord(next2) == 65: return 'UP'
                        elif ord(next2) == 66: return 'DOWN'
                        elif ord(next2) == 67: return 'RIGHT'  
                        elif ord(next2) == 68: return 'LEFT'
                except:
                    pass
                return 'ESC'
            
            return key
        return None

    def process_key(self, key):
        """키 입력 처리"""
        self.last_key_time = time.time()
        
        # 긴급 정지
        if key in ['ESC', 'z', 'Z']:
            self.emergency_stop = True
            self.active_keys.clear()
            return
        
        # 연속 이동 키들
        movement_keys = ['w', 's', 'a', 'd', 'q', 'e', 'r', 'f', 
                        'UP', 'DOWN', 'LEFT', 'RIGHT']
        
        if key.lower() in [k.lower() for k in movement_keys] or key in movement_keys:
            if key not in self.active_keys:
                self.active_keys.add(key)
        
        # 즉시 실행 키들
        else:
            # 이륙/착륙
            if key == ' ':  # SPACE
                self.takeoff()
            elif key in ['x', 'X']:
                self.land()
            elif key in ['h', 'H']:
                self.return_home()
            
            # 제어 토글
            elif key in ['t', 'T']:
                self.toggle_control()
            
            # 속도 조절
            elif key in '12345':
                self.set_speed_level(int(key))
            
            # 정보 토글
            elif key in ['i', 'I']:
                self.show_info = not self.show_info
            
            # 도움말
            elif key in ['?', '/']:
                self.print_help()

    def control_loop(self):
        """메인 제어 루프 (20Hz)"""
        if not self.running:
            return
            
        # 긴급 정지 처리
        if self.emergency_stop:
            self.emergency_landing()
            return
        
        # 키 타임아웃 체크 (키를 떼면 정지)
        if time.time() - self.last_key_time > self.key_timeout:
            self.active_keys.clear()
        
        # 연속 이동 처리
        if self.active_keys and self.is_enabled:
            self.process_continuous_movement()

    def process_continuous_movement(self):
        """연속적인 움직임 처리"""
        dx, dy, dz, dyaw = 0, 0, 0, 0
        
        for key in self.active_keys:
            # 기본 이동 (W/A/S/D)
            if key.lower() == 'w':  # 앞으로
                dy += self.speed['linear']
            elif key.lower() == 's':  # 뒤로
                dy -= self.speed['linear']
            elif key.lower() == 'a':  # 왼쪽
                dx -= self.speed['linear']
            elif key.lower() == 'd':  # 오른쪽
                dx += self.speed['linear']
            
            # 상하 이동
            elif key.lower() == 'r':  # 상승
                dz += self.speed['vertical']
            elif key.lower() == 'f':  # 하강
                dz -= self.speed['vertical']
            
            # 회전
            elif key.lower() == 'q':  # 좌회전
                dyaw += self.speed['angular']
            elif key.lower() == 'e':  # 우회전
                dyaw -= self.speed['angular']
            
            # 방향키 (세밀한 조정)
            elif key == 'UP':
                dy += self.speed['linear'] * 0.3
            elif key == 'DOWN':
                dy -= self.speed['linear'] * 0.3
            elif key == 'LEFT':
                dx -= self.speed['linear'] * 0.3
            elif key == 'RIGHT':
                dx += self.speed['linear'] * 0.3
        
        # 목표 위치 업데이트
        if dx != 0 or dy != 0 or dz != 0:
            self.update_target_position(dx, dy, dz)
        
        # 회전 업데이트  
        if dyaw != 0:
            self.update_target_yaw(dyaw)

    def update_target_position(self, dx, dy, dz):
        """목표 위치 업데이트"""
        # 새로운 목표 위치 계산
        new_x = self.target_position[0] + dx * 0.05  # 20Hz이므로 0.05초 간격
        new_y = self.target_position[1] + dy * 0.05
        new_z = self.target_position[2] + dz * 0.05
        
        # 안전 체크
        new_x, new_y, new_z = self.safety_check(new_x, new_y, new_z)
        
        # 목표 위치 설정
        self.target_position = [new_x, new_y, new_z]
        self.send_target_position()

    def update_target_yaw(self, dyaw):
        """목표 방향 업데이트"""
        self.target_yaw += dyaw * 0.05
        # -π ~ π 범위로 정규화
        self.target_yaw = math.atan2(math.sin(self.target_yaw), math.cos(self.target_yaw))

    def safety_check(self, x, y, z):
        """안전 체크"""
        # 고도 제한
        z = max(self.min_altitude, min(self.max_altitude, z))
        
        # 거리 제한 (홈으로부터)
        distance = math.sqrt((x - self.home_position[0])**2 + (y - self.home_position[1])**2)
        if distance > self.max_distance:
            # 최대 거리를 넘으면 방향은 유지하되 거리를 제한
            scale = self.max_distance / distance
            x = self.home_position[0] + (x - self.home_position[0]) * scale
            y = self.home_position[1] + (y - self.home_position[1]) * scale
        
        return x, y, z

    def send_target_position(self):
        """목표 위치 전송"""
        msg = Point()
        msg.x = float(self.target_position[0])
        msg.y = float(self.target_position[1])
        msg.z = float(self.target_position[2])
        self.target_publisher.publish(msg)

    def takeoff(self):
        """이륙"""
        if not self.is_flying:
            self.enable_control(True)
            self.target_position[2] = self.takeoff_height
            self.is_flying = True
            self.send_target_position()
            self.get_logger().info(f"🛫 이륙 - 목표 고도: {self.takeoff_height}m")

    def land(self):
        """착륙"""
        self.target_position[2] = 0.3  # 착륙 고도
        self.is_flying = False
        self.send_target_position()
        self.get_logger().info("🛬 착륙 시작")

    def return_home(self):
        """홈 포지션으로 복귀"""
        self.target_position = self.home_position.copy()
        self.target_position[2] = max(self.target_position[2], self.takeoff_height)
        self.send_target_position()
        self.get_logger().info("🏠 홈 포지션으로 복귀")

    def emergency_landing(self):
        """긴급 착륙"""
        self.target_position[2] = 0.2
        self.is_flying = False
        self.active_keys.clear()
        self.send_target_position()
        self.get_logger().warning("🚨 긴급 착륙!")

    def toggle_control(self):
        """제어 토글"""
        self.enable_control(not self.is_enabled)

    def enable_control(self, enable=True):
        """제어 활성화/비활성화"""
        msg = Bool()
        msg.data = enable
        self.enable_publisher.publish(msg)
        self.is_enabled = enable
        
        if enable:
            # 현재 위치를 목표 위치로 설정
            self.target_position = self.current_position.copy()
        
        self.get_logger().info(f"🔧 제어 {'활성화' if enable else '비활성화'}")

    def set_speed_level(self, level):
        """속도 레벨 설정"""
        if level in self.speed_levels:
            self.current_speed_level = level
            self.speed = self.speed_levels[level]
            self.get_logger().info(f"⚡ 속도 레벨: {level} ({'매우느림' if level==1 else '느림' if level==2 else '보통' if level==3 else '빠름' if level==4 else '매우빠름'})")

    def display_loop(self):
        """상태 표시 루프"""
        while self.running:
            try:
                if self.show_info:
                    self.display_status()
                time.sleep(0.2)  # 5Hz 업데이트
            except:
                break

    def display_status(self):
        """실시간 상태 표시"""
        # 커서를 홈으로 이동하여 덮어쓰기
        print("\033[8;0H", end="")  # 8번째 줄부터 시작
        
        print("╔════════════════════════════════════════════════════════════════╗")
        print("║                        🚁 드론 상태 정보                       ║")
        print("╠════════════════════════════════════════════════════════════════╣")
        
        # 위치 정보
        print(f"║ 📍 현재위치: ({self.current_position[0]:6.2f}, {self.current_position[1]:6.2f}, {self.current_position[2]:6.2f})     ║")
        print(f"║ 🎯 목표위치: ({self.target_position[0]:6.2f}, {self.target_position[1]:6.2f}, {self.target_position[2]:6.2f})     ║")
        
        # 홈으로부터의 거리
        home_dist = math.sqrt(sum([(c-h)**2 for c, h in zip(self.current_position, self.home_position)]))
        print(f"║ 🏠 홈거리: {home_dist:6.2f}m                                        ║")
        
        # 제어 상태
        control_status = "🟢 ON " if self.is_enabled else "🔴 OFF"
        flight_status = "🛫 비행중" if self.is_flying else "🛬 지상  "
        print(f"║ 🔧 제어: {control_status}    ✈️  상태: {flight_status}                 ║")
        
        # 속도 및 모드
        speed_names = {1:"매우느림", 2:"느림", 3:"보통", 4:"빠름", 5:"매우빠름"}
        speed_name = speed_names.get(self.current_speed_level, "보통")
        print(f"║ ⚡ 속도: Lv.{self.current_speed_level} ({speed_name:>6})                          ║")
        
        # 활성 키 표시
        active_keys_str = ", ".join(sorted(self.active_keys)) if self.active_keys else "없음"
        if len(active_keys_str) > 30:
            active_keys_str = active_keys_str[:27] + "..."
        print(f"║ ⌨️  활성키: {active_keys_str:<35} ║")
        
        # 안전 정보
        if self.emergency_stop:
            print("║ 🚨 긴급정지 활성화!                                           ║")
        elif home_dist > self.max_distance * 0.8:
            print("║ ⚠️  홈 거리 한계 접근 중                                       ║")
        elif self.current_position[2] > self.max_altitude * 0.8:
            print("║ ⚠️  최대 고도 접근 중                                          ║")
        else:
            print("║ ✅ 모든 시스템 정상                                           ║")
        
        print("╚════════════════════════════════════════════════════════════════╝")
        print()
        
        # 하단 명령어 힌트
        print("💡 명령어: SPACE(이륙) X(착륙) H(홈) T(제어) 1-5(속도) I(정보) ESC(종료)")
        print()

    def print_help(self):
        """도움말 출력"""
        print("\n🆘 도움말:")
        print("=" * 50)
        print("이동 제어:")
        print("  W/S       : 앞/뒤 이동")
        print("  A/D       : 왼쪽/오른쪽 이동")
        print("  R/F       : 상승/하강")
        print("  Q/E       : 좌회전/우회전")
        print("  방향키     : 세밀한 조정")
        print()
        print("비행 제어:")
        print("  SPACE     : 이륙")
        print("  X         : 착륙")
        print("  H         : 홈 복귀")
        print()
        print("시스템:")
        print("  T         : 제어 ON/OFF")
        print("  1-5       : 속도 조절")
        print("  I         : 정보 표시 ON/OFF")
        print("  ESC/Z     : 긴급 정지")
        print("=" * 50)

    def cleanup(self):
        """정리 작업"""
        self.running = False
        
        # 안전 착륙
        if self.is_flying:
            self.land()
            time.sleep(1)
        
        # 제어 비활성화
        self.enable_control(False)
        
        # 터미널 설정 복원
        try:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)
        except:
            pass
        
        print("\n👋 텔레오퍼레이션 시스템 종료")

def main():
    """메인 함수"""
    rclpy.init()
    
    try:
        controller = AdvancedTeleopController()
        
        print("🚁 고급 텔레오퍼레이션 시스템이 시작되었습니다!")
        print("제어를 시작하려면 'T' 키를 눌러주세요.")
        print("도움말을 보려면 '?' 키를 눌러주세요.")
        print()
        
        # ROS2 스핀
        while rclpy.ok() and controller.running:
            rclpy.spin_once(controller, timeout_sec=0.1)
            
            # 긴급 정지 체크
            if controller.emergency_stop:
                break
                
    except KeyboardInterrupt:
        print("\n사용자가 중단했습니다.")
    except Exception as e:
        print(f"\n오류 발생: {e}")
    finally:
        try:
            controller.cleanup()
        except:
            pass
        rclpy.shutdown()

if __name__ == '__main__':
    main() 