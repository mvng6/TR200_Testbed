#!/usr/bin/env python3
# remote_control_node.py
# 노트북에서 키보드 입력을 통한 TR200 로봇 원격 제어 노드

import rospy
import sys
import termios
import tty
import select
import threading
import time
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, String

class RemoteControlNode:
    """TR200 로봇 원격 제어 노드"""
    
    def __init__(self):
        # ROS 노드 초기화
        rospy.init_node('remote_control_node', anonymous=True)
        
        # 파라미터 로드
        self.load_parameters()
        
        # ROS 인터페이스 설정
        self.setup_ros_interface()
        
        # 제어 상태
        self.control_active = False
        self.current_cmd = Twist()
        self.last_command_time = 0.0
        
        # 속도 레벨 (개발용 안전 설정)
        self.speed_levels = {
            1: {'linear': 0.02, 'angular': 0.05},  # 매우 느림 (개발용)
            2: {'linear': 0.05, 'angular': 0.1},   # 느림
            3: {'linear': 0.1, 'angular': 0.2},    # 보통
        }
        self.current_speed_level = 1  # 기본값: 매우 느림
        
        # 키보드 입력 설정
        self.setup_keyboard()
        
        # 제어 스레드
        self.control_thread = None
        self.monitoring_active = True
        
        rospy.loginfo("🎮 Remote Control Node 초기화 완료")
    
    def load_parameters(self):
        """ROS 파라미터를 로드합니다."""
        # 속도 설정
        self.linear_speed = rospy.get_param('~linear_speed', 0.5)  # m/s
        self.angular_speed = rospy.get_param('~angular_speed', 0.5)  # rad/s
        
        # 제어 설정
        self.command_timeout = rospy.get_param('~command_timeout', 2.0)  # 초
        self.publish_frequency = rospy.get_param('~publish_frequency', 20.0)  # Hz
        
        # 안전 설정
        self.enable_safety_limits = rospy.get_param('~enable_safety_limits', True)
        self.max_linear_speed = rospy.get_param('~max_linear_speed', 1.0)  # m/s
        self.max_angular_speed = rospy.get_param('~max_angular_speed', 1.0)  # rad/s
        
        rospy.loginfo(f"📋 파라미터 로드 완료: 선속도={self.linear_speed}m/s, 각속도={self.angular_speed}rad/s")
    
    def setup_ros_interface(self):
        """ROS 인터페이스를 설정합니다."""
        # 퍼블리셔
        self.remote_cmd_vel_pub = rospy.Publisher(
            '/remote_cmd_vel', Twist, queue_size=10
        )
        self.control_status_pub = rospy.Publisher(
            '/remote_control_status', String, queue_size=10
        )
        
        # 서브스크라이버
        self.connection_status_sub = rospy.Subscriber(
            '/connection_status', Bool, self.connection_status_callback
        )
        self.emergency_stop_sub = rospy.Subscriber(
            '/emergency_stop', Bool, self.emergency_stop_callback
        )
        
        # 연결 상태
        self.robot_connected = False
        self.emergency_stop_active = False
        self.connection_logged = False  # 연결 로그 중복 방지
        
        rospy.loginfo("📡 ROS 인터페이스 설정 완료")
    
    def setup_keyboard(self):
        """키보드 입력을 설정합니다."""
        # 터미널 설정 저장
        self.old_settings = termios.tcgetattr(sys.stdin)
        
        # 키보드 매핑
        self.key_mapping = {
            'w': 'forward',      # 전진
            's': 'backward',     # 후진
            'a': 'left',         # 좌회전
            'd': 'right',        # 우회전
            'q': 'turn_left',    # 제자리 좌회전
            'e': 'turn_right',   # 제자리 우회전
            ' ': 'stop',         # 정지
            'x': 'emergency',    # 비상정지
            'r': 'reset',        # 리셋
            'h': 'help',         # 도움말
            'c': 'connect',      # 연결 상태 확인
            '+': 'speed_up',     # 속도 증가
            '-': 'speed_down',   # 속도 감소
            '1': 'speed_1',      # 속도 레벨 1 (매우 느림)
            '2': 'speed_2',      # 속도 레벨 2 (느림)
            '3': 'speed_3',      # 속도 레벨 3 (보통)
        }
        
        rospy.loginfo("⌨️ 키보드 입력 설정 완료")
    
    def connection_status_callback(self, msg):
        """로봇 연결 상태 콜백"""
        self.robot_connected = msg.data
        
        if self.robot_connected:
            if not self.connection_logged:
                rospy.loginfo("✅ 로봇 연결됨")
                self.connection_logged = True
        else:
            self.connection_logged = False
            rospy.logwarn("⚠️ 로봇 연결 끊어짐")
            self.stop_robot()
    
    def emergency_stop_callback(self, msg):
        """비상 정지 콜백"""
        if msg.data:
            rospy.logwarn("🚨 비상 정지 신호 수신!")
            self.emergency_stop_active = True
            self.stop_robot()
        else:
            self.emergency_stop_active = False
    
    def print_help(self):
        """도움말을 출력합니다."""
        current_linear = self.speed_levels[self.current_speed_level]['linear']
        current_angular = self.speed_levels[self.current_speed_level]['angular']
        
        help_text = f"""
🎮 TR200 원격 제어 도움말 (개발용 안전 모드)
==========================================
현재 속도 레벨: {self.current_speed_level}
- 선속도: {current_linear:.3f} m/s
- 각속도: {current_angular:.3f} rad/s

이동 제어:
  w - 전진
  s - 후진
  a - 좌회전
  d - 우회전
  q - 제자리 좌회전
  e - 제자리 우회전
  (스페이스) - 정지

속도 조절:
  + - 속도 증가
  - - 속도 감소
  1 - 속도 레벨 1 (매우 느림: 0.02m/s, 0.05rad/s)
  2 - 속도 레벨 2 (느림: 0.05m/s, 0.1rad/s)
  3 - 속도 레벨 3 (보통: 0.1m/s, 0.2rad/s)

특수 명령:
  x - 비상정지
  r - 리셋
  h - 이 도움말 표시
  c - 연결 상태 확인

종료: Ctrl+C
==========================================
        """
        print(help_text)
    
    def print_status(self):
        """현재 상태를 출력합니다."""
        status_text = f"""
📊 현재 상태
============
로봇 연결: {'✅ 연결됨' if self.robot_connected else '❌ 연결 끊어짐'}
비상정지: {'🚨 활성화' if self.emergency_stop_active else '✅ 비활성화'}
제어 활성: {'✅ 활성화' if self.control_active else '❌ 비활성화'}
현재 명령: 선속도={self.current_cmd.linear.x:.2f}m/s, 각속도={self.current_cmd.angular.z:.2f}rad/s
============
        """
        print(status_text)
    
    def stop_robot(self):
        """로봇을 정지시킵니다."""
        self.current_cmd = Twist()
        self.control_active = False
        rospy.loginfo("⏹️ 로봇 정지")
    
    def reset_robot(self):
        """로봇 상태를 리셋합니다."""
        self.stop_robot()
        self.emergency_stop_active = False
        rospy.loginfo("🔄 로봇 상태 리셋")
    
    def apply_safety_limits(self, cmd):
        """안전 제한을 적용합니다."""
        if not self.enable_safety_limits:
            return cmd
        
        # 선속도 제한
        if abs(cmd.linear.x) > self.max_linear_speed:
            cmd.linear.x = self.max_linear_speed if cmd.linear.x > 0 else -self.max_linear_speed
        
        # 각속도 제한
        if abs(cmd.angular.z) > self.max_angular_speed:
            cmd.angular.z = self.max_angular_speed if cmd.angular.z > 0 else -self.max_angular_speed
        
        return cmd
    
    def process_key_input(self, key):
        """키 입력을 처리합니다."""
        rospy.loginfo(f"🔑 키 입력 감지: '{key}'")
        
        if key not in self.key_mapping:
            rospy.logwarn(f"⚠️ 알 수 없는 키: '{key}'")
            return
        
        action = self.key_mapping[key]
        rospy.loginfo(f"🎯 액션 실행: {action}")
        
        # 로봇이 연결되지 않은 경우 제어 불가
        if not self.robot_connected and action not in ['h', 'c', 'r']:
            rospy.logwarn("⚠️ 로봇이 연결되지 않았습니다. 먼저 로봇을 연결하세요.")
            return
        
        # 비상정지가 활성화된 경우 제어 불가
        if self.emergency_stop_active and action not in ['h', 'c', 'r', 'x']:
            rospy.logwarn("🚨 비상정지가 활성화되어 있습니다. 먼저 비상정지를 해제하세요.")
            return
        
        # 명령 처리 (현재 속도 레벨 사용)
        current_linear = self.speed_levels[self.current_speed_level]['linear']
        current_angular = self.speed_levels[self.current_speed_level]['angular']
        
        if action == 'forward':
            self.current_cmd.linear.x = current_linear
            self.current_cmd.angular.z = 0.0
            self.control_active = True
            rospy.loginfo(f"⬆️ 전진 (속도 레벨 {self.current_speed_level}: {current_linear:.3f}m/s)")
            
        elif action == 'backward':
            self.current_cmd.linear.x = -current_linear
            self.current_cmd.angular.z = 0.0
            self.control_active = True
            rospy.loginfo(f"⬇️ 후진 (속도 레벨 {self.current_speed_level}: {current_linear:.3f}m/s)")
            
        elif action == 'left':
            self.current_cmd.linear.x = current_linear * 0.5
            self.current_cmd.angular.z = current_angular
            self.control_active = True
            rospy.loginfo(f"↖️ 좌회전 (속도 레벨 {self.current_speed_level})")
            
        elif action == 'right':
            self.current_cmd.linear.x = current_linear * 0.5
            self.current_cmd.angular.z = -current_angular
            self.control_active = True
            rospy.loginfo(f"↗️ 우회전 (속도 레벨 {self.current_speed_level})")
            
        elif action == 'turn_left':
            self.current_cmd.linear.x = 0.0
            self.current_cmd.angular.z = current_angular
            self.control_active = True
            rospy.loginfo(f"↻ 제자리 좌회전 (속도 레벨 {self.current_speed_level}: {current_angular:.3f}rad/s)")
            
        elif action == 'turn_right':
            self.current_cmd.linear.x = 0.0
            self.current_cmd.angular.z = -current_angular
            self.control_active = True
            rospy.loginfo(f"↺ 제자리 우회전 (속도 레벨 {self.current_speed_level}: {current_angular:.3f}rad/s)")
            
        elif action == 'stop':
            self.stop_robot()
            
        elif action == 'emergency':
            rospy.logwarn("🚨 비상정지!")
            self.stop_robot()
            
        elif action == 'reset':
            self.reset_robot()
            
        elif action == 'help':
            self.print_help()
            
        elif action == 'connect':
            self.print_status()
            
        elif action == 'speed_up':
            if self.current_speed_level < 3:
                self.current_speed_level += 1
                rospy.loginfo(f"🚀 속도 레벨 증가: {self.current_speed_level}")
            else:
                rospy.logwarn("⚠️ 최대 속도 레벨에 도달했습니다.")
                
        elif action == 'speed_down':
            if self.current_speed_level > 1:
                self.current_speed_level -= 1
                rospy.loginfo(f"🐌 속도 레벨 감소: {self.current_speed_level}")
            else:
                rospy.logwarn("⚠️ 최소 속도 레벨에 도달했습니다.")
                
        elif action == 'speed_1':
            self.current_speed_level = 1
            rospy.loginfo("🐌 속도 레벨 1 (매우 느림)")
            
        elif action == 'speed_2':
            self.current_speed_level = 2
            rospy.loginfo("🚶 속도 레벨 2 (느림)")
            
        elif action == 'speed_3':
            self.current_speed_level = 3
            rospy.loginfo("🏃 속도 레벨 3 (보통)")
        
        # 안전 제한 적용
        self.current_cmd = self.apply_safety_limits(self.current_cmd)
        
        # 명령 시간 업데이트
        self.last_command_time = time.time()
    
    def keyboard_input_thread(self):
        """키보드 입력을 처리하는 스레드"""
        rospy.loginfo("⌨️ 키보드 입력 스레드 시작")
        
        try:
            while not rospy.is_shutdown() and self.monitoring_active:
                # 키 입력 대기 (더 안정적인 방법)
                try:
                    if select.select([sys.stdin], [], [], 0.1)[0]:
                        key = sys.stdin.read(1)
                        
                        # Ctrl+C 처리
                        if ord(key) == 3:  # Ctrl+C
                            break
                        
                        # 키를 소문자로 변환
                        key = key.lower()
                        
                        # 키 입력 처리
                        rospy.loginfo(f"🔍 키보드 입력 감지: '{key}' (ord: {ord(key)})")
                        self.process_key_input(key)
                        
                except Exception as e:
                    rospy.logwarn(f"⚠️ 키보드 입력 처리 중 오류: {e}")
                    continue
                
                # 명령 타임아웃 체크
                if (self.control_active and 
                    time.time() - self.last_command_time > self.command_timeout):
                    rospy.logwarn("⏰ 명령 타임아웃 - 로봇 정지")
                    self.stop_robot()
                
        except Exception as e:
            rospy.logerr(f"❌ 키보드 입력 처리 중 오류: {e}")
        finally:
            rospy.loginfo("⌨️ 키보드 입력 스레드 종료")
    
    def control_publisher_thread(self):
        """제어 명령을 발행하는 스레드"""
        rate = rospy.Rate(self.publish_frequency)
        
        rospy.loginfo("📡 제어 명령 발행 스레드 시작")
        
        while not rospy.is_shutdown() and self.monitoring_active:
            try:
                # 제어 명령 발행
                if self.control_active and self.robot_connected and not self.emergency_stop_active:
                    self.remote_cmd_vel_pub.publish(self.current_cmd)
                
                # 제어 상태 발행
                status_msg = f"Active: {self.control_active}, Connected: {self.robot_connected}, Emergency: {self.emergency_stop_active}"
                self.control_status_pub.publish(String(data=status_msg))
                
            except Exception as e:
                rospy.logerr(f"❌ 제어 명령 발행 중 오류: {e}")
            
            rate.sleep()
        
        rospy.loginfo("📡 제어 명령 발행 스레드 종료")
    
    def run(self):
        """노드를 실행합니다."""
        try:
            rospy.loginfo("🚀 Remote Control Node 시작")
            
            # 도움말 출력
            self.print_help()
            
            # 키보드 입력 스레드 시작
            self.control_thread = threading.Thread(
                target=self.keyboard_input_thread, daemon=True
            )
            self.control_thread.start()
            
            # 제어 명령 발행 스레드 시작
            publisher_thread = threading.Thread(
                target=self.control_publisher_thread, daemon=True
            )
            publisher_thread.start()
            
            rospy.loginfo("✅ Remote Control Node 실행 중...")
            rospy.loginfo("💡 키보드 입력을 시작하려면 터미널을 클릭하세요.")
            
            # ROS 스핀
            rospy.spin()
            
        except KeyboardInterrupt:
            rospy.loginfo("⏹️ 사용자 요청으로 노드 중지")
        except Exception as e:
            rospy.logerr(f"❌ 노드 실행 중 오류: {e}")
        finally:
            self.cleanup()
    
    def cleanup(self):
        """리소스를 정리합니다."""
        rospy.loginfo("🧹 리소스 정리 중...")
        
        # 모니터링 중지
        self.monitoring_active = False
        
        # 로봇 정지
        self.stop_robot()
        
        # 키보드 설정 복원
        try:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)
            rospy.loginfo("✅ 키보드 설정 복원")
        except Exception as e:
            rospy.logwarn(f"⚠️ 키보드 설정 복원 중 오류: {e}")
        
        rospy.loginfo("✅ 리소스 정리 완료")

def main():
    """메인 실행 함수"""
    try:
        node = RemoteControlNode()
        node.run()
    except Exception as e:
        rospy.logerr(f"❌ 프로그램 실행 중 오류: {e}")

if __name__ == "__main__":
    main()
