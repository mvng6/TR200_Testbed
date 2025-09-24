#!/usr/bin/env python3
# robot_driver_node.py
# TR200 로봇 제어 전용 노드

import rospy
import asyncio
import threading
import time
from typing import Optional

# ROS 메시지 타입
from geometry_msgs.msg import Twist
from std_msgs.msg import String

# SDK 경로 추가
import sys
sys.path.insert(0, '/catkin_ws/src/woosh_robot_py')

from woosh.proto.robot.robot_pack_pb2 import Twist as WooshTwist
from woosh_interface import CommuSettings, NO_PRINT
from woosh_robot import WooshRobot

class RobotDriverNode:
    """TR200 로봇 제어 전용 노드"""
    
    def __init__(self):
        """로봇 제어 노드 초기화"""
        rospy.init_node('robot_driver', anonymous=True)
        
        # ROS 파라미터 로드
        self.load_parameters()
        
        # Woosh SDK 초기화
        self.setup_woosh_sdk()
        
        # ROS 서브스크라이버 설정
        self.setup_subscribers()
        
        # ROS 퍼블리셔 설정
        self.setup_publishers()
        
        # 상태 변수 초기화
        self.initialize_variables()
        
        rospy.loginfo("로봇 제어 노드가 시작되었습니다")
    
    def load_parameters(self):
        """파라미터 로드"""
        self.robot_ip = rospy.get_param('~robot_ip', '169.254.128.2')
        self.robot_port = rospy.get_param('~robot_port', 5480)
        self.safe_cmd_vel_topic = rospy.get_param('~safe_cmd_vel_topic', '/safe_cmd_vel')
        self.robot_status_topic = rospy.get_param('~robot_status_topic', '/robot_status')
        
        rospy.loginfo(f"로봇 제어 파라미터: {self.robot_ip}:{self.robot_port}")
    
    def setup_woosh_sdk(self):
        """Woosh SDK 초기화"""
        try:
            self.settings = CommuSettings(
                addr=self.robot_ip,
                port=self.robot_port,
                identity='robot-driver'
            )
            self.robot = WooshRobot(self.settings)
            self.robot_connected = False
            
            # 별도 스레드에서 로봇 연결
            self.connect_thread = threading.Thread(target=self.connect_robot_sync)
            self.connect_thread.daemon = True
            self.connect_thread.start()
            
        except Exception as e:
            rospy.logerr(f"Woosh SDK 초기화 실패: {e}")
            self.robot = None
    
    def setup_subscribers(self):
        """ROS 서브스크라이버 설정"""
        # 안전한 속도 명령 수신
        self.safe_cmd_vel_sub = rospy.Subscriber(
            self.safe_cmd_vel_topic, 
            Twist, 
            self.safe_cmd_vel_callback
        )
        
        rospy.loginfo("로봇 제어 서브스크라이버 설정 완료")
    
    def setup_publishers(self):
        """ROS 퍼블리셔 설정"""
        # 로봇 상태 퍼블리시
        self.robot_status_pub = rospy.Publisher(self.robot_status_topic, String, queue_size=10)
        
        rospy.loginfo("로봇 제어 퍼블리셔 설정 완료")
    
    def initialize_variables(self):
        """상태 변수 초기화"""
        self.current_speed = 0.0
        self.current_direction = "stop"
        self.last_command_time = 0
        self.command_timeout = 1.0  # 1초 타임아웃
        self.cycle_count = 0
        
        # 통계
        self.command_count = 0
        self.error_count = 0
    
    def connect_robot_sync(self):
        """동기적으로 로봇 연결"""
        try:
            loop = asyncio.new_event_loop()
            asyncio.set_event_loop(loop)
            
            connected = loop.run_until_complete(self.robot.run())
            
            if connected:
                self.robot_connected = True
                rospy.loginfo("TR200 로봇 연결 성공")
            else:
                rospy.logerr("TR200 로봇 연결 실패")
                
        except Exception as e:
            rospy.logerr(f"로봇 연결 오류: {e}")
    
    def safe_cmd_vel_callback(self, msg: Twist):
        """안전한 속도 명령 콜백"""
        self.last_command_time = time.time()
        
        # 방향 결정 (간단한 로직)
        if msg.linear.x > 0:
            self.current_direction = "forward"
        elif msg.linear.x < 0:
            self.current_direction = "backward"
        else:
            self.current_direction = "stop"
        
        # 로봇에 명령 전송
        self.send_velocity_command(msg.linear.x)
        
        rospy.logdebug(f"속도 명령 수신: linear={msg.linear.x:.3f}, direction={self.current_direction}")
    
    def send_velocity_command(self, speed: float):
        """로봇에 속도 명령 전송"""
        if not self.robot_connected or not self.robot:
            rospy.logwarn("로봇이 연결되지 않음")
            return
        
        try:
            loop = asyncio.new_event_loop()
            asyncio.set_event_loop(loop)
            
            twist_cmd = WooshTwist(linear=speed, angular=0.0)
            _, ok, msg = loop.run_until_complete(
                self.robot.twist_req(twist_cmd, NO_PRINT, NO_PRINT)
            )
            
            if ok:
                self.command_count += 1
                self.current_speed = speed
                rospy.logdebug(f"속도 명령 전송 성공: {speed:.3f}m/s")
            else:
                self.error_count += 1
                rospy.logwarn(f"속도 명령 전송 실패: {msg}")
                
        except Exception as e:
            self.error_count += 1
            rospy.logwarn(f"속도 명령 전송 오류: {e}")
    
    def emergency_stop(self):
        """비상 정지 수행"""
        rospy.logwarn("🚨 비상 정지 실행!")
        
        # 즉시 정지 명령 전송
        for i in range(10):
            self.send_velocity_command(0.0)
            time.sleep(0.05)
        
        rospy.loginfo("✅ 비상 정지 완료")
    
    def start_status_monitoring(self):
        """상태 모니터링 시작"""
        self.status_timer = rospy.Timer(rospy.Duration(1.0), self.status_monitoring_callback)
    
    def status_monitoring_callback(self, event):
        """상태 모니터링 콜백"""
        try:
            self.cycle_count += 1
            
            # 명령 타임아웃 체크
            if time.time() - self.last_command_time > self.command_timeout:
                if self.current_speed != 0.0:
                    rospy.logwarn("명령 타임아웃 - 자동 정지")
                    self.send_velocity_command(0.0)
                    self.current_speed = 0.0
                    self.current_direction = "stop"
            
            # 로봇 상태 메시지 생성
            status_data = f"속도:{self.current_speed:.3f}m/s, 방향:{self.current_direction}, 연결:{self.robot_connected}, 명령:{self.command_count}, 오류:{self.error_count}"
            
            # 로봇 상태 퍼블리시
            status_msg = String()
            status_msg.data = status_data
            self.robot_status_pub.publish(status_msg)
            
            # 주기적으로 상태 로그 출력
            if self.cycle_count % 10 == 0:  # 10초마다
                rospy.loginfo(f"로봇 상태: {status_data}")
            
        except Exception as e:
            rospy.logerr(f"상태 모니터링 오류: {e}")
    
    def cleanup(self):
        """리소스 정리"""
        rospy.loginfo("로봇 제어 노드 정리 중...")
        
        # 최종 정지 명령
        self.send_velocity_command(0.0)
        
        if self.robot and self.robot_connected:
            try:
                loop = asyncio.new_event_loop()
                asyncio.set_event_loop(loop)
                loop.run_until_complete(self.robot.stop())
                rospy.loginfo("로봇 연결 종료")
            except Exception as e:
                rospy.logwarn(f"로봇 연결 종료 오류: {e}")
        
        rospy.loginfo("로봇 제어 노드 정리 완료")

def main():
    """메인 함수"""
    try:
        driver = RobotDriverNode()
        
        # 상태 모니터링 시작
        driver.start_status_monitoring()
        
        # ROS 스핀
        rospy.spin()
        
    except rospy.ROSInterruptException:
        rospy.loginfo("ROS 인터럽트로 프로그램 종료")
    except Exception as e:
        rospy.logerr(f"프로그램 실행 오류: {e}")
    finally:
        if 'driver' in locals():
            driver.cleanup()

if __name__ == "__main__":
    main()
