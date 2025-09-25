#!/usr/bin/env python3
# robot_driver_node.py
# TR200 로봇 구동 제어 노드 - 원격 제어 명령을 실제 로봇에 전송

import rospy
import asyncio
import sys
import os
import time
import threading
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, String

# SDK 경로 추가
sys.path.insert(0, '/catkin_ws/src/woosh_robot_py')

from woosh.proto.robot.robot_pack_pb2 import Twist as WooshTwist
from woosh_interface import CommuSettings, NO_PRINT
from woosh_robot import WooshRobot

class RobotDriverNode:
    """TR200 로봇 구동 제어 노드"""
    
    def __init__(self):
        # ROS 노드 초기화
        rospy.init_node('robot_driver_node', anonymous=True)
        
        # 파라미터 로드
        self.load_parameters()
        
        # ROS 인터페이스 설정
        self.setup_ros_interface()
        
        # 로봇 연결 상태
        self.robot = None
        self.connected = False
        
        # 제어 상태
        self.current_cmd = Twist()
        self.last_command_time = 0.0
        self.command_timeout = 2.0
        
        # 비동기 이벤트 루프
        self.loop = None
        self.loop_thread = None
        
        # 제어 스레드
        self.driver_thread = None
        self.monitoring_active = True
        
        rospy.loginfo("🚗 Robot Driver Node 초기화 완료")
    
    def load_parameters(self):
        """ROS 파라미터를 로드합니다."""
        # 로봇 연결 설정
        self.robot_ip = rospy.get_param('~robot_ip', '169.254.128.2')
        self.robot_port = rospy.get_param('~robot_port', 5480)
        self.robot_identity = rospy.get_param('~robot_identity', 'tr200_robot_driver')
        
        # 제어 설정
        self.control_frequency = rospy.get_param('~control_frequency', 20.0)
        self.max_linear_velocity = rospy.get_param('~max_linear_velocity', 1.0)
        self.max_angular_velocity = rospy.get_param('~max_angular_velocity', 1.0)
        
        rospy.loginfo(f"📋 파라미터 로드 완료: {self.robot_ip}:{self.robot_port}")
    
    def setup_ros_interface(self):
        """ROS 인터페이스를 설정합니다."""
        # 서브스크라이버
        self.remote_cmd_vel_sub = rospy.Subscriber(
            '/remote_cmd_vel', Twist, self.remote_cmd_vel_callback
        )
        self.external_cmd_vel_sub = rospy.Subscriber(
            '/external_cmd_vel', Twist, self.external_cmd_vel_callback
        )
        self.connection_status_sub = rospy.Subscriber(
            '/connection_status', Bool, self.connection_status_callback
        )
        self.emergency_stop_sub = rospy.Subscriber(
            '/emergency_stop', Bool, self.emergency_stop_callback
        )
        
        # 퍼블리셔
        self.driver_status_pub = rospy.Publisher(
            '/robot_driver_status', String, queue_size=10
        )
        
        # 연결 상태
        self.robot_connected = False
        self.emergency_stop_active = False
        self.connection_logged = False  # 연결 로그 중복 방지
        
        rospy.loginfo("📡 ROS 인터페이스 설정 완료")
    
    def remote_cmd_vel_callback(self, msg):
        """원격 제어 명령 콜백"""
        self.current_cmd = msg
        self.last_command_time = time.time()
        rospy.loginfo(f"🎮 원격 제어 명령 수신: linear={msg.linear.x:.2f}, angular={msg.angular.z:.2f}")
    
    def external_cmd_vel_callback(self, msg):
        """외부 제어 명령 콜백"""
        self.current_cmd = msg
        self.last_command_time = time.time()
        rospy.loginfo(f"🔗 외부 제어 명령 수신: linear={msg.linear.x:.2f}, angular={msg.angular.z:.2f}")
    
    def connection_status_callback(self, msg):
        """로봇 연결 상태 콜백"""
        self.robot_connected = msg.data
        
        if self.robot_connected:
            if not self.connection_logged:
                rospy.loginfo("✅ 로봇 연결됨 (Driver)")
                self.connection_logged = True
        else:
            self.connection_logged = False
            rospy.logwarn("⚠️ 로봇 연결 끊어짐 (Driver)")
            self.stop_robot()
    
    def emergency_stop_callback(self, msg):
        """비상 정지 콜백"""
        if msg.data:
            rospy.logwarn("🚨 비상 정지 신호 수신!")
            self.emergency_stop_active = True
            self.stop_robot()
        else:
            self.emergency_stop_active = False
    
    def start_async_loop(self):
        """비동기 이벤트 루프를 시작합니다."""
        def run_loop():
            self.loop = asyncio.new_event_loop()
            asyncio.set_event_loop(self.loop)
            self.loop.run_forever()
        
        self.loop_thread = threading.Thread(target=run_loop, daemon=True)
        self.loop_thread.start()
        
        # 루프가 시작될 때까지 잠시 대기
        time.sleep(0.1)
        
        rospy.loginfo("🔄 비동기 이벤트 루프 시작")
    
    def stop_async_loop(self):
        """비동기 이벤트 루프를 중지합니다."""
        if self.loop:
            self.loop.call_soon_threadsafe(self.loop.stop)
        if self.loop_thread:
            self.loop_thread.join(timeout=2.0)
        
        rospy.loginfo("⏹️ 비동기 이벤트 루프 중지")
    
    async def initialize_robot(self) -> bool:
        """로봇을 초기화하고 연결합니다."""
        try:
            rospy.loginfo("🔍 TR200 로봇 연결을 시도합니다...")
            
            # 로봇 설정
            settings = CommuSettings(
                addr=self.robot_ip,
                port=self.robot_port,
                identity=self.robot_identity
            )
            
            # 로봇 인스턴스 생성
            self.robot = WooshRobot(settings)
            
            # 연결 시도
            if not await self.robot.run():
                rospy.logerr("❌ 로봇 연결 실패")
                return False
            
            rospy.loginfo("✅ 로봇 연결 성공 (Driver)")
            self.connected = True
            
            return True
                
        except Exception as e:
            rospy.logerr(f"❌ 로봇 초기화 중 오류: {e}")
            return False
    
    def apply_safety_limits(self, cmd):
        """안전 제한을 적용합니다."""
        # 선속도 제한
        if abs(cmd.linear.x) > self.max_linear_velocity:
            cmd.linear.x = self.max_linear_velocity if cmd.linear.x > 0 else -self.max_linear_velocity
        
        # 각속도 제한
        if abs(cmd.angular.z) > self.max_angular_velocity:
            cmd.angular.z = self.max_angular_velocity if cmd.angular.z > 0 else -self.max_angular_velocity
        
        return cmd
    
    def stop_robot(self):
        """로봇을 정지시킵니다."""
        self.current_cmd = Twist()
        rospy.loginfo("⏹️ 로봇 정지 (Driver)")
    
    async def send_robot_command(self, cmd):
        """로봇에 명령을 전송합니다."""
        try:
            if self.robot and self.robot.comm.is_connected():
                # WooshTwist 메시지 생성
                woosh_twist = WooshTwist(
                    linear=cmd.linear.x,
                    angular=cmd.angular.z
                )
                
                # 명령 전송
                _, ok, msg = await self.robot.twist_req(woosh_twist, NO_PRINT, NO_PRINT)
                
                if ok:
                    rospy.logdebug(f"✅ 로봇 명령 전송 성공: linear={cmd.linear.x:.2f}, angular={cmd.angular.z:.2f}")
                else:
                    rospy.logwarn(f"⚠️ 로봇 명령 전송 실패: {msg}")
                
                return ok
            else:
                rospy.logwarn("⚠️ 로봇이 연결되지 않음")
                return False
                
        except Exception as e:
            rospy.logerr(f"❌ 로봇 명령 전송 중 오류: {e}")
            return False
    
    def driver_thread_func(self):
        """로봇 구동 제어 스레드"""
        rate = rospy.Rate(self.control_frequency)
        
        rospy.loginfo("🚗 로봇 구동 제어 스레드 시작")
        
        while not rospy.is_shutdown() and self.monitoring_active:
            try:
                current_time = time.time()
                
                # 명령 타임아웃 체크
                if current_time - self.last_command_time > self.command_timeout:
                    if self.current_cmd.linear.x != 0 or self.current_cmd.angular.z != 0:
                        rospy.logwarn("⏰ 명령 타임아웃 - 로봇 정지")
                        self.stop_robot()
                
                # 로봇 명령 전송
                if (self.robot_connected and not self.emergency_stop_active and 
                    (self.current_cmd.linear.x != 0 or self.current_cmd.angular.z != 0)):
                    
                    # 안전 제한 적용
                    safe_cmd = self.apply_safety_limits(self.current_cmd)
                    
                    # 비동기 명령 전송
                    if self.loop:
                        future = asyncio.run_coroutine_threadsafe(
                            self.send_robot_command(safe_cmd), self.loop
                        )
                        future.result(timeout=0.1)  # 짧은 타임아웃
                
                # 드라이버 상태 발행
                status_msg = f"Connected: {self.robot_connected}, Emergency: {self.emergency_stop_active}, Cmd: linear={self.current_cmd.linear.x:.2f}, angular={self.current_cmd.angular.z:.2f}"
                self.driver_status_pub.publish(String(data=status_msg))
                
            except Exception as e:
                rospy.logerr(f"❌ 로봇 구동 제어 중 오류: {e}")
            
            rate.sleep()
        
        rospy.loginfo("🚗 로봇 구동 제어 스레드 종료")
    
    def run(self):
        """노드를 실행합니다."""
        try:
            rospy.loginfo("🚀 Robot Driver Node 시작")
            
            # 비동기 이벤트 루프 시작
            self.start_async_loop()
            
            # 로봇 초기화
            if self.loop:
                future = asyncio.run_coroutine_threadsafe(
                    self.initialize_robot(), self.loop
                )
                if not future.result(timeout=10.0):
                    rospy.logerr("❌ 초기 로봇 연결 실패")
                    return
            
            # 구동 제어 스레드 시작
            self.driver_thread = threading.Thread(
                target=self.driver_thread_func, daemon=True
            )
            self.driver_thread.start()
            
            rospy.loginfo("✅ Robot Driver Node 실행 중...")
            
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
        
        # 로봇 연결 종료
        if self.robot and self.loop:
            try:
                future = asyncio.run_coroutine_threadsafe(
                    self.robot.stop(), self.loop
                )
                future.result(timeout=5.0)
                rospy.loginfo("✅ 로봇 연결 종료 (Driver)")
            except Exception as e:
                rospy.logwarn(f"⚠️ 로봇 연결 종료 중 오류: {e}")
        
        # 비동기 루프 중지
        self.stop_async_loop()
        
        rospy.loginfo("✅ 리소스 정리 완료")

def main():
    """메인 실행 함수"""
    try:
        node = RobotDriverNode()
        node.run()
    except Exception as e:
        rospy.logerr(f"❌ 프로그램 실행 중 오류: {e}")

if __name__ == "__main__":
    main()
