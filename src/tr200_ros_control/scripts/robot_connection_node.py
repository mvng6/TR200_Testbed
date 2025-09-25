#!/usr/bin/env python3
# robot_connection_node.py
# TR200 로봇과의 SDK 연결을 관리하는 ROS 노드

import rospy
import asyncio
import sys
import os
import time
import threading
from std_msgs.msg import Bool, String
from geometry_msgs.msg import Pose2D, Twist

# SDK 경로 추가
sys.path.insert(0, '/catkin_ws/src/woosh_robot_py')

from woosh.proto.robot.robot_pb2 import PoseSpeed, OperationState, ScannerData
from woosh.proto.robot.robot_pack_pb2 import Twist as WooshTwist
from woosh_interface import CommuSettings, NO_PRINT
from woosh_robot import WooshRobot

class RobotConnectionNode:
    """TR200 로봇 연결 관리 노드"""
    
    def __init__(self):
        # ROS 노드 초기화
        rospy.init_node('robot_connection_node', anonymous=True)
        
        # 파라미터 로드
        self.load_parameters()
        
        # ROS 퍼블리셔/서브스크라이버 설정
        self.setup_ros_interface()
        
        # 로봇 연결 상태
        self.robot = None
        self.connected = False
        self.connection_uptime = 0.0
        self.last_connection_time = 0.0
        self.connection_logged = False  # 연결 로그 중복 방지
        
        # 비동기 이벤트 루프
        self.loop = None
        self.loop_thread = None
        
        # 연결 상태 모니터링
        self.connection_monitor_thread = None
        self.monitoring_active = True
        
        # 로봇 상태 정보
        self.robot_status = {
            'robot_id': 'unknown',
            'robot_mode': 'unknown',
            'operation_state': 'unknown',
            'battery_level': 0.0,
            'battery_status': 'unknown',
            'pose': Pose2D(),
            'velocity': Twist()
        }
        
        rospy.loginfo("🤖 Robot Connection Node 초기화 완료")
    
    def load_parameters(self):
        """ROS 파라미터를 로드합니다."""
        # 로봇 연결 설정
        self.robot_ip = rospy.get_param('~robot_ip', '169.254.128.2')
        self.robot_port = rospy.get_param('~robot_port', 5480)
        self.robot_identity = rospy.get_param('~robot_identity', 'tr200_ros_controller')
        
        # 연결 설정
        self.connection_timeout = rospy.get_param('~connection_timeout', 10.0)
        self.reconnect_interval = rospy.get_param('~reconnect_interval', 2.0)
        self.max_reconnect_attempts = rospy.get_param('~max_reconnect_attempts', 5)
        
        # 모니터링 설정
        self.status_publish_frequency = rospy.get_param('~status_publish_frequency', 10.0)
        
        rospy.loginfo(f"📋 파라미터 로드 완료: {self.robot_ip}:{self.robot_port}")
    
    def setup_ros_interface(self):
        """ROS 인터페이스를 설정합니다."""
        # 퍼블리셔
        self.connection_status_pub = rospy.Publisher(
            '/connection_status', Bool, queue_size=10
        )
        self.robot_status_pub = rospy.Publisher(
            '/robot_status', String, queue_size=10
        )
        
        # 서비스 (향후 구현)
        # self.set_robot_pose_srv = rospy.Service(
        #     '/set_robot_pose', SetRobotPose, self.set_robot_pose_callback
        # )
        
        rospy.loginfo("📡 ROS 인터페이스 설정 완료")
    
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
            
            rospy.loginfo("✅ 로봇 연결 성공")
            self.connected = True
            self.last_connection_time = time.time()
            
            # 로봇 상태 구독 설정
            await self.setup_robot_subscriptions()
            
            return True
                
        except Exception as e:
            rospy.logerr(f"❌ 로봇 초기화 중 오류: {e}")
            return False
    
    async def setup_robot_subscriptions(self):
        """로봇 상태 구독을 설정합니다."""
        try:
            # 위치 및 속도 구독
            await self.robot.robot_pose_speed_sub(self.pose_speed_callback, NO_PRINT)
            
            # 배터리 상태 구독
            await self.robot.robot_battery_sub(self.battery_callback, NO_PRINT)
            
            # 작업 상태 구독
            await self.robot.robot_operation_state_sub(self.operation_state_callback, NO_PRINT)
            
            rospy.loginfo("📡 로봇 상태 구독 설정 완료")
            
        except Exception as e:
            rospy.logerr(f"❌ 로봇 구독 설정 중 오류: {e}")
    
    def pose_speed_callback(self, pose_speed: PoseSpeed):
        """위치 및 속도 콜백 함수"""
        try:
            # 위치 정보 업데이트
            self.robot_status['pose'].x = pose_speed.pose.x
            self.robot_status['pose'].y = pose_speed.pose.y
            self.robot_status['pose'].theta = pose_speed.pose.theta
            
            # 속도 정보 업데이트
            self.robot_status['velocity'].linear.x = pose_speed.twist.linear
            self.robot_status['velocity'].angular.z = pose_speed.twist.angular
            
        except Exception as e:
            rospy.logwarn(f"⚠️ 위치/속도 콜백 처리 중 오류: {e}")
    
    def battery_callback(self, battery):
        """배터리 상태 콜백 함수"""
        try:
            self.robot_status['battery_level'] = battery.power
            self.robot_status['battery_status'] = battery.charge_state
            
        except Exception as e:
            rospy.logwarn(f"⚠️ 배터리 콜백 처리 중 오류: {e}")
    
    def operation_state_callback(self, operation_state: OperationState):
        """작업 상태 콜백 함수"""
        try:
            # 작업 상태를 문자열로 변환
            if operation_state.robot & OperationState.RobotBit.kTaskable:
                self.robot_status['operation_state'] = 'TASKABLE'
            elif operation_state.robot & OperationState.RobotBit.kCharging:
                self.robot_status['operation_state'] = 'CHARGING'
            elif operation_state.robot & OperationState.RobotBit.kEmergency:
                self.robot_status['operation_state'] = 'EMERGENCY'
            else:
                self.robot_status['operation_state'] = 'UNKNOWN'
                
        except Exception as e:
            rospy.logwarn(f"⚠️ 작업 상태 콜백 처리 중 오류: {e}")
    
    def connection_monitor(self):
        """연결 상태를 모니터링합니다."""
        rate = rospy.Rate(self.status_publish_frequency)
        last_status_msg_time = 0
        status_msg_interval = 10.0  # 10초마다 상태 메시지 출력
        
        while not rospy.is_shutdown() and self.monitoring_active:
            try:
                # 연결 상태 확인
                current_time = time.time()
                is_connected = self.robot and self.robot.comm.is_connected()
                
                if is_connected:
                    if not self.connected:
                        self.connected = True
                        self.last_connection_time = current_time
                        if not self.connection_logged:
                            rospy.loginfo("✅ 로봇 연결됨")
                            self.connection_logged = True
                    
                    # 연결 지속 시간 계산
                    self.connection_uptime = current_time - self.last_connection_time
                    
                else:
                    if self.connected:
                        self.connected = False
                        self.connection_logged = False
                        rospy.logwarn("⚠️ 로봇 연결 끊어짐")
                
                # 연결 상태 발행
                self.connection_status_pub.publish(Bool(data=self.connected))
                
                # 로봇 상태 정보 발행 (주기적으로만)
                if current_time - last_status_msg_time > status_msg_interval:
                    status_msg = f"Connected: {self.connected}, Uptime: {self.connection_uptime:.1f}s"
                    self.robot_status_pub.publish(String(data=status_msg))
                    last_status_msg_time = current_time
                
            except Exception as e:
                rospy.logerr(f"❌ 연결 모니터링 중 오류: {e}")
            
            rate.sleep()
    
    async def reconnect_robot(self):
        """로봇 재연결을 시도합니다."""
        reconnect_count = 0
        
        while reconnect_count < self.max_reconnect_attempts and not rospy.is_shutdown():
            try:
                rospy.loginfo(f"🔄 로봇 재연결 시도 {reconnect_count + 1}/{self.max_reconnect_attempts}")
                
                if await self.initialize_robot():
                    rospy.loginfo("✅ 로봇 재연결 성공")
                    return True
                
                reconnect_count += 1
                await asyncio.sleep(self.reconnect_interval)
                
            except Exception as e:
                rospy.logerr(f"❌ 재연결 시도 중 오류: {e}")
                reconnect_count += 1
                await asyncio.sleep(self.reconnect_interval)
        
        rospy.logerr("❌ 최대 재연결 시도 횟수 초과")
        return False
    
    def run(self):
        """노드를 실행합니다."""
        try:
            rospy.loginfo("🚀 Robot Connection Node 시작")
            
            # 비동기 이벤트 루프 시작
            self.start_async_loop()
            
            # 로봇 초기화
            if self.loop:
                future = asyncio.run_coroutine_threadsafe(
                    self.initialize_robot(), self.loop
                )
                if not future.result(timeout=self.connection_timeout):
                    rospy.logerr("❌ 초기 로봇 연결 실패")
                    return
            
            # 연결 모니터링 스레드 시작
            self.connection_monitor_thread = threading.Thread(
                target=self.connection_monitor, daemon=True
            )
            self.connection_monitor_thread.start()
            
            rospy.loginfo("✅ Robot Connection Node 실행 중...")
            
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
        
        # 로봇 연결 종료
        if self.robot and self.loop:
            try:
                future = asyncio.run_coroutine_threadsafe(
                    self.robot.stop(), self.loop
                )
                future.result(timeout=5.0)
                rospy.loginfo("✅ 로봇 연결 종료")
            except Exception as e:
                rospy.logwarn(f"⚠️ 로봇 연결 종료 중 오류: {e}")
        
        # 비동기 루프 중지
        self.stop_async_loop()
        
        rospy.loginfo("✅ 리소스 정리 완료")

def main():
    """메인 실행 함수"""
    try:
        node = RobotConnectionNode()
        node.run()
    except Exception as e:
        rospy.logerr(f"❌ 프로그램 실행 중 오류: {e}")

if __name__ == "__main__":
    main()
