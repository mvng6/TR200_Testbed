#!/usr/bin/env python3
# tr200_ros_sensor_safety_controller.py
# TR200 ROS 통합 센서 기반 안전 제어 시스템

import rospy
import asyncio
import threading
import time
import math
from datetime import datetime
from typing import Optional, Tuple

# ROS 메시지 타입
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float32, String
from std_srvs.srv import SetBool, SetBoolResponse
from tr200_simple_control.srv import SetSafetyParams, SetSafetyParamsResponse

# SDK 경로 추가
import sys
sys.path.insert(0, '/catkin_ws/src/woosh_robot_py')

from woosh.proto.robot.robot_pb2 import ScannerData
from woosh.proto.robot.robot_pack_pb2 import Twist as WooshTwist
from woosh_interface import CommuSettings, NO_PRINT
from woosh_robot import WooshRobot

class TR200ROSSensorSafetyController:
    """TR200 ROS 통합 센서 기반 안전 제어 시스템"""
    
    def __init__(self):
        """ROS 노드 초기화"""
        rospy.init_node('tr200_sensor_safety_controller', anonymous=True)
        
        # ROS 파라미터 로드
        self.load_ros_parameters()
        
        # Woosh SDK 초기화
        self.setup_woosh_sdk()
        
        # ROS 퍼블리셔 설정
        self.setup_ros_publishers()
        
        # ROS 서브스크라이버 설정
        self.setup_ros_subscribers()
        
        # ROS 서비스 설정
        self.setup_ros_services()
        
        # 상태 변수 초기화
        self.initialize_state_variables()
        
        # 비동기 작업을 위한 스레드 시작
        self.start_async_threads()
        
        rospy.loginfo("TR200 ROS 센서 안전 제어 시스템이 시작되었습니다")
    
    def load_ros_parameters(self):
        """ROS 파라미터 로드"""
        # 로봇 연결 설정
        self.robot_ip = rospy.get_param('~robot_ip', '169.254.128.2')
        self.robot_port = rospy.get_param('~robot_port', 5480)
        
        # 센서 설정
        self.min_obstacle_distance = rospy.get_param('~min_obstacle_distance', 0.5)
        self.warning_distance = rospy.get_param('~warning_distance', 0.8)
        self.safe_distance = rospy.get_param('~safe_distance', 1.0)
        
        # 제어 설정
        self.normal_speed = rospy.get_param('~normal_speed', 0.2)
        self.slow_speed = rospy.get_param('~slow_speed', 0.1)
        self.stop_speed = rospy.get_param('~stop_speed', 0.0)
        self.control_frequency = rospy.get_param('~control_frequency', 20.0)
        
        # 토픽 설정
        self.cmd_vel_topic = rospy.get_param('~cmd_vel_topic', '/cmd_vel')
        self.scan_topic = rospy.get_param('~scan_topic', '/scan')
        self.safety_status_topic = rospy.get_param('~safety_status_topic', '/safety_status')
        
        rospy.loginfo(f"ROS 파라미터 로드 완료:")
        rospy.loginfo(f"  로봇 IP: {self.robot_ip}:{self.robot_port}")
        rospy.loginfo(f"  안전 거리: {self.min_obstacle_distance}m / {self.warning_distance}m / {self.safe_distance}m")
        rospy.loginfo(f"  속도 설정: {self.normal_speed}m/s / {self.slow_speed}m/s / {self.stop_speed}m/s")
    
    def setup_woosh_sdk(self):
        """Woosh SDK 초기화"""
        try:
            self.settings = CommuSettings(
                addr=self.robot_ip,
                port=self.robot_port,
                identity='ros-sensor-safety-controller'
            )
            self.robot = WooshRobot(self.settings)
            self.robot_connected = False
            
            # 별도 스레드에서 로봇 연결
            self.connect_thread = threading.Thread(target=self.connect_robot_sync)
            self.connect_thread.daemon = True
            self.connect_thread.start()
            
            rospy.loginfo("Woosh SDK 초기화 완료")
            
        except Exception as e:
            rospy.logerr(f"Woosh SDK 초기화 실패: {e}")
            self.robot = None
    
    def setup_ros_publishers(self):
        """ROS 퍼블리셔 설정"""
        # 안전 상태 퍼블리시
        self.safety_status_pub = rospy.Publisher(
            self.safety_status_topic, 
            String, 
            queue_size=10
        )
        
        # 장애물 거리 퍼블리시
        self.obstacle_distance_pub = rospy.Publisher(
            '/obstacle_distance', 
            Float32, 
            queue_size=10
        )
        
        # 비상 정지 상태 퍼블리시
        self.emergency_stop_pub = rospy.Publisher(
            '/emergency_stop', 
            Bool, 
            queue_size=10
        )
        
        # 로봇 상태 퍼블리시
        self.robot_state_pub = rospy.Publisher(
            '/robot_state', 
            String, 
            queue_size=10
        )
        
        rospy.loginfo("ROS 퍼블리셔 설정 완료")
    
    def setup_ros_subscribers(self):
        """ROS 서브스크라이버 설정"""
        # 외부 cmd_vel 명령 수신 (선택적)
        self.cmd_vel_sub = rospy.Subscriber(
            '/external_cmd_vel', 
            Twist, 
            self.external_cmd_vel_callback
        )
        
        rospy.loginfo("ROS 서브스크라이버 설정 완료")
    
    def setup_ros_services(self):
        """ROS 서비스 설정"""
        # 안전 파라미터 설정 서비스
        self.set_safety_params_service = rospy.Service(
            '/set_safety_params',
            SetSafetyParams,
            self.set_safety_params_callback
        )
        
        # 비상 정지 서비스
        self.emergency_stop_service = rospy.Service(
            '/emergency_stop',
            SetBool,
            self.emergency_stop_callback
        )
        
        rospy.loginfo("ROS 서비스 설정 완료")
    
    def initialize_state_variables(self):
        """상태 변수 초기화"""
        # 센서 데이터
        self.current_scanner_data = None
        self.front_scanner_data = None
        self.rear_scanner_data = None
        
        # 안전 상태
        self.obstacle_detected = False
        self.warning_zone = False
        self.safe_zone = True
        self.is_emergency_stop = False
        
        # 제어 상태
        self.current_speed = 0.0
        self.current_direction = "stop"
        self.target_speed = 0.0
        
        # 통계
        self.obstacle_count = 0
        self.warning_count = 0
        self.cycle_count = 0
        
        # 센서 데이터 수신 상태
        self.sensor_data_received = False
        self.last_sensor_time = 0
        
        # 외부 명령 수신 상태
        self.external_command_active = False
        self.external_command_timeout = 2.0  # 2초 타임아웃
        self.last_external_command_time = 0
    
    def start_async_threads(self):
        """비동기 작업을 위한 스레드 시작"""
        # 센서 데이터 구독 스레드
        self.sensor_thread = threading.Thread(target=self.sensor_subscription_thread)
        self.sensor_thread.daemon = True
        self.sensor_thread.start()
        
        # 메인 제어 루프 스레드
        self.control_thread = threading.Thread(target=self.main_control_loop)
        self.control_thread.daemon = True
        self.control_thread.start()
        
        rospy.loginfo("비동기 스레드 시작 완료")
    
    def connect_robot_sync(self):
        """동기적으로 로봇 연결"""
        try:
            loop = asyncio.new_event_loop()
            asyncio.set_event_loop(loop)
            
            connected = loop.run_until_complete(self.robot.run())
            
            if connected:
                self.robot_connected = True
                rospy.loginfo("TR200 로봇 연결 성공")
                
                # 센서 데이터 구독 설정
                loop.run_until_complete(
                    self.robot.scanner_data_sub(self.scanner_data_callback, NO_PRINT)
                )
                rospy.loginfo("센서 데이터 구독 설정 완료")
            else:
                rospy.logerr("TR200 로봇 연결 실패")
                
        except Exception as e:
            rospy.logerr(f"로봇 연결 오류: {e}")
    
    def scanner_data_callback(self, scanner_data: ScannerData):
        """스캐너 데이터 콜백 함수"""
        self.current_scanner_data = scanner_data
        self.sensor_data_received = True
        self.last_sensor_time = time.time()
        
        # 전방/후방 센서 데이터 분리
        self.separate_front_rear_scanner_data(scanner_data)
        
        # ROS LaserScan 메시지로 변환하여 퍼블리시
        self.publish_laser_scan(scanner_data)
    
    def separate_front_rear_scanner_data(self, scanner_data: ScannerData):
        """스캐너 데이터를 전방/후방 센서로 분리"""
        if not scanner_data.ranges:
            return
        
        total_points = len(scanner_data.ranges)
        center_index = total_points // 2
        
        # 전방 섹터: 중앙에서 ±90도
        front_start = center_index - (total_points // 4)
        front_end = center_index + (total_points // 4)
        
        # 후방 섹터: 나머지 부분
        rear_start = front_end
        rear_end = front_start + total_points
        
        # 전방 센서 데이터 추출
        front_ranges = []
        for i in range(front_start, front_end):
            idx = i % total_points
            front_ranges.append(scanner_data.ranges[idx])
        
        # 후방 센서 데이터 추출
        rear_ranges = []
        for i in range(rear_start, rear_end):
            idx = i % total_points
            rear_ranges.append(scanner_data.ranges[idx])
        
        # 전방/후방 센서 데이터 저장
        self.front_scanner_data = {
            'ranges': front_ranges,
            'angle_min': scanner_data.angle_min,
            'angle_max': scanner_data.angle_max,
            'angle_increment': scanner_data.angle_increment
        }
        
        self.rear_scanner_data = {
            'ranges': rear_ranges,
            'angle_min': scanner_data.angle_min,
            'angle_max': scanner_data.angle_max,
            'angle_increment': scanner_data.angle_increment
        }
    
    def publish_laser_scan(self, scanner_data: ScannerData):
        """센서 데이터를 ROS LaserScan 메시지로 퍼블리시"""
        try:
            scan_msg = LaserScan()
            scan_msg.header.stamp = rospy.Time.now()
            scan_msg.header.frame_id = 'laser_link'
            
            scan_msg.angle_min = scanner_data.angle_min
            scan_msg.angle_max = scanner_data.angle_max
            scan_msg.angle_increment = scanner_data.angle_increment
            scan_msg.time_increment = 0.0
            scan_msg.scan_time = 1.0 / self.control_frequency
            scan_msg.range_min = scanner_data.range_min
            scan_msg.range_max = scanner_data.range_max
            scan_msg.ranges = list(scanner_data.ranges)
            scan_msg.intensities = []
            
            # LaserScan 퍼블리시 (별도 토픽)
            scan_pub = rospy.Publisher(self.scan_topic, LaserScan, queue_size=10)
            scan_pub.publish(scan_msg)
            
        except Exception as e:
            rospy.logwarn(f"LaserScan 퍼블리시 실패: {e}")
    
    def analyze_scanner_data(self) -> Tuple[bool, str, float]:
        """센서 데이터 분석하여 장애물 감지"""
        if not self.current_scanner_data or not self.front_scanner_data or not self.rear_scanner_data:
            return False, "센서 데이터 없음", 999.0
        
        # 전방 센서 분석
        front_min_distance, front_obstacle_count = self.analyze_single_scanner(
            self.front_scanner_data, "전방"
        )
        
        # 후방 센서 분석
        rear_min_distance, rear_obstacle_count = self.analyze_single_scanner(
            self.rear_scanner_data, "후방"
        )
        
        # 전체 최소 거리 계산
        min_distance = min(front_min_distance, rear_min_distance)
        total_obstacle_count = front_obstacle_count + rear_obstacle_count
        
        # 장애물 상태 판단
        if min_distance <= self.min_obstacle_distance:
            return True, "🔴 위험", min_distance
        elif min_distance <= self.warning_distance or total_obstacle_count >= 5:
            return True, "🟡 주의", min_distance
        else:
            return False, "🟢 안전", min_distance
    
    def analyze_single_scanner(self, scanner_data: dict, sensor_name: str) -> Tuple[float, int]:
        """단일 센서 데이터 분석"""
        ranges = scanner_data['ranges']
        if not ranges:
            return 999.0, 0
        
        min_distance = 999.0
        obstacle_count = 0
        
        for distance in ranges:
            if distance > 0 and distance < 10.0:
                if distance < min_distance:
                    min_distance = distance
                
                if distance <= self.warning_distance:
                    obstacle_count += 1
        
        return min_distance, obstacle_count
    
    def determine_speed(self, obstacle_detected: bool, warning_zone: bool, min_distance: float) -> float:
        """장애물 상태에 따라 속도 결정"""
        if obstacle_detected:
            return self.stop_speed
        elif warning_zone:
            if min_distance <= self.min_obstacle_distance + 0.1:
                return self.stop_speed
            else:
                return self.slow_speed
        else:
            return self.normal_speed
    
    def smooth_speed_change(self, target_speed: float, current_speed: float) -> float:
        """부드러운 속도 변화"""
        if abs(target_speed - current_speed) < 0.01:
            return target_speed
        
        max_change = 0.05
        if target_speed > current_speed:
            new_speed = min(current_speed + max_change, target_speed)
        else:
            new_speed = max(current_speed - max_change, target_speed)
        
        return new_speed
    
    def external_cmd_vel_callback(self, msg: Twist):
        """외부 cmd_vel 명령 콜백"""
        self.external_command_active = True
        self.last_external_command_time = time.time()
        
        # 외부 명령을 타겟 속도로 설정
        self.target_speed = msg.linear.x
        
        rospy.logdebug(f"외부 명령 수신: linear={msg.linear.x:.3f}, angular={msg.angular.z:.3f}")
    
    def set_safety_params_callback(self, req: SetSafetyParams) -> SetSafetyParamsResponse:
        """안전 파라미터 설정 서비스 콜백"""
        try:
            # 파라미터 업데이트
            self.min_obstacle_distance = req.min_obstacle_distance
            self.warning_distance = req.warning_distance
            self.safe_distance = req.safe_distance
            self.normal_speed = req.normal_speed
            self.slow_speed = req.slow_speed
            
            rospy.loginfo(f"안전 파라미터 업데이트:")
            rospy.loginfo(f"  최소 장애물 거리: {self.min_obstacle_distance}m")
            rospy.loginfo(f"  경고 거리: {self.warning_distance}m")
            rospy.loginfo(f"  안전 거리: {self.safe_distance}m")
            rospy.loginfo(f"  정상 속도: {self.normal_speed}m/s")
            rospy.loginfo(f"  감속 속도: {self.slow_speed}m/s")
            
            return SetSafetyParamsResponse(success=True, message="파라미터 업데이트 성공")
            
        except Exception as e:
            rospy.logerr(f"파라미터 업데이트 실패: {e}")
            return SetSafetyParamsResponse(success=False, message=str(e))
    
    def emergency_stop_callback(self, req: SetBool) -> SetBoolResponse:
        """비상 정지 서비스 콜백"""
        try:
            if req.data:
                self.is_emergency_stop = True
                self.target_speed = 0.0
                rospy.logwarn("비상 정지 활성화")
                return SetBoolResponse(success=True, message="비상 정지 활성화")
            else:
                self.is_emergency_stop = False
                rospy.loginfo("비상 정지 해제")
                return SetBoolResponse(success=True, message="비상 정지 해제")
                
        except Exception as e:
            rospy.logerr(f"비상 정지 처리 실패: {e}")
            return SetBoolResponse(success=False, message=str(e))
    
    def sensor_subscription_thread(self):
        """센서 데이터 구독 스레드"""
        try:
            loop = asyncio.new_event_loop()
            asyncio.set_event_loop(loop)
            
            # 센서 데이터 구독 대기
            while not rospy.is_shutdown():
                if self.robot_connected:
                    loop.run_until_complete(asyncio.sleep(0.1))
                else:
                    time.sleep(1.0)
                    
        except Exception as e:
            rospy.logerr(f"센서 구독 스레드 오류: {e}")
    
    def main_control_loop(self):
        """메인 제어 루프"""
        rate = rospy.Rate(self.control_frequency)
        
        while not rospy.is_shutdown():
            try:
                self.cycle_count += 1
                
                # 외부 명령 타임아웃 체크
                if self.external_command_active:
                    if time.time() - self.last_external_command_time > self.external_command_timeout:
                        self.external_command_active = False
                        rospy.logdebug("외부 명령 타임아웃")
                
                # 센서 데이터 수신 상태 확인
                if not self.sensor_data_received:
                    if self.cycle_count % 50 == 0:
                        rospy.logwarn("센서 데이터를 수신하지 못했습니다")
                elif time.time() - self.last_sensor_time > 2.0:
                    if self.cycle_count % 50 == 0:
                        rospy.logwarn("센서 데이터 수신이 중단되었습니다")
                
                # 센서 데이터 분석
                obstacle_detected, status, min_distance = self.analyze_scanner_data()
                
                # 상태 업데이트
                self.obstacle_detected = obstacle_detected
                self.warning_zone = (status == "🟡 주의")
                self.safe_zone = (status == "🟢 안전")
                
                # 속도 결정 (외부 명령이 있으면 우선 적용)
                if self.external_command_active:
                    target_speed = self.target_speed
                else:
                    target_speed = self.determine_speed(obstacle_detected, self.warning_zone, min_distance)
                
                # 비상 정지 상태면 속도 0으로 설정
                if self.is_emergency_stop:
                    target_speed = 0.0
                
                # 부드러운 속도 변화
                self.current_speed = self.smooth_speed_change(target_speed, self.current_speed)
                
                # 방향 결정 (외부 명령이 없을 때만 자동 전환)
                if not self.external_command_active:
                    if self.cycle_count % 100 == 0:  # 5초마다 방향 전환
                        if self.current_direction == "forward":
                            self.current_direction = "backward"
                        else:
                            self.current_direction = "forward"
                
                # 실제 속도 명령 (방향 고려)
                if self.current_direction == "backward":
                    actual_speed = -self.current_speed
                else:
                    actual_speed = self.current_speed
                
                # 위험 상태 처리
                if obstacle_detected and min_distance <= self.min_obstacle_distance:
                    rospy.logwarn(f"위험! 장애물 감지 (거리: {min_distance:.3f}m)")
                    self.emergency_stop()
                
                # ROS 메시지 퍼블리시
                self.publish_ros_messages(status, min_distance, actual_speed)
                
                # 로봇에 속도 명령 전송
                self.send_velocity_command(actual_speed)
                
                rate.sleep()
                
            except Exception as e:
                rospy.logerr(f"제어 루프 오류: {e}")
                rate.sleep()
    
    def publish_ros_messages(self, status: str, min_distance: float, actual_speed: float):
        """ROS 메시지 퍼블리시"""
        try:
            # 안전 상태 퍼블리시
            safety_msg = String()
            safety_msg.data = status
            self.safety_status_pub.publish(safety_msg)
            
            # 장애물 거리 퍼블리시
            distance_msg = Float32()
            distance_msg.data = min_distance
            self.obstacle_distance_pub.publish(distance_msg)
            
            # 비상 정지 상태 퍼블리시
            emergency_msg = Bool()
            emergency_msg.data = self.is_emergency_stop
            self.emergency_stop_pub.publish(emergency_msg)
            
            # 로봇 상태 퍼블리시
            state_msg = String()
            state_data = f"속도:{actual_speed:.3f}m/s, 방향:{self.current_direction}, 거리:{min_distance:.3f}m"
            state_msg.data = state_data
            self.robot_state_pub.publish(state_msg)
            
        except Exception as e:
            rospy.logwarn(f"ROS 메시지 퍼블리시 실패: {e}")
    
    def send_velocity_command(self, speed: float):
        """로봇에 속도 명령 전송"""
        if not self.robot_connected or not self.robot:
            return
        
        try:
            loop = asyncio.new_event_loop()
            asyncio.set_event_loop(loop)
            
            twist_cmd = WooshTwist(linear=speed, angular=0.0)
            _, ok, msg = loop.run_until_complete(
                self.robot.twist_req(twist_cmd, NO_PRINT, NO_PRINT)
            )
            
            if not ok:
                rospy.logwarn(f"속도 명령 전송 실패: {msg}")
                
        except Exception as e:
            rospy.logwarn(f"속도 명령 전송 오류: {e}")
    
    def emergency_stop(self):
        """비상 정지 수행"""
        rospy.logwarn("🚨 비상 정지 실행!")
        self.is_emergency_stop = True
        self.target_speed = 0.0
        
        # 즉시 정지 명령 전송
        for i in range(10):
            self.send_velocity_command(0.0)
            time.sleep(0.05)
        
        rospy.loginfo("✅ 비상 정지 완료")
    
    def cleanup(self):
        """리소스 정리"""
        rospy.loginfo("시스템 정리 중...")
        
        if self.robot and self.robot_connected:
            try:
                loop = asyncio.new_event_loop()
                asyncio.set_event_loop(loop)
                loop.run_until_complete(self.robot.stop())
                rospy.loginfo("로봇 연결 종료")
            except Exception as e:
                rospy.logwarn(f"로봇 연결 종료 오류: {e}")
        
        rospy.loginfo("시스템 정리 완료")

def main():
    """메인 함수"""
    try:
        controller = TR200ROSSensorSafetyController()
        
        # ROS 스핀
        rospy.spin()
        
    except rospy.ROSInterruptException:
        rospy.loginfo("ROS 인터럽트로 프로그램 종료")
    except Exception as e:
        rospy.logerr(f"프로그램 실행 오류: {e}")
    finally:
        if 'controller' in locals():
            controller.cleanup()

if __name__ == "__main__":
    main()
