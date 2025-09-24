#!/usr/bin/env python3
# sensor_processor_node.py
# TR200 센서 데이터 처리 전용 노드

import rospy
import asyncio
import threading
import time
from typing import Optional, Tuple

# ROS 메시지 타입
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32, String

# SDK 경로 추가
import sys
sys.path.insert(0, '/catkin_ws/src/woosh_robot_py')

from woosh.proto.robot.robot_pb2 import ScannerData
from woosh_interface import CommuSettings, NO_PRINT
from woosh_robot import WooshRobot

class SensorProcessorNode:
    """TR200 센서 데이터 처리 전용 노드"""
    
    def __init__(self):
        """센서 처리 노드 초기화"""
        rospy.init_node('sensor_processor', anonymous=True)
        
        # ROS 파라미터 로드
        self.load_parameters()
        
        # Woosh SDK 초기화
        self.setup_woosh_sdk()
        
        # ROS 퍼블리셔 설정
        self.setup_publishers()
        
        # 상태 변수 초기화
        self.initialize_variables()
        
        # 센서 데이터 구독 스레드 시작
        self.start_sensor_thread()
        
        rospy.loginfo("센서 처리 노드가 시작되었습니다")
    
    def load_parameters(self):
        """파라미터 로드"""
        self.robot_ip = rospy.get_param('~robot_ip', '169.254.128.2')
        self.robot_port = rospy.get_param('~robot_port', 5480)
        self.scan_topic = rospy.get_param('~scan_topic', '/scan')
        self.processed_scan_topic = rospy.get_param('~processed_scan_topic', '/processed_scan')
        
        rospy.loginfo(f"센서 처리 파라미터: {self.robot_ip}:{self.robot_port}")
    
    def setup_woosh_sdk(self):
        """Woosh SDK 초기화"""
        try:
            self.settings = CommuSettings(
                addr=self.robot_ip,
                port=self.robot_port,
                identity='sensor-processor'
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
    
    def setup_publishers(self):
        """ROS 퍼블리셔 설정"""
        # 원본 라이다 데이터
        self.scan_pub = rospy.Publisher(self.scan_topic, LaserScan, queue_size=10)
        
        # 처리된 센서 데이터
        self.processed_scan_pub = rospy.Publisher(self.processed_scan_topic, LaserScan, queue_size=10)
        
        # 센서 상태
        self.sensor_status_pub = rospy.Publisher('/sensor_status', String, queue_size=10)
        
        rospy.loginfo("센서 처리 퍼블리셔 설정 완료")
    
    def initialize_variables(self):
        """상태 변수 초기화"""
        self.current_scanner_data = None
        self.front_scanner_data = None
        self.rear_scanner_data = None
        self.sensor_data_received = False
        self.last_sensor_time = 0
        self.cycle_count = 0
    
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
        
        # 처리된 센서 데이터 퍼블리시
        self.publish_processed_scan()
    
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
        """원본 센서 데이터를 ROS LaserScan 메시지로 퍼블리시"""
        try:
            scan_msg = LaserScan()
            scan_msg.header.stamp = rospy.Time.now()
            scan_msg.header.frame_id = 'laser_link'
            
            scan_msg.angle_min = scanner_data.angle_min
            scan_msg.angle_max = scanner_data.angle_max
            scan_msg.angle_increment = scanner_data.angle_increment
            scan_msg.time_increment = 0.0
            scan_msg.scan_time = 0.05  # 20Hz
            scan_msg.range_min = scanner_data.range_min
            scan_msg.range_max = scanner_data.range_max
            scan_msg.ranges = list(scanner_data.ranges)
            scan_msg.intensities = []
            
            self.scan_pub.publish(scan_msg)
            
        except Exception as e:
            rospy.logwarn(f"LaserScan 퍼블리시 실패: {e}")
    
    def publish_processed_scan(self):
        """처리된 센서 데이터 퍼블리시"""
        if not self.current_scanner_data:
            return
        
        try:
            # 노이즈 제거 및 필터링된 데이터 생성
            processed_ranges = []
            for distance in self.current_scanner_data.ranges:
                if 0.1 < distance < 10.0:  # 유효한 거리만
                    processed_ranges.append(distance)
                else:
                    processed_ranges.append(float('inf'))
            
            # 처리된 LaserScan 메시지 생성
            processed_msg = LaserScan()
            processed_msg.header.stamp = rospy.Time.now()
            processed_msg.header.frame_id = 'laser_link'
            processed_msg.angle_min = self.current_scanner_data.angle_min
            processed_msg.angle_max = self.current_scanner_data.angle_max
            processed_msg.angle_increment = self.current_scanner_data.angle_increment
            processed_msg.time_increment = 0.0
            processed_msg.scan_time = 0.05
            processed_msg.range_min = self.current_scanner_data.range_min
            processed_msg.range_max = self.current_scanner_data.range_max
            processed_msg.ranges = processed_ranges
            processed_msg.intensities = []
            
            self.processed_scan_pub.publish(processed_msg)
            
        except Exception as e:
            rospy.logwarn(f"처리된 센서 데이터 퍼블리시 실패: {e}")
    
    def start_sensor_thread(self):
        """센서 데이터 구독 스레드 시작"""
        self.sensor_thread = threading.Thread(target=self.sensor_subscription_thread)
        self.sensor_thread.daemon = True
        self.sensor_thread.start()
    
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
    
    def cleanup(self):
        """리소스 정리"""
        rospy.loginfo("센서 처리 노드 정리 중...")
        
        if self.robot and self.robot_connected:
            try:
                loop = asyncio.new_event_loop()
                asyncio.set_event_loop(loop)
                loop.run_until_complete(self.robot.stop())
                rospy.loginfo("로봇 연결 종료")
            except Exception as e:
                rospy.logwarn(f"로봇 연결 종료 오류: {e}")
        
        rospy.loginfo("센서 처리 노드 정리 완료")

def main():
    """메인 함수"""
    try:
        processor = SensorProcessorNode()
        
        # ROS 스핀
        rospy.spin()
        
    except rospy.ROSInterruptException:
        rospy.loginfo("ROS 인터럽트로 프로그램 종료")
    except Exception as e:
        rospy.logerr(f"프로그램 실행 오류: {e}")
    finally:
        if 'processor' in locals():
            processor.cleanup()

if __name__ == "__main__":
    main()
