#!/usr/bin/env python3
# safety_controller_node.py
# TR200 안전 제어 전용 노드

import rospy
import time
from typing import Tuple

# ROS 메시지 타입
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Float32, Bool
from std_srvs.srv import SetBool, SetBoolResponse
from tr200_simple_control.srv import SetSafetyParams, SetSafetyParamsResponse

class SafetyControllerNode:
    """TR200 안전 제어 전용 노드"""
    
    def __init__(self):
        """안전 제어 노드 초기화"""
        rospy.init_node('safety_controller', anonymous=True)
        
        # ROS 파라미터 로드
        self.load_parameters()
        
        # ROS 퍼블리셔 설정
        self.setup_publishers()
        
        # ROS 서브스크라이버 설정
        self.setup_subscribers()
        
        # ROS 서비스 설정
        self.setup_services()
        
        # 상태 변수 초기화
        self.initialize_variables()
        
        # 메인 제어 루프 시작
        self.start_control_loop()
        
        rospy.loginfo("안전 제어 노드가 시작되었습니다")
    
    def load_parameters(self):
        """파라미터 로드"""
        # 안전 설정
        self.min_obstacle_distance = rospy.get_param('~min_obstacle_distance', 0.5)
        self.warning_distance = rospy.get_param('~warning_distance', 0.8)
        self.safe_distance = rospy.get_param('~safe_distance', 1.0)
        
        # 제어 설정
        self.normal_speed = rospy.get_param('~normal_speed', 0.2)
        self.slow_speed = rospy.get_param('~slow_speed', 0.1)
        self.stop_speed = rospy.get_param('~stop_speed', 0.0)
        self.control_frequency = rospy.get_param('~control_frequency', 20.0)
        
        # 토픽 설정
        self.processed_scan_topic = rospy.get_param('~processed_scan_topic', '/processed_scan')
        self.safe_cmd_vel_topic = rospy.get_param('~safe_cmd_vel_topic', '/safe_cmd_vel')
        self.safety_status_topic = rospy.get_param('~safety_status_topic', '/safety_status')
        
        rospy.loginfo(f"안전 제어 파라미터: 위험거리={self.min_obstacle_distance}m, 경고거리={self.warning_distance}m")
    
    def setup_publishers(self):
        """ROS 퍼블리셔 설정"""
        # 안전한 속도 명령
        self.safe_cmd_vel_pub = rospy.Publisher(self.safe_cmd_vel_topic, Twist, queue_size=10)
        
        # 안전 상태
        self.safety_status_pub = rospy.Publisher(self.safety_status_topic, String, queue_size=10)
        
        # 장애물 거리
        self.obstacle_distance_pub = rospy.Publisher('/obstacle_distance', Float32, queue_size=10)
        
        # 비상 정지 상태
        self.emergency_stop_pub = rospy.Publisher('/emergency_stop', Bool, queue_size=10)
        
        rospy.loginfo("안전 제어 퍼블리셔 설정 완료")
    
    def setup_subscribers(self):
        """ROS 서브스크라이버 설정"""
        # 처리된 센서 데이터 수신
        self.processed_scan_sub = rospy.Subscriber(
            self.processed_scan_topic, 
            LaserScan, 
            self.processed_scan_callback
        )
        
        # 외부 속도 명령 수신
        self.external_cmd_vel_sub = rospy.Subscriber(
            '/external_cmd_vel', 
            Twist, 
            self.external_cmd_vel_callback
        )
        
        rospy.loginfo("안전 제어 서브스크라이버 설정 완료")
    
    def setup_services(self):
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
        
        rospy.loginfo("안전 제어 서비스 설정 완료")
    
    def initialize_variables(self):
        """상태 변수 초기화"""
        # 센서 데이터
        self.current_scan_data = None
        self.scan_data_received = False
        self.last_scan_time = 0
        
        # 안전 상태
        self.obstacle_detected = False
        self.warning_zone = False
        self.safe_zone = True
        self.is_emergency_stop = False
        
        # 제어 상태
        self.external_command_active = False
        self.external_command_timeout = 2.0
        self.last_external_command_time = 0
        self.target_speed = 0.0
        
        # 통계
        self.cycle_count = 0
        self.obstacle_count = 0
        self.warning_count = 0
    
    def processed_scan_callback(self, scan_msg: LaserScan):
        """처리된 센서 데이터 콜백"""
        self.current_scan_data = scan_msg
        self.scan_data_received = True
        self.last_scan_time = time.time()
    
    def external_cmd_vel_callback(self, msg: Twist):
        """외부 속도 명령 콜백"""
        self.external_command_active = True
        self.last_external_command_time = time.time()
        self.target_speed = msg.linear.x
        
        rospy.logdebug(f"외부 명령 수신: linear={msg.linear.x:.3f}")
    
    def analyze_sensor_data(self) -> Tuple[bool, str, float]:
        """센서 데이터 분석하여 장애물 감지"""
        if not self.current_scan_data:
            return False, "센서 데이터 없음", 999.0
        
        ranges = self.current_scan_data.ranges
        if not ranges:
            return False, "센서 데이터 없음", 999.0
        
        # 최소 거리 계산
        min_distance = 999.0
        obstacle_count = 0
        
        for distance in ranges:
            if 0.1 < distance < 10.0:
                if distance < min_distance:
                    min_distance = distance
                
                if distance <= self.warning_distance:
                    obstacle_count += 1
        
        # 장애물 상태 판단
        if min_distance <= self.min_obstacle_distance:
            return True, "🔴 위험", min_distance
        elif min_distance <= self.warning_distance or obstacle_count >= 5:
            return True, "🟡 주의", min_distance
        else:
            return False, "🟢 안전", min_distance
    
    def determine_safe_speed(self, obstacle_detected: bool, warning_zone: bool, min_distance: float) -> float:
        """안전한 속도 결정"""
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
            rospy.loginfo(f"  정상 속도: {self.normal_speed}m/s")
            
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
    
    def start_control_loop(self):
        """메인 제어 루프 시작"""
        self.control_thread = rospy.Timer(rospy.Duration(1.0 / self.control_frequency), self.control_loop_callback)
    
    def control_loop_callback(self, event):
        """제어 루프 콜백"""
        try:
            self.cycle_count += 1
            
            # 외부 명령 타임아웃 체크
            if self.external_command_active:
                if time.time() - self.last_external_command_time > self.external_command_timeout:
                    self.external_command_active = False
                    rospy.logdebug("외부 명령 타임아웃")
            
            # 센서 데이터 수신 상태 확인
            if not self.scan_data_received:
                if self.cycle_count % 50 == 0:
                    rospy.logwarn("센서 데이터를 수신하지 못했습니다")
            elif time.time() - self.last_scan_time > 2.0:
                if self.cycle_count % 50 == 0:
                    rospy.logwarn("센서 데이터 수신이 중단되었습니다")
            
            # 센서 데이터 분석
            obstacle_detected, status, min_distance = self.analyze_sensor_data()
            
            # 상태 업데이트
            self.obstacle_detected = obstacle_detected
            self.warning_zone = (status == "🟡 주의")
            self.safe_zone = (status == "🟢 안전")
            
            # 속도 결정 (외부 명령이 있으면 우선 적용)
            if self.external_command_active:
                target_speed = self.target_speed
            else:
                target_speed = self.determine_safe_speed(obstacle_detected, self.warning_zone, min_distance)
            
            # 비상 정지 상태면 속도 0으로 설정
            if self.is_emergency_stop:
                target_speed = 0.0
            
            # 부드러운 속도 변화
            current_speed = self.smooth_speed_change(target_speed, 0.0)  # 간단화
            
            # 실제 속도 명령 생성
            cmd_vel_msg = Twist()
            cmd_vel_msg.linear.x = current_speed
            cmd_vel_msg.angular.z = 0.0
            
            # 위험 상태 처리
            if obstacle_detected and min_distance <= self.min_obstacle_distance:
                rospy.logwarn(f"위험! 장애물 감지 (거리: {min_distance:.3f}m)")
                cmd_vel_msg.linear.x = 0.0  # 즉시 정지
            
            # 안전한 속도 명령 퍼블리시
            self.safe_cmd_vel_pub.publish(cmd_vel_msg)
            
            # 상태 메시지 퍼블리시
            self.publish_status_messages(status, min_distance, current_speed)
            
        except Exception as e:
            rospy.logerr(f"제어 루프 오류: {e}")
    
    def publish_status_messages(self, status: str, min_distance: float, current_speed: float):
        """상태 메시지 퍼블리시"""
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
            
        except Exception as e:
            rospy.logwarn(f"상태 메시지 퍼블리시 실패: {e}")
    
    def cleanup(self):
        """리소스 정리"""
        rospy.loginfo("안전 제어 노드 정리 중...")
        rospy.loginfo("안전 제어 노드 정리 완료")

def main():
    """메인 함수"""
    try:
        controller = SafetyControllerNode()
        
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
