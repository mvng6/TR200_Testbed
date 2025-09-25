#!/usr/bin/env python3
# sensor_based_safety_controller.py
# TR200 센서 기반 장애물 감지 및 안전 제어 시스템

import asyncio
import sys
import os
import time
import math
from datetime import datetime

# SDK 경로 추가
sys.path.insert(0, '/catkin_ws/src/woosh_robot_py')

from woosh.proto.robot.robot_pb2 import PoseSpeed, OperationState, ScannerData
from woosh.proto.robot.robot_pack_pb2 import Twist
from woosh_interface import CommuSettings, NO_PRINT
from woosh_robot import WooshRobot

class SensorBasedSafetyController:
    """센서 기반 안전 제어 시스템"""
    
    def __init__(self):
        self.settings = CommuSettings(addr='169.254.128.2', port=5480, identity='sensor-safety-controller')
        self.robot = WooshRobot(self.settings)
        
        # 로깅 레벨 조정 (INFO 메시지 최소화)
        import logging
        logging.getLogger('sensor-safety-controller').setLevel(logging.WARNING)
        
        # 센서 설정 (더 민감하게 조정)
        self.min_obstacle_distance = 0.5   # 최소 장애물 거리 (m) - 위험
        self.warning_distance = 0.8        # 경고 거리 (m) - 주의
        self.safe_distance = 1.0           # 안전 거리 (m) - 안전
        
        # 제어 설정
        self.normal_speed = 0.2           # 정상 속도 (m/s)
        self.slow_speed = 0.1             # 감속 속도 (m/s)
        self.stop_speed = 0.0             # 정지 속도 (m/s)
        self.control_frequency = 20.0     # 제어 주파수 (Hz)
        
        # 센서 데이터 (두 개 라이다 센서 시뮬레이션)
        self.current_scanner_data = None
        self.front_scanner_data = None    # 전방 라이다 센서
        self.rear_scanner_data = None     # 후방 라이다 센서
        self.obstacle_detected = False
        self.warning_zone = False
        self.safe_zone = True
        
        # 상태 변수
        self.current_speed = 0.0
        self.current_direction = "stop"
        self.is_emergency_stop = False
        
        # 통계
        self.obstacle_count = 0
        self.warning_count = 0
        
        # 센서 데이터 수신 상태
        self.sensor_data_received = False
        self.last_sensor_time = 0
        
    async def initialize_robot(self) -> bool:
        """로봇을 초기화하고 연결합니다."""
        try:
            print("🔍 센서 기반 안전 제어 시스템을 시작합니다...")
            
            if not await self.robot.run():
                print("❌ 로봇 연결 실패")
                return False
            
            print("✅ 로봇 연결 성공")
            
            # 스캐너 데이터 구독 설정
            await self.robot.scanner_data_sub(self.scanner_data_callback, NO_PRINT)
            print("✅ 스캐너 데이터 구독 설정 완료")
            
            return True
                
        except Exception as e:
            print(f"❌ 로봇 초기화 중 오류: {e}")
            return False
    
    def scanner_data_callback(self, scanner_data: ScannerData):
        """스캐너 데이터 콜백 함수 - 두 개 라이다 센서 데이터 분리"""
        self.current_scanner_data = scanner_data
        self.sensor_data_received = True
        self.last_sensor_time = time.time()
        
        # 두 개 라이다 센서 데이터 분리 (TR200는 앞뒤에 라이다 센서가 있음)
        self.separate_front_rear_scanner_data(scanner_data)
        
        # 디버깅을 위해 첫 번째 데이터만 출력
        if not hasattr(self, '_first_data_printed'):
            print(f"📡 스캐너 데이터 수신: {len(scanner_data.ranges)}개 포인트")
            if hasattr(scanner_data, 'angle_min') and hasattr(scanner_data, 'angle_max'):
                print(f"📡 각도 범위: {scanner_data.angle_min:.2f}° ~ {scanner_data.angle_max:.2f}°")
            if hasattr(scanner_data, 'range_min') and hasattr(scanner_data, 'range_max'):
                print(f"📡 거리 범위: {scanner_data.range_min:.2f}m ~ {scanner_data.range_max:.2f}m")
            print("=" * 60)
            self._first_data_printed = True
    
    def separate_front_rear_scanner_data(self, scanner_data: ScannerData):
        """스캐너 데이터를 전방/후방 센서로 분리"""
        if not scanner_data.ranges:
            return
        
        total_points = len(scanner_data.ranges)
        center_index = total_points // 2
        
        # 전방 섹터: 중앙에서 ±90도 (전방 180도)
        front_start = center_index - (total_points // 4)
        front_end = center_index + (total_points // 4)
        
        # 후방 섹터: 나머지 부분 (후방 180도)
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
        
        # 디버깅 정보 출력 (한 번만)
        if not hasattr(self, '_sensor_separation_printed'):
            print(f"🔍 센서 분리 완료:")
            print(f"   전방 센서: {len(front_ranges)}개 포인트 (인덱스 {front_start}~{front_end})")
            print(f"   후방 센서: {len(rear_ranges)}개 포인트 (인덱스 {rear_start}~{rear_end})")
            self._sensor_separation_printed = True
    
    def analyze_scanner_data(self) -> tuple:
        """두 개 라이다 센서 데이터를 분석하여 장애물을 감지합니다."""
        if not self.current_scanner_data or not self.front_scanner_data or not self.rear_scanner_data:
            return False, "센서 데이터 없음", 999.0
        
        # 전방 센서 분석
        front_min_distance, front_obstacle_count = self.analyze_single_scanner(self.front_scanner_data, "전방")
        
        # 후방 센서 분석
        rear_min_distance, rear_obstacle_count = self.analyze_single_scanner(self.rear_scanner_data, "후방")
        
        # 전체 최소 거리 계산 (전방과 후방 중 더 가까운 거리)
        min_distance = min(front_min_distance, rear_min_distance)
        total_obstacle_count = front_obstacle_count + rear_obstacle_count
        
        # 디버깅 정보 출력 (처음 몇 번만)
        if not hasattr(self, '_dual_sensor_analysis_printed'):
            print(f"🔍 듀얼 센서 분석:")
            print(f"   전방 센서: 최소거리 {front_min_distance:.3f}m, 장애물 {front_obstacle_count}개")
            print(f"   후방 센서: 최소거리 {rear_min_distance:.3f}m, 장애물 {rear_obstacle_count}개")
            print(f"   전체 최소거리: {min_distance:.3f}m, 총 장애물: {total_obstacle_count}개")
            self._dual_sensor_analysis_printed = True
        
        # 장애물 상태 판단 (더 엄격한 기준)
        if min_distance <= self.min_obstacle_distance:
            return True, "🔴 위험", min_distance
        elif min_distance <= self.warning_distance or total_obstacle_count >= 5:
            return True, "🟡 주의", min_distance
        else:
            return False, "🟢 안전", min_distance
    
    def analyze_single_scanner(self, scanner_data: dict, sensor_name: str) -> tuple:
        """단일 센서 데이터를 분석합니다."""
        ranges = scanner_data['ranges']
        if not ranges:
            return 999.0, 0
        
        min_distance = 999.0
        obstacle_count = 0
        
        # 센서 데이터 분석
        for distance in ranges:
            if distance > 0 and distance < 10.0:  # 유효한 거리 데이터만 처리
                if distance < min_distance:
                    min_distance = distance
                
                # 장애물 개수 카운트 (거리 기반)
                if distance <= self.warning_distance:
                    obstacle_count += 1
        
        return min_distance, obstacle_count
    
    def determine_speed(self, obstacle_detected: bool, warning_zone: bool, min_distance: float) -> float:
        """장애물 상태에 따라 속도를 결정합니다."""
        if obstacle_detected:
            return self.stop_speed
        elif warning_zone:
            # 거리에 따라 점진적 감속
            if min_distance <= self.min_obstacle_distance + 0.1:
                return self.stop_speed
            else:
                return self.slow_speed
        else:
            return self.normal_speed
    
    async def emergency_stop(self, reason: str):
        """비상 정지를 수행합니다."""
        print(f"🚨 비상 정지! 이유: {reason}")
        self.is_emergency_stop = True
        
        # 즉시 정지 명령 전송 (더 강력하게)
        for i in range(20):  # 더 많은 정지 명령 전송
            await self.robot.twist_req(Twist(linear=0.0, angular=0.0), NO_PRINT, NO_PRINT)
            await asyncio.sleep(0.02)  # 더 빠른 주기로 전송
        
        # 추가 안전 조치: 음수 속도로 역방향 제동
        for i in range(5):
            await self.robot.twist_req(Twist(linear=-0.1, angular=0.0), NO_PRINT, NO_PRINT)
            await asyncio.sleep(0.05)
        
        # 최종 정지
        for i in range(10):
            await self.robot.twist_req(Twist(linear=0.0, angular=0.0), NO_PRINT, NO_PRINT)
            await asyncio.sleep(0.05)
        
        print("✅ 비상 정지 완료")
    
    async def smooth_speed_change(self, target_speed: float, current_speed: float) -> float:
        """부드러운 속도 변화를 수행합니다."""
        if abs(target_speed - current_speed) < 0.01:
            return target_speed
        
        # 속도 변화율 제한 (0.05 m/s per step)
        max_change = 0.05
        if target_speed > current_speed:
            new_speed = min(current_speed + max_change, target_speed)
        else:
            new_speed = max(current_speed - max_change, target_speed)
        
        return new_speed
    
    async def run_sensor_based_control(self) -> None:
        """센서 기반 안전 제어를 실행합니다."""
        print("🎯 센서 기반 안전 제어 시스템을 시작합니다!")
        print(f"   최소 장애물 거리: {self.min_obstacle_distance}m")
        print(f"   경고 거리: {self.warning_distance}m")
        print(f"   안전 거리: {self.safe_distance}m")
        print(f"   정상 속도: {self.normal_speed} m/s")
        print(f"   감속 속도: {self.slow_speed} m/s")
        print("   중지하려면 Ctrl+C를 누르세요.")
        print("=" * 80)
        print("시간     | 방향      | 속도    | 거리      | 상태")
        print("-" * 80)
        
        delay = 1.0 / self.control_frequency
        cycle_count = 0
        
        try:
            while not self.is_emergency_stop:
                cycle_count += 1
                
                # 센서 데이터 수신 상태 확인
                if not self.sensor_data_received:
                    if cycle_count % 50 == 0:  # 2.5초마다 경고
                        print("⚠️ 센서 데이터를 수신하지 못했습니다. 센서 연결을 확인하세요.")
                elif time.time() - self.last_sensor_time > 2.0:  # 2초 이상 센서 데이터 없음
                    if cycle_count % 50 == 0:
                        print("⚠️ 센서 데이터 수신이 중단되었습니다.")
                
                # 센서 데이터 분석
                obstacle_detected, status, min_distance = self.analyze_scanner_data()
                
                # 상태 업데이트
                self.obstacle_detected = obstacle_detected
                self.warning_zone = (status == "🟡 주의")
                self.safe_zone = (status == "🟢 안전")
                
                # 속도 결정
                target_speed = self.determine_speed(obstacle_detected, self.warning_zone, min_distance)
                
                # 부드러운 속도 변화
                self.current_speed = await self.smooth_speed_change(target_speed, self.current_speed)
                
                # 방향 결정 (간단한 앞뒤 왕복)
                if cycle_count % 100 == 0:  # 5초마다 방향 전환
                    if self.current_direction == "forward":
                        self.current_direction = "backward"
                    else:
                        self.current_direction = "forward"
                
                # 실제 속도 명령 (방향 고려)
                if self.current_direction == "backward":
                    actual_speed = -self.current_speed
                else:
                    actual_speed = self.current_speed
                
                # 경고 및 위험 상태 처리
                if obstacle_detected:
                    if min_distance <= self.min_obstacle_distance:
                        print(f"🚨 위험! 장애물 감지 (거리: {min_distance:.3f}m)")
                        await self.emergency_stop(f"장애물 감지 (거리: {min_distance:.3f}m)")
                        break
                elif self.warning_zone:
                    if cycle_count % 20 == 0:  # 1초마다 경고
                        print(f"⚠️ 주의: 장애물 근접 (거리: {min_distance:.3f}m)")
                
                # 상태 출력 (매 10번째마다) - 깔끔한 출력
                if cycle_count % 10 == 0:
                    current_time = time.strftime("%H:%M:%S")
                    print(f"{current_time} | {self.current_direction:8} | {actual_speed:6.3f} | {min_distance:6.3f}m | {status}")
                
                # 속도 명령 전송
                _, ok, msg = await self.robot.twist_req(
                    Twist(linear=actual_speed, angular=0.0), NO_PRINT, NO_PRINT
                )
                
                if not ok:
                    print(f"❌ 속도 제어 요청 실패: {msg}")
                    await self.emergency_stop("속도 제어 실패")
                    break
                
                await asyncio.sleep(delay)
                
        except KeyboardInterrupt:
            print("\n⏹️ 사용자 요청으로 구동 중단")
        except Exception as e:
            print(f"\n❌ 구동 중 오류 발생: {e}")
        finally:
            # 최종 정지
            await self.robot.twist_req(Twist(linear=0.0, angular=0.0), NO_PRINT, NO_PRINT)
            print(f"🏁 총 {cycle_count}개 사이클 완료")
            print(f"📊 장애물 감지 횟수: {self.obstacle_count}")
            print(f"📊 경고 횟수: {self.warning_count}")
    
    async def cleanup(self) -> None:
        """리소스를 정리합니다."""
        print("🔌 시스템 정리 중...")
        
        if self.robot.comm.is_connected():
            await self.robot.stop()
            print("✅ 로봇 연결 종료")
        
        print("✅ 시스템 정리 완료")

async def main():
    """메인 실행 함수"""
    controller = SensorBasedSafetyController()
    
    try:
        # 로봇 초기화
        if not await controller.initialize_robot():
            print("❌ 로봇 초기화 실패")
            return
        
        print("✅ 로봇 초기화 완료")
        
        # 센서 기반 안전 제어 실행
        await controller.run_sensor_based_control()
        
    except KeyboardInterrupt:
        print("\n⏹️ 사용자 요청으로 프로그램을 중지합니다.")
    except Exception as e:
        print(f"\n❌ 프로그램 실행 중 오류 발생: {e}")
    finally:
        await controller.cleanup()

if __name__ == "__main__":
    asyncio.run(main())
