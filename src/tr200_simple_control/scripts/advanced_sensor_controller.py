#!/usr/bin/env python3
# advanced_sensor_controller.py
# 고급 센서 기반 장애물 감지 및 안전 제어 시스템

import asyncio
import sys
import os
import time
import math
import numpy as np
from datetime import datetime

# SDK 경로 추가
sys.path.insert(0, '/catkin_ws/src/woosh_robot_py')

from woosh.proto.robot.robot_pb2 import PoseSpeed, OperationState, ScannerData
from woosh.proto.robot.robot_pack_pb2 import Twist
from woosh_interface import CommuSettings, NO_PRINT
from woosh_robot import WooshRobot

class AdvancedSensorController:
    """고급 센서 기반 안전 제어 시스템"""
    
    def __init__(self):
        self.settings = CommuSettings(addr='169.254.128.2', port=5480, identity='advanced-sensor-controller')
        self.robot = WooshRobot(self.settings)
        
        # 센서 설정
        self.min_obstacle_distance = 0.2  # 최소 장애물 거리 (m)
        self.warning_distance = 0.4       # 경고 거리 (m)
        self.safe_distance = 0.6          # 안전 거리 (m)
        
        # 제어 설정
        self.normal_speed = 0.15          # 정상 속도 (m/s)
        self.slow_speed = 0.08            # 감속 속도 (m/s)
        self.stop_speed = 0.0             # 정지 속도 (m/s)
        self.control_frequency = 20.0     # 제어 주파수 (Hz)
        
        # 센서 데이터
        self.current_scanner_data = None
        self.scanner_history = []  # 센서 데이터 히스토리
        self.max_history = 10     # 최대 히스토리 길이
        
        # 상태 변수
        self.current_speed = 0.0
        self.current_direction = "forward"
        self.emergency_stop = False
        self.direction_change_timer = 0
        
        # 통계
        self.obstacle_count = 0
        self.warning_count = 0
        self.total_distance = 0.0
        
    async def initialize_robot(self) -> bool:
        """로봇을 초기화하고 연결합니다."""
        try:
            print("🔍 고급 센서 기반 안전 제어 시스템을 시작합니다...")
            
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
        """스캐너 데이터 콜백 함수"""
        self.current_scanner_data = scanner_data
        
        # 히스토리에 추가
        self.scanner_history.append({
            'timestamp': time.time(),
            'ranges': list(scanner_data.ranges) if scanner_data.ranges else []
        })
        
        # 히스토리 길이 제한
        if len(self.scanner_history) > self.max_history:
            self.scanner_history.pop(0)
    
    def analyze_scanner_data_advanced(self) -> tuple:
        """고급 스캐너 데이터 분석"""
        if not self.current_scanner_data or not self.current_scanner_data.ranges:
            return False, "센서 데이터 없음", 999.0, {}
        
        ranges = self.current_scanner_data.ranges
        num_points = len(ranges)
        
        if num_points == 0:
            return False, "거리 데이터 없음", 999.0, {}
        
        # 섹터별 분석
        sectors = {
            'front': [],      # 전방 0-30도, 330-360도
            'front_left': [], # 전방 좌측 30-90도
            'left': [],       # 좌측 90-150도
            'back_left': [],  # 후방 좌측 150-210도
            'back': [],       # 후방 210-270도
            'back_right': [], # 후방 우측 270-330도
            'right': [],      # 우측 270-330도
            'front_right': [] # 전방 우측 330-360도, 0-30도
        }
        
        # 각 섹터별로 거리 데이터 분류
        for i, distance in enumerate(ranges):
            if distance <= 0:
                continue
                
            angle = i * (360.0 / num_points)
            
            if (angle >= 0 and angle <= 30) or (angle >= 330 and angle <= 360):
                sectors['front'].append(distance)
            elif angle > 30 and angle <= 90:
                sectors['front_left'].append(distance)
            elif angle > 90 and angle <= 150:
                sectors['left'].append(distance)
            elif angle > 150 and angle <= 210:
                sectors['back_left'].append(distance)
            elif angle > 210 and angle <= 270:
                sectors['back'].append(distance)
            elif angle > 270 and angle <= 330:
                sectors['back_right'].append(distance)
        
        # 각 섹터별 최소 거리 계산
        sector_min_distances = {}
        for sector, distances in sectors.items():
            if distances:
                sector_min_distances[sector] = min(distances)
            else:
                sector_min_distances[sector] = 999.0
        
        # 전방 장애물 감지 (가장 중요)
        front_distance = sector_min_distances['front']
        
        # 장애물 상태 판단
        if front_distance <= self.min_obstacle_distance:
            return True, "🔴 위험", front_distance, sector_min_distances
        elif front_distance <= self.warning_distance:
            return True, "🟡 주의", front_distance, sector_min_distances
        else:
            return False, "🟢 안전", front_distance, sector_min_distances
    
    def determine_speed_and_direction(self, obstacle_detected: bool, warning_zone: bool, 
                                    min_distance: float, sector_distances: dict) -> tuple:
        """장애물 상태에 따라 속도와 방향을 결정합니다."""
        
        # 기본 속도 결정
        if obstacle_detected:
            target_speed = self.stop_speed
        elif warning_zone:
            # 거리에 따라 점진적 감속
            if min_distance <= self.min_obstacle_distance + 0.1:
                target_speed = self.stop_speed
            else:
                target_speed = self.slow_speed
        else:
            target_speed = self.normal_speed
        
        # 방향 결정 (측면 장애물 고려)
        target_direction = self.current_direction
        
        # 전방에 장애물이 있으면 후진
        if obstacle_detected and min_distance <= self.min_obstacle_distance:
            target_direction = "backward"
        # 측면 장애물이 있으면 반대 방향으로 회피
        elif warning_zone:
            left_distance = sector_distances.get('front_left', 999.0)
            right_distance = sector_distances.get('front_right', 999.0)
            
            if left_distance < right_distance:
                target_direction = "right"  # 우측으로 회피
            else:
                target_direction = "left"  # 좌측으로 회피
        
        return target_speed, target_direction
    
    async def emergency_stop(self, reason: str):
        """비상 정지를 수행합니다."""
        print(f"🚨 비상 정지! 이유: {reason}")
        self.emergency_stop = True
        
        # 즉시 정지 명령 전송
        for i in range(10):
            await self.robot.twist_req(Twist(linear=0.0, angular=0.0), NO_PRINT, NO_PRINT)
            await asyncio.sleep(0.05)
        
        print("✅ 비상 정지 완료")
    
    async def smooth_speed_change(self, target_speed: float, current_speed: float) -> float:
        """부드러운 속도 변화를 수행합니다."""
        if abs(target_speed - current_speed) < 0.01:
            return target_speed
        
        # 속도 변화율 제한
        max_change = 0.03
        if target_speed > current_speed:
            new_speed = min(current_speed + max_change, target_speed)
        else:
            new_speed = max(current_speed - max_change, target_speed)
        
        return new_speed
    
    async def run_advanced_sensor_control(self) -> None:
        """고급 센서 기반 안전 제어를 실행합니다."""
        print("🎯 고급 센서 기반 안전 제어 시스템을 시작합니다!")
        print(f"   최소 장애물 거리: {self.min_obstacle_distance}m")
        print(f"   경고 거리: {self.warning_distance}m")
        print(f"   안전 거리: {self.safe_distance}m")
        print(f"   정상 속도: {self.normal_speed} m/s")
        print(f"   감속 속도: {self.slow_speed} m/s")
        print("   중지하려면 Ctrl+C를 누르세요.")
        print("=" * 70)
        
        delay = 1.0 / self.control_frequency
        cycle_count = 0
        
        try:
            while not self.emergency_stop:
                cycle_count += 1
                self.direction_change_timer += 1
                
                # 센서 데이터 분석
                obstacle_detected, status, min_distance, sector_distances = self.analyze_scanner_data_advanced()
                
                # 속도와 방향 결정
                target_speed, target_direction = self.determine_speed_and_direction(
                    obstacle_detected, status == "🟡 주의", min_distance, sector_distances
                )
                
                # 부드러운 속도 변화
                self.current_speed = await self.smooth_speed_change(target_speed, self.current_speed)
                
                # 방향 전환 (일정 시간마다)
                if self.direction_change_timer >= 100:  # 5초마다
                    if target_direction == "forward":
                        self.current_direction = "backward"
                    else:
                        self.current_direction = "forward"
                    self.direction_change_timer = 0
                
                # 실제 속도 명령
                if self.current_direction == "backward":
                    actual_speed = -self.current_speed
                else:
                    actual_speed = self.current_speed
                
                # 상태 출력 (매 20번째마다)
                if cycle_count % 20 == 0:
                    current_time = time.strftime("%H:%M:%S")
                    print(f"{current_time} | 방향: {self.current_direction} | 속도: {actual_speed:.3f} | 거리: {min_distance:.3f}m | 상태: {status}")
                    
                    # 섹터별 거리 정보 출력
                    if sector_distances:
                        print(f"   섹터 거리 - 전방: {sector_distances.get('front', 0):.2f}m, 좌측: {sector_distances.get('front_left', 0):.2f}m, 우측: {sector_distances.get('front_right', 0):.2f}m")
                
                # 경고 및 위험 상태 처리
                if obstacle_detected:
                    if min_distance <= self.min_obstacle_distance:
                        print(f"🚨 위험! 장애물 감지 (거리: {min_distance:.3f}m)")
                        await self.emergency_stop(f"장애물 감지 (거리: {min_distance:.3f}m)")
                        break
                    else:
                        self.obstacle_count += 1
                elif status == "🟡 주의":
                    if cycle_count % 40 == 0:  # 2초마다 경고
                        print(f"⚠️ 주의: 장애물 근접 (거리: {min_distance:.3f}m)")
                    self.warning_count += 1
                
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
    controller = AdvancedSensorController()
    
    try:
        # 로봇 초기화
        if not await controller.initialize_robot():
            print("❌ 로봇 초기화 실패")
            return
        
        print("✅ 로봇 초기화 완료")
        
        # 고급 센서 기반 안전 제어 실행
        await controller.run_advanced_sensor_control()
        
    except KeyboardInterrupt:
        print("\n⏹️ 사용자 요청으로 프로그램을 중지합니다.")
    except Exception as e:
        print(f"\n❌ 프로그램 실행 중 오류 발생: {e}")
    finally:
        await controller.cleanup()

if __name__ == "__main__":
    asyncio.run(main())
