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
        
        # 센서 설정
        self.min_obstacle_distance = 0.3  # 최소 장애물 거리 (m)
        self.warning_distance = 0.5       # 경고 거리 (m)
        self.safe_distance = 0.8          # 안전 거리 (m)
        
        # 제어 설정
        self.normal_speed = 0.2           # 정상 속도 (m/s)
        self.slow_speed = 0.1             # 감속 속도 (m/s)
        self.stop_speed = 0.0             # 정지 속도 (m/s)
        self.control_frequency = 20.0     # 제어 주파수 (Hz)
        
        # 센서 데이터
        self.current_scanner_data = None
        self.obstacle_detected = False
        self.warning_zone = False
        self.safe_zone = True
        
        # 상태 변수
        self.current_speed = 0.0
        self.current_direction = "stop"
        self.emergency_stop = False
        
        # 통계
        self.obstacle_count = 0
        self.warning_count = 0
        
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
        """스캐너 데이터 콜백 함수"""
        self.current_scanner_data = scanner_data
        # print(f"📡 스캐너 데이터 수신: {len(scanner_data.ranges)}개 포인트")
    
    def analyze_scanner_data(self) -> tuple:
        """스캐너 데이터를 분석하여 장애물을 감지합니다."""
        if not self.current_scanner_data:
            return False, "센서 데이터 없음", 999.0
        
        ranges = self.current_scanner_data.ranges
        
        if not ranges:
            return False, "거리 데이터 없음", 999.0
        
        # 전방 180도 범위에서 최소 거리 찾기 (로봇 전진 방향)
        min_distance = 999.0
        obstacle_angle = 0
        
        # 스캐너 데이터 분석 (일반적으로 360도 스캔)
        for i, distance in enumerate(ranges):
            if distance > 0 and distance < min_distance:
                # 전방 180도 범위만 고려 (0도~180도, 270도~360도)
                angle = i * (360.0 / len(ranges))
                if (angle >= 0 and angle <= 180) or (angle >= 270 and angle <= 360):
                    min_distance = distance
                    obstacle_angle = angle
        
        # 장애물 상태 판단
        if min_distance <= self.min_obstacle_distance:
            return True, "🔴 위험", min_distance
        elif min_distance <= self.warning_distance:
            return True, "🟡 주의", min_distance
        else:
            return False, "🟢 안전", min_distance
    
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
        print("=" * 60)
        
        delay = 1.0 / self.control_frequency
        cycle_count = 0
        
        try:
            while not self.emergency_stop:
                cycle_count += 1
                
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
                
                # 상태 출력 (매 10번째마다)
                if cycle_count % 10 == 0:
                    current_time = time.strftime("%H:%M:%S")
                    print(f"{current_time} | 방향: {self.current_direction} | 속도: {actual_speed:.3f} | 거리: {min_distance:.3f}m | 상태: {status}")
                
                # 경고 및 위험 상태 처리
                if obstacle_detected:
                    if min_distance <= self.min_obstacle_distance:
                        print(f"🚨 위험! 장애물 감지 (거리: {min_distance:.3f}m)")
                        await self.emergency_stop(f"장애물 감지 (거리: {min_distance:.3f}m)")
                        break
                elif self.warning_zone:
                    if cycle_count % 20 == 0:  # 1초마다 경고
                        print(f"⚠️ 주의: 장애물 근접 (거리: {min_distance:.3f}m)")
                
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
