#!/usr/bin/env python3
# integrated_safety_controller.py
# 통합 안전 제어 시스템 (장애물 감지 + 영역 제한 + 실시간 모니터링)

import asyncio
import sys
import os
import time
from datetime import datetime

# SDK 경로 추가
sys.path.insert(0, '/catkin_ws/src/woosh_robot_py')

from woosh.proto.robot.robot_pb2 import PoseSpeed, OperationState
from woosh.proto.robot.robot_pack_pb2 import Twist
from woosh_interface import CommuSettings, NO_PRINT
from woosh_robot import WooshRobot

class IntegratedSafetyController:
    """통합 안전 제어 시스템"""
    
    def __init__(self):
        self.settings = CommuSettings(addr='169.254.128.2', port=5480, identity='safety-controller')
        self.robot = WooshRobot(self.settings)
        
        # 안전 설정
        self.start_position = None
        self.max_distance = 0.5  # 최대 이동 거리 (m)
        self.safety_margin = 0.01  # 안전 여유 거리 (m)
        self.control_frequency = 20.0
        
        # 구동 설정
        self.forward_speed = 0.15
        self.backward_speed = -0.15
        self.move_duration = 2.0
        self.wait_duration = 1.0
        
        # 안전 상태
        self.is_safe = True
        self.emergency_stop = False
        self.current_direction = "stop"
        
        # 통계
        self.max_distance_reached = 0.0
        self.cycle_count = 0
        
    async def initialize_robot(self) -> bool:
        """로봇을 초기화하고 연결합니다."""
        try:
            print("🔍 통합 안전 제어 시스템을 시작합니다...")
            
            if not await self.robot.run():
                print("❌ 로봇 연결 실패")
                return False
            
            print("✅ 로봇 연결 성공")
            
            # 초기 위치 설정
            pose_speed, ok, msg = await self.robot.robot_pose_speed_req(
                PoseSpeed(), NO_PRINT, NO_PRINT
            )
            
            if ok:
                self.start_position = {
                    'x': pose_speed.pose.x,
                    'y': pose_speed.pose.y,
                    'theta': pose_speed.pose.theta
                }
                print(f"📍 시작 위치: X={self.start_position['x']:.3f}, Y={self.start_position['y']:.3f}")
                return True
            else:
                print(f"❌ 초기 위치 요청 실패: {msg}")
                return False
                
        except Exception as e:
            print(f"❌ 로봇 초기화 중 오류: {e}")
            return False
    
    def calculate_distance_from_start(self, current_x: float, current_y: float) -> float:
        """시작점으로부터의 거리를 계산합니다."""
        if not self.start_position:
            return 0.0
        
        dx = current_x - self.start_position['x']
        dy = current_y - self.start_position['y']
        return (dx**2 + dy**2)**0.5
    
    def check_safety_status(self, distance: float) -> tuple:
        """안전 상태를 확인합니다."""
        max_allowed = self.max_distance - self.safety_margin
        
        if distance >= max_allowed:
            return False, "🔴 위험", "영역 제한 도달"
        elif distance >= max_allowed * 0.8:
            return True, "🟡 주의", "영역 제한 근접"
        else:
            return True, "🟢 안전", "정상 범위"
    
    async def emergency_stop_robot(self, reason: str):
        """비상 정지를 수행합니다."""
        print(f"🚨 비상 정지! 이유: {reason}")
        self.emergency_stop = True
        self.is_safe = False
        
        # 즉시 정지 명령 전송
        for i in range(10):  # 0.5초간 정지 명령 반복
            await self.robot.twist_req(Twist(linear=0.0, angular=0.0), NO_PRINT, NO_PRINT)
            await asyncio.sleep(0.05)
        
        print("✅ 비상 정지 완료")
    
    async def smooth_stop(self, current_speed: float) -> None:
        """부드럽게 정지합니다."""
        if self.emergency_stop:
            return  # 비상 정지 중이면 건너뛰기
        
        print("🛑 부드러운 정지 시작...")
        
        num_steps = int(1.0 * self.control_frequency)
        if num_steps == 0:
            await self.robot.twist_req(Twist(linear=0.0, angular=0.0), NO_PRINT, NO_PRINT)
            return
        
        speed_reduction_step = current_speed / num_steps
        
        for i in range(num_steps):
            if self.emergency_stop:
                break  # 비상 정지 중이면 중단
                
            current_speed -= speed_reduction_step
            await self.robot.twist_req(Twist(linear=current_speed, angular=0.0), NO_PRINT, NO_PRINT)
            await asyncio.sleep(1.0 / self.control_frequency)
        
        # 최종 정지
        await self.robot.twist_req(Twist(linear=0.0, angular=0.0), NO_PRINT, NO_PRINT)
        print("✅ 정지 완료")
    
    async def move_with_safety_check(self, linear_speed: float, duration: float, direction: str) -> bool:
        """안전 체크를 포함한 이동을 수행합니다."""
        delay = 1.0 / self.control_frequency
        num_commands = int(duration * self.control_frequency)
        
        print(f"🚀 {direction} 방향으로 {duration}초 동안 이동 시작 (속도: {linear_speed} m/s)")
        self.current_direction = direction
        
        for i in range(num_commands):
            if self.emergency_stop:
                print("🚨 비상 정지로 인한 이동 중단")
                return False
            
            # 현재 위치 확인
            pose_speed, ok, msg = await self.robot.robot_pose_speed_req(
                PoseSpeed(), NO_PRINT, NO_PRINT
            )
            
            if ok:
                # 거리 계산
                distance = self.calculate_distance_from_start(pose_speed.pose.x, pose_speed.pose.y)
                self.max_distance_reached = max(self.max_distance_reached, distance)
                
                # 안전 상태 확인
                is_safe, safety_status, safety_msg = self.check_safety_status(distance)
                
                # 실시간 모니터링 출력 (매 10번째마다)
                if i % 10 == 0:
                    current_time = time.strftime("%H:%M:%S")
                    print(f"   {current_time} | 거리: {distance:.3f}m | 상태: {safety_status}")
                
                # 안전하지 않으면 비상 정지
                if not is_safe:
                    await self.emergency_stop_robot(safety_msg)
                    return False
                
                # 주의 상태일 때 경고
                if safety_status == "🟡 주의":
                    print(f"⚠️ 주의: {safety_msg} (거리: {distance:.3f}m)")
            
            # 속도 명령 전송
            _, ok, msg = await self.robot.twist_req(Twist(linear=linear_speed, angular=0.0), NO_PRINT, NO_PRINT)
            if not ok:
                print(f"❌ 속도 제어 요청 실패: {msg}")
                await self.emergency_stop_robot("속도 제어 실패")
                return False
            
            await asyncio.sleep(delay)
        
        # 이동 완료 후 부드럽게 정지
        if not self.emergency_stop:
            await self.smooth_stop(linear_speed)
        
        return True
    
    async def run_safe_area_motion(self) -> None:
        """안전한 영역 제한 구동을 실행합니다."""
        print("🎯 통합 안전 제어 시스템을 시작합니다!")
        print(f"   최대 이동 거리: {self.max_distance}m")
        print(f"   안전 여유 거리: {self.safety_margin}m")
        print(f"   전진 속도: {self.forward_speed} m/s")
        print(f"   후진 속도: {self.backward_speed} m/s")
        print("   중지하려면 Ctrl+C를 누르세요.")
        print("=" * 60)
        
        try:
            while not self.emergency_stop:
                self.cycle_count += 1
                print(f"\n🔄 사이클 {self.cycle_count} 시작")
                
                # 전진 운동
                print("1️⃣ 전진 운동")
                success = await self.move_with_safety_check(
                    self.forward_speed, self.move_duration, "forward"
                )
                
                if not success or self.emergency_stop:
                    print("❌ 전진 운동 실패 또는 비상 정지, 구동 중단")
                    break
                
                # 대기
                print(f"2️⃣ {self.wait_duration}초 대기")
                await asyncio.sleep(self.wait_duration)
                
                # 후진 운동
                print("3️⃣ 후진 운동")
                success = await self.move_with_safety_check(
                    self.backward_speed, self.move_duration, "backward"
                )
                
                if not success or self.emergency_stop:
                    print("❌ 후진 운동 실패 또는 비상 정지, 구동 중단")
                    break
                
                # 대기
                print(f"4️⃣ {self.wait_duration}초 대기")
                await asyncio.sleep(self.wait_duration)
                
                print(f"✅ 사이클 {self.cycle_count} 완료")
                
        except KeyboardInterrupt:
            print("\n⏹️ 사용자 요청으로 구동 중단")
        except Exception as e:
            print(f"\n❌ 구동 중 오류 발생: {e}")
        finally:
            # 최종 정지
            if not self.emergency_stop:
                await self.smooth_stop(self.forward_speed)
            print(f"🏁 총 {self.cycle_count}개 사이클 완료")
            print(f"📊 최대 도달 거리: {self.max_distance_reached:.3f}m")
    
    async def cleanup(self) -> None:
        """리소스를 정리합니다."""
        print("🔌 시스템 정리 중...")
        
        if self.robot.comm.is_connected():
            await self.robot.stop()
            print("✅ 로봇 연결 종료")
        
        print("✅ 시스템 정리 완료")

async def main():
    """메인 실행 함수"""
    controller = IntegratedSafetyController()
    
    try:
        # 로봇 초기화
        if not await controller.initialize_robot():
            print("❌ 로봇 초기화 실패")
            return
        
        print("✅ 로봇 초기화 완료")
        
        # 안전한 영역 제한 구동 실행
        await controller.run_safe_area_motion()
        
    except KeyboardInterrupt:
        print("\n⏹️ 사용자 요청으로 프로그램을 중지합니다.")
    except Exception as e:
        print(f"\n❌ 프로그램 실행 중 오류 발생: {e}")
    finally:
        await controller.cleanup()

if __name__ == "__main__":
    asyncio.run(main())
