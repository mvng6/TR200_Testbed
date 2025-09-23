#!/usr/bin/env python3
# bypass_taskable_test.py
# kTaskable 상태 체크를 우회하는 영역 제한 구동 테스트

import asyncio
import sys
import os
import yaml
from datetime import datetime
from typing import Dict, Any, Optional, Tuple

# SDK 경로 추가
sys.path.insert(0, '/catkin_ws/src/woosh_robot_py')

from woosh.proto.robot.robot_pb2 import PoseSpeed, OperationState
from woosh.proto.robot.robot_pack_pb2 import Twist
from woosh_interface import CommuSettings, NO_PRINT
from woosh_robot import WooshRobot

class SimpleAreaMotionController:
    """kTaskable 상태 체크를 우회하는 간단한 영역 제한 구동 컨트롤러"""
    
    def __init__(self):
        self.settings = CommuSettings(addr='169.254.128.2', port=5480, identity='bypass-test')
        self.robot = WooshRobot(self.settings)
        
        # 영역 제한 설정
        self.max_distance = 0.5  # 최대 이동 거리 (m)
        self.safety_margin = 0.1  # 안전 여유 거리 (m)
        self.start_position = None
        self.control_frequency = 20.0
        
        # 구동 설정
        self.forward_speed = 0.15  # 전진 속도 (m/s)
        self.backward_speed = -0.15  # 후진 속도 (m/s)
        self.move_duration = 2.0  # 한 방향 이동 시간 (초)
        self.wait_duration = 1.0  # 대기 시간 (초)
    
    async def initialize_robot(self) -> bool:
        """로봇을 초기화하고 연결합니다."""
        try:
            print("🔍 로봇 연결을 시작합니다...")
            
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
                print(f"📍 시작 위치 설정: X={self.start_position['x']:.3f}, Y={self.start_position['y']:.3f}")
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
    
    def check_area_constraint(self, direction: str, current_x: float, current_y: float) -> Tuple[bool, str]:
        """영역 제한을 확인합니다."""
        distance = self.calculate_distance_from_start(current_x, current_y)
        max_allowed = self.max_distance - self.safety_margin
        
        if distance >= max_allowed:
            return False, f"영역 제한 도달 (거리: {distance:.3f}m >= {max_allowed:.3f}m)"
        
        return True, f"영역 내 이동 가능 (거리: {distance:.3f}m < {max_allowed:.3f}m)"
    
    async def smooth_stop(self, current_speed: float) -> None:
        """부드럽게 정지합니다."""
        print("🛑 부드러운 정지 시작...")
        
        num_steps = int(1.0 * self.control_frequency)  # 1초간 감속
        if num_steps == 0:
            await self.robot.twist_req(Twist(linear=0.0, angular=0.0), NO_PRINT, NO_PRINT)
            return
        
        speed_reduction_step = current_speed / num_steps
        
        for i in range(num_steps):
            current_speed -= speed_reduction_step
            await self.robot.twist_req(Twist(linear=current_speed, angular=0.0), NO_PRINT, NO_PRINT)
            await asyncio.sleep(1.0 / self.control_frequency)
        
        # 최종 정지
        await self.robot.twist_req(Twist(linear=0.0, angular=0.0), NO_PRINT, NO_PRINT)
        print("✅ 정지 완료")
    
    async def move_with_area_constraint(self, linear_speed: float, duration: float, direction: str) -> bool:
        """영역 제한을 고려하여 이동합니다."""
        delay = 1.0 / self.control_frequency
        num_commands = int(duration * self.control_frequency)
        
        print(f"🚀 {direction} 방향으로 {duration}초 동안 이동 시작 (속도: {linear_speed} m/s)")
        
        for i in range(num_commands):
            # 현재 위치 확인
            pose_speed, ok, msg = await self.robot.robot_pose_speed_req(
                PoseSpeed(), NO_PRINT, NO_PRINT
            )
            
            if ok:
                # 영역 제한 확인
                constraint_ok, constraint_msg = self.check_area_constraint(
                    direction, pose_speed.pose.x, pose_speed.pose.y
                )
                
                if not constraint_ok:
                    print(f"⚠️ {constraint_msg}")
                    await self.smooth_stop(linear_speed)
                    return False
                
                # 진행 상황 출력 (매 10번째마다)
                if i % 10 == 0:
                    distance = self.calculate_distance_from_start(pose_speed.pose.x, pose_speed.pose.y)
                    print(f"   진행: {i}/{num_commands}, 거리: {distance:.3f}m")
            
            # 속도 명령 전송
            _, ok, msg = await self.robot.twist_req(Twist(linear=linear_speed, angular=0.0), NO_PRINT, NO_PRINT)
            if not ok:
                print(f"❌ 속도 제어 요청 실패: {msg}")
                await self.smooth_stop(linear_speed)
                return False
            
            await asyncio.sleep(delay)
        
        # 이동 완료 후 부드럽게 정지
        await self.smooth_stop(linear_speed)
        return True
    
    async def run_area_motion(self) -> None:
        """영역 제한 구동을 실행합니다."""
        print("🎯 영역 제한 구동을 시작합니다!")
        print(f"   최대 이동 거리: {self.max_distance}m")
        print(f"   안전 여유 거리: {self.safety_margin}m")
        print(f"   전진 속도: {self.forward_speed} m/s")
        print(f"   후진 속도: {self.backward_speed} m/s")
        print("   중지하려면 Ctrl+C를 누르세요.")
        print("=" * 50)
        
        cycle_count = 0
        
        try:
            while True:
                cycle_count += 1
                print(f"\n🔄 사이클 {cycle_count} 시작")
                
                # 전진 운동
                print("1️⃣ 전진 운동")
                success = await self.move_with_area_constraint(
                    self.forward_speed, self.move_duration, "forward"
                )
                
                if not success:
                    print("❌ 전진 운동 실패, 구동 중단")
                    break
                
                # 대기
                print(f"2️⃣ {self.wait_duration}초 대기")
                await asyncio.sleep(self.wait_duration)
                
                # 후진 운동
                print("3️⃣ 후진 운동")
                success = await self.move_with_area_constraint(
                    self.backward_speed, self.move_duration, "backward"
                )
                
                if not success:
                    print("❌ 후진 운동 실패, 구동 중단")
                    break
                
                # 대기
                print(f"4️⃣ {self.wait_duration}초 대기")
                await asyncio.sleep(self.wait_duration)
                
                print(f"✅ 사이클 {cycle_count} 완료")
                
        except KeyboardInterrupt:
            print("\n⏹️ 사용자 요청으로 구동 중단")
        except Exception as e:
            print(f"\n❌ 구동 중 오류 발생: {e}")
        finally:
            # 최종 정지
            await self.smooth_stop(self.forward_speed)
            print(f"🏁 총 {cycle_count}개 사이클 완료")
    
    async def cleanup(self) -> None:
        """리소스를 정리합니다."""
        print("🔌 시스템 정리 중...")
        
        if self.robot.comm.is_connected():
            await self.robot.stop()
            print("✅ 로봇 연결 종료")
        
        print("✅ 시스템 정리 완료")

async def main():
    """메인 실행 함수"""
    controller = SimpleAreaMotionController()
    
    try:
        # 로봇 초기화
        if not await controller.initialize_robot():
            print("❌ 로봇 초기화 실패")
            return
        
        print("✅ 로봇 초기화 완료")
        
        # 영역 제한 구동 실행
        await controller.run_area_motion()
        
    except KeyboardInterrupt:
        print("\n⏹️ 사용자 요청으로 프로그램을 중지합니다.")
    except Exception as e:
        print(f"\n❌ 프로그램 실행 중 오류 발생: {e}")
    finally:
        await controller.cleanup()

if __name__ == "__main__":
    asyncio.run(main())
