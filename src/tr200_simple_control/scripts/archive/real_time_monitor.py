#!/usr/bin/env python3
# real_time_monitor.py
# TR200 로봇의 실시간 위치 및 거리 모니터링

import asyncio
import sys
import os
import time
from datetime import datetime

# SDK 경로 추가
sys.path.insert(0, '/catkin_ws/src/woosh_robot_py')

from woosh.proto.robot.robot_pb2 import PoseSpeed
from woosh_interface import CommuSettings, NO_PRINT
from woosh_robot import WooshRobot

class RealTimeMonitor:
    """실시간 로봇 위치 및 거리 모니터링 클래스"""
    
    def __init__(self):
        self.settings = CommuSettings(addr='169.254.128.2', port=5480, identity='real-time-monitor')
        self.robot = WooshRobot(self.settings)
        
        # 모니터링 설정
        self.start_position = None
        self.max_distance = 0.5  # 영역 제한 거리 (m)
        self.safety_margin = 0.1  # 안전 여유 거리 (m)
        self.monitor_frequency = 5.0  # 모니터링 주파수 (Hz)
        
        # 통계
        self.max_distance_reached = 0.0
        self.total_distance_traveled = 0.0
        self.last_position = None
        
    async def initialize_robot(self) -> bool:
        """로봇을 초기화하고 연결합니다."""
        try:
            print("🔍 모니터링용 로봇 연결을 시작합니다...")
            
            if not await self.robot.run():
                print("❌ 로봇 연결 실패")
                return False
            
            print("✅ 모니터링용 로봇 연결 성공")
            
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
                self.last_position = self.start_position.copy()
                print(f"📍 모니터링 시작 위치: X={self.start_position['x']:.3f}, Y={self.start_position['y']:.3f}")
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
    
    def calculate_distance_traveled(self, current_x: float, current_y: float) -> float:
        """이전 위치로부터 이동한 거리를 계산합니다."""
        if not self.last_position:
            return 0.0
        
        dx = current_x - self.last_position['x']
        dy = current_y - self.last_position['y']
        return (dx**2 + dy**2)**0.5
    
    def get_safety_status(self, distance: float) -> tuple:
        """안전 상태를 반환합니다."""
        max_allowed = self.max_distance - self.safety_margin
        
        if distance >= max_allowed:
            return "🔴 위험", "영역 제한 도달"
        elif distance >= max_allowed * 0.8:
            return "🟡 주의", "영역 제한 근접"
        else:
            return "🟢 안전", "정상 범위"
    
    def print_status_header(self):
        """상태 헤더를 출력합니다."""
        print("\n" + "="*80)
        print("📊 TR200 실시간 위치 및 거리 모니터링")
        print("="*80)
        print(f"{'시간':<12} {'X좌표':<8} {'Y좌표':<8} {'거리':<8} {'상태':<8} {'속도':<8} {'총이동':<8}")
        print("-"*80)
    
    def print_status_line(self, timestamp: str, x: float, y: float, distance: float, 
                         safety_status: str, linear_speed: float, total_traveled: float):
        """상태 라인을 출력합니다."""
        print(f"{timestamp:<12} {x:<8.3f} {y:<8.3f} {distance:<8.3f} {safety_status:<8} {linear_speed:<8.3f} {total_traveled:<8.3f}")
    
    async def monitor_robot(self) -> None:
        """로봇을 실시간으로 모니터링합니다."""
        delay = 1.0 / self.monitor_frequency
        
        self.print_status_header()
        
        try:
            while True:
                # 현재 위치 및 속도 정보 요청
                pose_speed, ok, msg = await self.robot.robot_pose_speed_req(
                    PoseSpeed(), NO_PRINT, NO_PRINT
                )
                
                if ok:
                    current_x = pose_speed.pose.x
                    current_y = pose_speed.pose.y
                    current_theta = pose_speed.pose.theta
                    linear_speed = pose_speed.twist.linear
                    angular_speed = pose_speed.twist.angular
                    
                    # 거리 계산
                    distance_from_start = self.calculate_distance_from_start(current_x, current_y)
                    distance_traveled = self.calculate_distance_traveled(current_x, current_y)
                    
                    # 통계 업데이트
                    self.max_distance_reached = max(self.max_distance_reached, distance_from_start)
                    self.total_distance_traveled += distance_traveled
                    
                    # 안전 상태 확인
                    safety_status, safety_msg = self.get_safety_status(distance_from_start)
                    
                    # 현재 시간
                    timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
                    
                    # 상태 출력
                    self.print_status_line(
                        timestamp, current_x, current_y, distance_from_start,
                        safety_status, linear_speed, self.total_distance_traveled
                    )
                    
                    # 위험 상태일 때 경고
                    if safety_status == "🔴 위험":
                        print(f"⚠️ 경고: {safety_msg} (거리: {distance_from_start:.3f}m)")
                    elif safety_status == "🟡 주의":
                        print(f"⚠️ 주의: {safety_msg} (거리: {distance_from_start:.3f}m)")
                    
                    # 위치 업데이트
                    self.last_position = {'x': current_x, 'y': current_y, 'theta': current_theta}
                    
                else:
                    print(f"❌ 위치 정보 요청 실패: {msg}")
                
                await asyncio.sleep(delay)
                
        except KeyboardInterrupt:
            print("\n⏹️ 모니터링이 중단되었습니다.")
        except Exception as e:
            print(f"\n❌ 모니터링 중 오류 발생: {e}")
    
    def print_summary(self):
        """모니터링 요약을 출력합니다."""
        print("\n" + "="*50)
        print("📈 모니터링 요약")
        print("="*50)
        print(f"최대 도달 거리: {self.max_distance_reached:.3f}m")
        print(f"총 이동 거리: {self.total_distance_traveled:.3f}m")
        print(f"영역 제한: {self.max_distance}m")
        print(f"안전 여유: {self.safety_margin}m")
        
        if self.max_distance_reached >= self.max_distance - self.safety_margin:
            print("🔴 영역 제한에 도달했습니다!")
        elif self.max_distance_reached >= (self.max_distance - self.safety_margin) * 0.8:
            print("🟡 영역 제한에 근접했습니다.")
        else:
            print("🟢 안전한 범위 내에서 운행되었습니다.")
    
    async def cleanup(self) -> None:
        """리소스를 정리합니다."""
        print("\n🔌 모니터링 시스템 정리 중...")
        
        if self.robot.comm.is_connected():
            await self.robot.stop()
            print("✅ 모니터링용 로봇 연결 종료")
        
        print("✅ 모니터링 시스템 정리 완료")

async def main():
    """메인 실행 함수"""
    monitor = RealTimeMonitor()
    
    try:
        # 로봇 초기화
        if not await monitor.initialize_robot():
            print("❌ 모니터링용 로봇 초기화 실패")
            return
        
        print("✅ 모니터링용 로봇 초기화 완료")
        print("🚀 실시간 모니터링을 시작합니다...")
        print("   중지하려면 Ctrl+C를 누르세요.")
        
        # 실시간 모니터링 실행
        await monitor.monitor_robot()
        
    except KeyboardInterrupt:
        print("\n⏹️ 사용자 요청으로 모니터링을 중지합니다.")
    except Exception as e:
        print(f"\n❌ 모니터링 실행 중 오류 발생: {e}")
    finally:
        monitor.print_summary()
        await monitor.cleanup()

if __name__ == "__main__":
    asyncio.run(main())
