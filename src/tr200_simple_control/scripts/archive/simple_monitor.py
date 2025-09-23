#!/usr/bin/env python3
# simple_monitor.py
# 간단한 실시간 거리 모니터링

import asyncio
import sys
import time

# SDK 경로 추가
sys.path.insert(0, '/catkin_ws/src/woosh_robot_py')

from woosh.proto.robot.robot_pb2 import PoseSpeed
from woosh_interface import CommuSettings, NO_PRINT
from woosh_robot import WooshRobot

async def simple_monitor():
    """간단한 실시간 모니터링"""
    settings = CommuSettings(addr='169.254.128.2', port=5480, identity='simple-monitor')
    robot = WooshRobot(settings)
    
    try:
        if await robot.run():
            print("✅ 모니터링 연결 성공")
            
            # 초기 위치 설정
            pose_speed, ok, msg = await robot.robot_pose_speed_req(PoseSpeed(), NO_PRINT, NO_PRINT)
            if ok:
                start_x = pose_speed.pose.x
                start_y = pose_speed.pose.y
                print(f"📍 시작 위치: X={start_x:.3f}, Y={start_y:.3f}")
                
                max_distance = 0.0
                
                print("\n🔄 실시간 모니터링 시작 (Ctrl+C로 중단)")
                print("시간\t\t거리(m)\t상태")
                print("-" * 40)
                
                while True:
                    pose_speed, ok, msg = await robot.robot_pose_speed_req(PoseSpeed(), NO_PRINT, NO_PRINT)
                    if ok:
                        current_x = pose_speed.pose.x
                        current_y = pose_speed.pose.y
                        
                        # 거리 계산
                        dx = current_x - start_x
                        dy = current_y - start_y
                        distance = (dx**2 + dy**2)**0.5
                        
                        max_distance = max(max_distance, distance)
                        
                        # 상태 결정
                        if distance >= 0.4:  # 0.5 - 0.1 안전거리
                            status = "🔴 위험"
                        elif distance >= 0.32:  # 0.4 * 0.8
                            status = "🟡 주의"
                        else:
                            status = "🟢 안전"
                        
                        # 현재 시간
                        current_time = time.strftime("%H:%M:%S")
                        
                        # 출력
                        print(f"{current_time}\t{distance:.3f}\t{status}")
                        
                        # 위험 상태일 때 경고
                        if distance >= 0.4:
                            print(f"⚠️ 경고: 영역 제한 도달! (거리: {distance:.3f}m)")
                    
                    await asyncio.sleep(0.2)  # 5Hz로 모니터링
                    
        else:
            print("❌ 모니터링 연결 실패")
            
    except KeyboardInterrupt:
        print(f"\n⏹️ 모니터링 종료")
        print(f"📊 최대 도달 거리: {max_distance:.3f}m")
    except Exception as e:
        print(f"❌ 오류: {e}")
    finally:
        if robot.comm.is_connected():
            await robot.stop()

if __name__ == "__main__":
    asyncio.run(simple_monitor())
