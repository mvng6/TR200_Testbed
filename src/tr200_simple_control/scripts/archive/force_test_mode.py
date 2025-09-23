#!/usr/bin/env python3
# force_test_mode.py
# TR200 앱 없이 강제 테스트 모드

import asyncio
import sys
import os

# SDK 경로 추가
sys.path.insert(0, '/catkin_ws/src/woosh_robot_py')

from woosh.proto.robot.robot_pack_pb2 import Twist
from woosh_interface import CommuSettings, NO_PRINT
from woosh_robot import WooshRobot

async def force_test_mode():
    """TR200 앱 없이 강제로 테스트를 수행합니다."""
    settings = CommuSettings(addr='169.254.128.2', port=5480, identity='force-test')
    robot = WooshRobot(settings)
    
    try:
        print("⚠️ 강제 테스트 모드를 시작합니다...")
        print("⚠️ 주의: 이 모드는 TR200 앱 없이 실행됩니다!")
        print("⚠️ 로봇이 예상치 못하게 움직일 수 있습니다!")
        print("=" * 50)
        
        # 사용자 확인
        response = input("계속하시겠습니까? (y/N): ")
        if response.lower() != 'y':
            print("테스트를 취소합니다.")
            return
        
        if await robot.run():
            print('✅ 로봇 연결 성공')
            
            # 매우 짧고 안전한 테스트
            print("\n🚀 매우 짧은 테스트를 시작합니다...")
            
            # 1. 매우 느린 전진 (0.1m/s, 1초)
            print("1. 0.1m/s로 1초 전진...")
            twist_cmd = Twist(linear=0.1, angular=0.0)
            for i in range(20):  # 1초 (20Hz)
                await robot.twist_req(twist_cmd, NO_PRINT, NO_PRINT)
                await asyncio.sleep(0.05)
            
            # 2. 정지
            print("2. 정지...")
            stop_cmd = Twist(linear=0.0, angular=0.0)
            for i in range(10):  # 0.5초
                await robot.twist_req(stop_cmd, NO_PRINT, NO_PRINT)
                await asyncio.sleep(0.05)
            
            # 3. 매우 느린 후진 (0.1m/s, 1초)
            print("3. 0.1m/s로 1초 후진...")
            twist_cmd = Twist(linear=-0.1, angular=0.0)
            for i in range(20):  # 1초 (20Hz)
                await robot.twist_req(twist_cmd, NO_PRINT, NO_PRINT)
                await asyncio.sleep(0.05)
            
            # 4. 최종 정지
            print("4. 최종 정지...")
            for i in range(20):  # 1초
                await robot.twist_req(stop_cmd, NO_PRINT, NO_PRINT)
                await asyncio.sleep(0.05)
            
            print("✅ 강제 테스트 완료!")
            print("⚠️ 로봇이 정상적으로 움직였다면 SDK 연결은 정상입니다.")
            print("⚠️ TR200 앱을 사용하면 더 안전하고 정확한 제어가 가능합니다.")
            
        else:
            print('❌ 로봇 연결 실패')
            
    except KeyboardInterrupt:
        print("\n⏹️ 사용자에 의해 테스트가 중단되었습니다.")
        # 비상 정지
        stop_cmd = Twist(linear=0.0, angular=0.0)
        for i in range(10):
            await robot.twist_req(stop_cmd, NO_PRINT, NO_PRINT)
            await asyncio.sleep(0.05)
    except Exception as e:
        print(f'❌ 테스트 중 오류 발생: {e}')
    finally:
        if robot.comm.is_connected():
            await robot.stop()
            print("🔌 로봇 연결 종료")

if __name__ == "__main__":
    asyncio.run(force_test_mode())
