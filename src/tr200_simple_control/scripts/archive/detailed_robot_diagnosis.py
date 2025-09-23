#!/usr/bin/env python3
# detailed_robot_diagnosis.py
# TR200 로봇 상세 진단 스크립트

import asyncio
import sys
import os

# SDK 경로 추가
sys.path.insert(0, '/catkin_ws/src/woosh_robot_py')

from woosh.proto.robot.robot_pb2 import OperationState, PoseSpeed
from woosh_interface import CommuSettings, NO_PRINT
from woosh_robot import WooshRobot

async def detailed_diagnosis():
    """로봇에 대한 상세 진단을 수행합니다."""
    settings = CommuSettings(addr='169.254.128.2', port=5480, identity='detailed-diagnosis')
    robot = WooshRobot(settings)
    
    try:
        print("🔍 TR200 로봇 상세 진단을 시작합니다...")
        print("=" * 50)
        
        if await robot.run():
            print('✅ 로봇 연결 성공')
            
            # 1. 운행 상태 상세 확인
            print("\n📊 1. 운행 상태 상세 분석")
            print("-" * 30)
            state, ok, msg = await robot.robot_operation_state_req(OperationState(), NO_PRINT, NO_PRINT)
            if ok:
                print(f"   Robot bits: {state.robot} (16진수: 0x{state.robot:08x})")
                print(f"   Nav bits: {state.nav} (16진수: 0x{state.nav:08x})")
                
                # RobotBit 값들 분석
                print("\n   RobotBit 분석:")
                robot_bit_names = [
                    'kTaskable', 'kEmergencyStop', 'kCharging', 'kLowBattery',
                    'kError', 'kWarning', 'kMaintenance', 'kCalibration'
                ]
                
                for bit_name in robot_bit_names:
                    if hasattr(OperationState.RobotBit, bit_name):
                        bit_value = getattr(OperationState.RobotBit, bit_name)
                        is_set = bool(state.robot & bit_value)
                        status = "✅ 설정됨" if is_set else "❌ 설정 안됨"
                        print(f"     {bit_name}: {status} (값: {bit_value})")
                
                # NavBit 값들 분석
                print("\n   NavBit 분석:")
                nav_bit_names = [
                    'kImpede', 'kPathBlocked', 'kGoalReached', 'kPathFound',
                    'kLocalized', 'kMapLoaded', 'kNavigationActive'
                ]
                
                for bit_name in nav_bit_names:
                    if hasattr(OperationState.NavBit, bit_name):
                        bit_value = getattr(OperationState.NavBit, bit_name)
                        is_set = bool(state.nav & bit_value)
                        status = "✅ 설정됨" if is_set else "❌ 설정 안됨"
                        print(f"     {bit_name}: {status} (값: {bit_value})")
                        
            else:
                print(f"❌ 상태 요청 실패: {msg}")
            
            # 2. 위치 정보 확인
            print("\n📍 2. 위치 정보 확인")
            print("-" * 30)
            pose_speed, ok, msg = await robot.robot_pose_speed_req(PoseSpeed(), NO_PRINT, NO_PRINT)
            if ok:
                print(f"   현재 위치: X={pose_speed.pose.x:.3f}, Y={pose_speed.pose.y:.3f}, Theta={pose_speed.pose.theta:.3f}")
                print(f"   현재 속도: Linear={pose_speed.twist.linear:.3f}, Angular={pose_speed.twist.angular:.3f}")
                print(f"   맵 ID: {pose_speed.map_id}")
                print(f"   주행거리: {pose_speed.mileage:.3f}m")
            else:
                print(f"❌ 위치 정보 요청 실패: {msg}")
            
            # 3. 권장 해결 방법
            print("\n🔧 3. 권장 해결 방법")
            print("-" * 30)
            if state.robot == 0:
                print("   Robot bits가 0인 경우:")
                print("   1. TR200 앱을 실행하세요")
                print("   2. 로봇에 연결하세요 (169.254.128.2)")
                print("   3. 로봇의 비상정지 버튼을 해제하세요")
                print("   4. TR200 앱에서 '작업 모드' 또는 '자동 모드'로 설정하세요")
                print("   5. 로봇이 완전히 초기화될 때까지 기다리세요")
                print("\n   ⚠️ 중요: TR200 앱이 실행 중이어야 로봇이 활성화됩니다!")
            
            if pose_speed.map_id == 0:
                print("\n   맵 ID가 0인 경우:")
                print("   1. TR200 앱에서 맵을 로드하세요")
                print("   2. 로봇의 위치를 설정하세요")
                print("   3. 로컬라이제이션이 완료될 때까지 기다리세요")
                
        else:
            print('❌ 로봇 연결 실패')
            print("   해결 방법:")
            print("   1. TR200 로봇이 켜져 있는지 확인")
            print("   2. 네트워크 연결 상태 확인 (ping 169.254.128.2)")
            print("   3. TR200 앱이 실행 중인지 확인")
            
    except Exception as e:
        print(f'❌ 진단 중 오류 발생: {e}')
    finally:
        if robot.comm.is_connected():
            await robot.stop()
            print("\n🔌 로봇 연결 종료")

if __name__ == "__main__":
    asyncio.run(detailed_diagnosis())
