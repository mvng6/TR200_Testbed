# simple_linear_motion.py

import sys
import asyncio

# --- 필요한 Protobuf 및 SDK 클래스만 임포트 ---
from woosh.proto.robot.robot_pb2 import PoseSpeed, OperationState
from woosh.proto.robot.robot_pack_pb2 import Twist
from woosh_interface import CommuSettings, NO_PRINT
from woosh_robot import WooshRobot

# ==============================================================================
# 							    사용자 설정 (User Settings)
# ==============================================================================
# 로봇 연결 정보
ROBOT_ADDR = "169.254.128.2"  # TR200 로봇의 IP 주소
ROBOT_PORT = 5480             # TR200 로봇의 포트

# 로봇 움직임 파라미터
FORWARD_SPEED_MPS = 0.5       # 전진 속도 (m/s), 예: 0.3m/s
BACKWARD_SPEED_MPS = -0.3     # 후진 속도 (m/s), 음수 값으로 설정
MOVE_DURATION_SEC = 5.0       # 한 방향으로 움직이는 시간 (초)
WAIT_DURATION_SEC = 3.0       # 전진/후진 후 대기하는 시간 (초)

# 제어 주기 설정
CONTROL_HERTZ = 20            # 1초에 몇 번이나 속도 명령을 보낼지 결정 (20Hz 권장)
# ==============================================================================

async def smooth_stop(robot: WooshRobot, current_linear: float, stop_duration: float = 1.0):
    """로봇을 부드럽게 감속하여 정지시키는 함수."""
    print("...부드럽게 정지 중...")
    num_steps = int(stop_duration * CONTROL_HERTZ)
    if num_steps == 0:
        await robot.twist_req(Twist(linear=0.0, angular=0.0), NO_PRINT, NO_PRINT)
        return

    speed_reduction_step = current_linear / num_steps
    
    for _ in range(num_steps):
        current_linear -= speed_reduction_step
        await robot.twist_req(Twist(linear=current_linear, angular=0.0), NO_PRINT, NO_PRINT)
        await asyncio.sleep(1.0 / CONTROL_HERTZ)

    # 안전을 위해 마지막에 정지 명령을 한 번 더 전송
    await robot.twist_req(Twist(linear=0.0, angular=0.0), NO_PRINT, NO_PRINT)
    print("✅ 정지 완료.")

async def move_robot(robot: WooshRobot, linear_speed: float, duration: float):
    """지정된 시간 동안 특정 선속도로 로봇을 움직이는 함수."""
    delay = 1.0 / CONTROL_HERTZ
    twist_cmd = Twist(linear=linear_speed, angular=0.0)
    
    num_commands = int(duration * CONTROL_HERTZ)
    print(f"🚀 {duration}초 동안 속도 {linear_speed} m/s 로 이동 시작...")
    
    for i in range(num_commands):
        _, ok, msg = await robot.twist_req(twist_cmd, NO_PRINT, NO_PRINT)
        if not ok:
            print(f"❌ 속도 제어 요청 실패: {msg}")
            # 실패 시에도 루프를 계속 진행할 수 있으나, 여기서 멈추는 것이 안전할 수 있음
            break
        await asyncio.sleep(delay)
        
    # 이동 완료 후 부드럽게 정지
    await smooth_stop(robot, linear_speed)


async def main():
    """메인 비동기 실행 함수."""
    settings = CommuSettings(addr=ROBOT_ADDR, port=ROBOT_PORT, identity="woosdk-linear-motion")
    robot = WooshRobot(settings)

    try:
        # --- 1. 로봇 연결 ---
        if not await robot.run():
            print(f"❌ 로봇 연결 실패 ({ROBOT_ADDR}:{ROBOT_PORT}). IP 주소와 네트워크를 확인하세요.")
            return
        print(f"✅ 로봇에 성공적으로 연결되었습니다 ({ROBOT_ADDR}:{ROBOT_PORT}).")
        
        # --- 2. 로봇 상태 확인 ---
        state, ok, msg = await robot.robot_operation_state_req(OperationState(), NO_PRINT, NO_PRINT)
        if ok and (state.robot & OperationState.RobotBit.kTaskable):
            print("   -> 🟢 로봇이 작업을 받을 수 있는 'kTaskable' 상태입니다.")
        else:
            print(f"   -> 🔴 로봇이 'kTaskable' 상태가 아닙니다. (요청 실패: {not ok}, 메시지: {msg})")
            print("   ->   로봇의 비상정지 버튼이나 다른 상태를 확인해주세요. 5초 후 프로그램을 종료합니다.")
            await asyncio.sleep(5)
            return

        # --- 3. 왕복 운동 무한 루프 ---
        print("\n\n*** 단순 왕복 운동을 시작합니다. 중지하려면 Ctrl+C를 누르세요. ***\n")
        while True:
            # 전진 운동
            await move_robot(robot, FORWARD_SPEED_MPS, MOVE_DURATION_SEC)
            
            # 대기
            print(f"🕒 {WAIT_DURATION_SEC}초 동안 대기합니다.")
            await asyncio.sleep(WAIT_DURATION_SEC)
            
            # 후진 운동
            await move_robot(robot, BACKWARD_SPEED_MPS, MOVE_DURATION_SEC)

            # 대기
            print(f"🕒 {WAIT_DURATION_SEC}초 동안 대기합니다.")
            await asyncio.sleep(WAIT_DURATION_SEC)

    except KeyboardInterrupt:
        print("\n\n⏹️ 사용자 요청으로 프로그램을 중지합니다. 로봇을 안전하게 정지시킵니다.")
    except Exception as e:
        print(f"\n❌ 프로그램 실행 중 오류 발생: {e}")
    finally:
        # --- 4. 종료 처리 ---
        if robot.comm.is_connected():
            print("🔌 로봇 연결을 종료합니다...")
            # 어떤 상황에서든 마지막에는 반드시 정지 명령을 보냅니다.
            await smooth_stop(robot, FORWARD_SPEED_MPS)
            await robot.stop()
            print("   -> 로봇 연결이 안전하게 종료되었습니다.")

if __name__ == "__main__":
    asyncio.run(main())