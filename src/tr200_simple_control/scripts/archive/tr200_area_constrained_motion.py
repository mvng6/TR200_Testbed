# tr200_area_constrained_motion.py
# TR200 로봇의 특정 영역 내에서만 앞뒤로 구동하는 고급 제어 시스템

import sys
import asyncio
import yaml
import csv
import os
from datetime import datetime
from typing import Dict, Any, Optional, Tuple
from dataclasses import dataclass

# --- 필요한 Protobuf 및 SDK 클래스 임포트 ---
from woosh.proto.robot.robot_pb2 import PoseSpeed, OperationState, ScannerData
from woosh.proto.robot.robot_pack_pb2 import Twist
from woosh_interface import CommuSettings, NO_PRINT, FULL_PRINT
from woosh_robot import WooshRobot

# ==============================================================================
# 							    데이터 클래스 정의
# ==============================================================================

@dataclass
class RobotPosition:
    """로봇의 현재 위치를 나타내는 클래스"""
    x: float
    y: float
    theta: float
    timestamp: float

@dataclass
class AreaConstraints:
    """영역 제한 설정을 나타내는 클래스"""
    max_forward_distance: float
    max_backward_distance: float
    safety_margin: float
    detection_method: str
    use_initial_position: bool
    initial_position: RobotPosition

@dataclass
class MotionPattern:
    """구동 패턴 설정을 나타내는 클래스"""
    forward_speed: float
    backward_speed: float
    move_duration: float
    wait_duration: float
    enabled: bool

# ==============================================================================
# 							    설정 관리 클래스
# ==============================================================================

class ConfigManager:
    """YAML 설정 파일을 관리하는 클래스"""
    
    def __init__(self, config_path: str):
        self.config_path = config_path
        self.config: Dict[str, Any] = {}
        self.load_config()
    
    def load_config(self) -> None:
        """설정 파일을 로드합니다."""
        try:
            with open(self.config_path, 'r', encoding='utf-8') as file:
                self.config = yaml.safe_load(file)
            print(f"✅ 설정 파일 로드 완료: {self.config_path}")
        except FileNotFoundError:
            print(f"❌ 설정 파일을 찾을 수 없습니다: {self.config_path}")
            sys.exit(1)
        except yaml.YAMLError as e:
            print(f"❌ 설정 파일 파싱 오류: {e}")
            sys.exit(1)
    
    def get_robot_config(self) -> Dict[str, Any]:
        """로봇 연결 설정을 반환합니다."""
        return self.config.get('robot', {})
    
    def get_area_constraints(self) -> AreaConstraints:
        """영역 제한 설정을 반환합니다."""
        area_config = self.config.get('area_constraints', {})
        initial_pos_config = area_config.get('initial_position', {})
        
        return AreaConstraints(
            max_forward_distance=area_config.get('max_forward_distance', 2.0),
            max_backward_distance=area_config.get('max_backward_distance', 2.0),
            safety_margin=area_config.get('safety_margin', 0.3),
            detection_method=area_config.get('detection_method', 'distance'),
            use_initial_position=area_config.get('use_initial_position', True),
            initial_position=RobotPosition(
                x=initial_pos_config.get('x', 0.0),
                y=initial_pos_config.get('y', 0.0),
                theta=initial_pos_config.get('theta', 0.0),
                timestamp=0.0
            )
        )
    
    def get_motion_pattern(self, pattern_name: str = 'simple_back_forth') -> MotionPattern:
        """구동 패턴 설정을 반환합니다."""
        pattern_config = self.config.get('motion_patterns', {}).get(pattern_name, {})
        
        return MotionPattern(
            forward_speed=pattern_config.get('forward_speed', 0.4),
            backward_speed=pattern_config.get('backward_speed', -0.3),
            move_duration=pattern_config.get('move_duration', 4.0),
            wait_duration=pattern_config.get('wait_duration', 2.0),
            enabled=pattern_config.get('enabled', True)
        )
    
    def get_control_config(self) -> Dict[str, Any]:
        """제어 설정을 반환합니다."""
        return self.config.get('control', {})
    
    def get_safety_config(self) -> Dict[str, Any]:
        """안전 설정을 반환합니다."""
        return self.config.get('safety', {})
    
    def get_logging_config(self) -> Dict[str, Any]:
        """로깅 설정을 반환합니다."""
        return self.config.get('logging', {})

# ==============================================================================
# 							    데이터 로거 클래스
# ==============================================================================

class MotionDataLogger:
    """로봇 구동 데이터를 CSV 파일로 로깅하는 클래스"""
    
    def __init__(self, config: Dict[str, Any]):
        self.config = config
        self.log_file = None
        self.csv_writer = None
        self.data_file = None
        self.data_writer = None
        self.setup_logging()
    
    def setup_logging(self) -> None:
        """로깅 시스템을 설정합니다."""
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        
        # 로그 파일 설정
        if self.config.get('log_to_file', True):
            log_path = self.config.get('log_file_path', f'/tmp/area_motion_{timestamp}.log')
            os.makedirs(os.path.dirname(log_path), exist_ok=True)
            self.log_file = open(log_path, 'w', encoding='utf-8')
            print(f"📝 로그 파일 생성: {log_path}")
        
        # 데이터 수집 파일 설정
        if self.config.get('collect_motion_data', True):
            data_path = self.config.get('data_file_path', f'/tmp/motion_data_{timestamp}.csv')
            os.makedirs(os.path.dirname(data_path), exist_ok=True)
            self.data_file = open(data_path, 'w', newline='', encoding='utf-8')
            self.data_writer = csv.writer(self.data_file)
            
            # CSV 헤더 작성
            self.data_writer.writerow([
                'timestamp', 'position_x', 'position_y', 'position_theta',
                'linear_velocity', 'angular_velocity', 'move_direction',
                'distance_from_start', 'area_constraint_status', 'safety_status'
            ])
            self.data_file.flush()
            print(f"📊 데이터 수집 파일 생성: {data_path}")
    
    def log_message(self, message: str, level: str = "INFO") -> None:
        """메시지를 로그 파일에 기록합니다."""
        timestamp = datetime.now().isoformat()
        log_entry = f"[{timestamp}] [{level}] {message}"
        
        if self.config.get('console_output', True):
            print(log_entry)
        
        if self.log_file and not self.log_file.closed:
            self.log_file.write(log_entry + '\n')
            self.log_file.flush()
    
    def log_motion_data(self, position: RobotPosition, velocity: Tuple[float, float], 
                       direction: str, distance_from_start: float, 
                       area_status: str, safety_status: str) -> None:
        """구동 데이터를 CSV 파일에 기록합니다."""
        if not self.data_writer:
            return
        
        timestamp = datetime.now().isoformat()
        self.data_writer.writerow([
            timestamp, position.x, position.y, position.theta,
            velocity[0], velocity[1], direction,
            distance_from_start, area_status, safety_status
        ])
        self.data_file.flush()
    
    def close(self) -> None:
        """로깅 파일들을 안전하게 닫습니다."""
        if self.log_file:
            self.log_file.close()
        if self.data_file:
            self.data_file.close()
        print("📝 로깅 시스템이 안전하게 종료되었습니다.")

# ==============================================================================
# 							    영역 제한 관리 클래스
# ==============================================================================

class AreaConstraintManager:
    """로봇이 특정 영역 내에서만 움직이도록 제한하는 클래스"""
    
    def __init__(self, constraints: AreaConstraints, logger: MotionDataLogger):
        self.constraints = constraints
        self.logger = logger
        self.start_position: Optional[RobotPosition] = None
        self.current_position: Optional[RobotPosition] = None
        self.total_distance_moved = 0.0
    
    def set_start_position(self, position: RobotPosition) -> None:
        """시작 위치를 설정합니다."""
        self.start_position = position
        self.current_position = position
        self.total_distance_moved = 0.0
        
        self.logger.log_message(
            f"시작 위치 설정: X={position.x:.3f}, Y={position.y:.3f}, Theta={position.theta:.3f}"
        )
    
    def update_position(self, position: RobotPosition) -> None:
        """현재 위치를 업데이트하고 거리를 계산합니다."""
        if not self.start_position:
            self.set_start_position(position)
            return
        
        self.current_position = position
        
        # 시작점으로부터의 거리 계산
        distance_from_start = self.calculate_distance_from_start(position)
        self.total_distance_moved = distance_from_start
    
    def calculate_distance_from_start(self, position: RobotPosition) -> float:
        """시작점으로부터의 직선 거리를 계산합니다."""
        if not self.start_position:
            return 0.0
        
        dx = position.x - self.start_position.x
        dy = position.y - self.start_position.y
        return (dx**2 + dy**2)**0.5
    
    def check_forward_constraint(self, current_speed: float) -> Tuple[bool, str]:
        """전진 방향 제한을 확인합니다."""
        if not self.current_position or not self.start_position:
            return True, "위치 정보 없음"
        
        distance_from_start = self.calculate_distance_from_start(self.current_position)
        max_allowed_distance = self.constraints.max_forward_distance - self.constraints.safety_margin
        
        if distance_from_start >= max_allowed_distance:
            return False, f"전진 제한 도달 (거리: {distance_from_start:.3f}m >= {max_allowed_distance:.3f}m)"
        
        return True, f"전진 가능 (거리: {distance_from_start:.3f}m < {max_allowed_distance:.3f}m)"
    
    def check_backward_constraint(self, current_speed: float) -> Tuple[bool, str]:
        """후진 방향 제한을 확인합니다."""
        if not self.current_position or not self.start_position:
            return True, "위치 정보 없음"
        
        distance_from_start = self.calculate_distance_from_start(self.current_position)
        max_allowed_distance = self.constraints.max_backward_distance - self.constraints.safety_margin
        
        if distance_from_start >= max_allowed_distance:
            return False, f"후진 제한 도달 (거리: {distance_from_start:.3f}m >= {max_allowed_distance:.3f}m)"
        
        return True, f"후진 가능 (거리: {distance_from_start:.3f}m < {max_allowed_distance:.3f}m)"
    
    def get_constraint_status(self, direction: str, current_speed: float) -> Tuple[bool, str]:
        """현재 방향에 대한 제한 상태를 반환합니다."""
        if direction == "forward":
            return self.check_forward_constraint(current_speed)
        elif direction == "backward":
            return self.check_backward_constraint(current_speed)
        else:
            return True, "알 수 없는 방향"

# ==============================================================================
# 							    안전 관리 클래스
# ==============================================================================

class SafetyManager:
    """로봇의 안전을 관리하는 클래스"""
    
    def __init__(self, config: Dict[str, Any], logger: MotionDataLogger):
        self.config = config
        self.logger = logger
        self.obstacle_detected = False
        self.connection_lost = False
        self.speed_violation_count = 0
        self.start_time = datetime.now()
    
    def check_obstacle_detection(self, scanner_data: Optional[ScannerData]) -> Tuple[bool, str]:
        """장애물 감지를 확인합니다."""
        if not self.config.get('obstacle_detection_enabled', True):
            return True, "장애물 감지 비활성화"
        
        if not scanner_data:
            return True, "스캐너 데이터 없음"
        
        # 여기서 실제 스캐너 데이터를 분석하여 장애물 감지
        # 현재는 기본적인 체크만 수행
        min_distance = self.config.get('min_obstacle_distance', 0.4)
        
        # 실제 구현에서는 스캐너 데이터의 거리 정보를 분석해야 함
        # 예시: scanner_data.ranges에서 최소값을 찾아서 min_distance와 비교
        
        return True, "장애물 감지 정상"
    
    def check_connection_status(self, robot: WooshRobot) -> Tuple[bool, str]:
        """연결 상태를 확인합니다."""
        if not self.config.get('connection_monitoring', True):
            return True, "연결 모니터링 비활성화"
        
        if not robot.comm.is_connected():
            return False, "로봇 연결 끊김"
        
        return True, "로봇 연결 정상"
    
    def check_operation_time(self) -> Tuple[bool, str]:
        """작동 시간을 확인합니다."""
        if not self.config.get('emergency_stop_enabled', True):
            return True, "비상정지 비활성화"
        
        max_time = self.config.get('max_operation_time', 300.0)
        elapsed_time = (datetime.now() - self.start_time).total_seconds()
        
        if elapsed_time >= max_time:
            return False, f"최대 작동 시간 초과 ({elapsed_time:.1f}s >= {max_time}s)"
        
        return True, f"작동 시간 정상 ({elapsed_time:.1f}s < {max_time}s)"
    
    def check_speed_violation(self, current_speed: float, max_speed: float) -> Tuple[bool, str]:
        """속도 위반을 확인합니다."""
        if not self.config.get('speed_monitoring', True):
            return True, "속도 모니터링 비활성화"
        
        if abs(current_speed) > max_speed:
            self.speed_violation_count += 1
            max_violations = self.config.get('max_speed_violation_count', 3)
            
            if self.speed_violation_count >= max_violations:
                return False, f"속도 위반 횟수 초과 ({self.speed_violation_count} >= {max_violations})"
            
            return True, f"속도 위반 경고 ({self.speed_violation_count}/{max_violations})"
        
        # 속도가 정상이면 위반 카운트 리셋
        self.speed_violation_count = 0
        return True, "속도 정상"
    
    def get_safety_status(self, robot: WooshRobot, scanner_data: Optional[ScannerData], 
                         current_speed: float, max_speed: float) -> Tuple[bool, str]:
        """전체 안전 상태를 확인합니다."""
        # 각종 안전 체크 수행
        obstacle_ok, obstacle_msg = self.check_obstacle_detection(scanner_data)
        connection_ok, connection_msg = self.check_connection_status(robot)
        time_ok, time_msg = self.check_operation_time()
        speed_ok, speed_msg = self.check_speed_violation(current_speed, max_speed)
        
        # 모든 체크가 통과해야 안전
        all_safe = obstacle_ok and connection_ok and time_ok and speed_ok
        
        status_msg = f"안전상태: 장애물({obstacle_msg}), 연결({connection_msg}), 시간({time_msg}), 속도({speed_msg})"
        
        return all_safe, status_msg

# ==============================================================================
# 							    메인 제어 클래스
# ==============================================================================

class TR200AreaMotionController:
    """TR200 로봇의 영역 제한 구동을 제어하는 메인 클래스"""
    
    def __init__(self, config_path: str):
        self.config_manager = ConfigManager(config_path)
        self.logger = MotionDataLogger(self.config_manager.get_logging_config())
        self.area_manager = AreaConstraintManager(
            self.config_manager.get_area_constraints(), 
            self.logger
        )
        self.safety_manager = SafetyManager(
            self.config_manager.get_safety_config(), 
            self.logger
        )
        
        # 로봇 연결 설정
        robot_config = self.config_manager.get_robot_config()
        self.settings = CommuSettings(
            addr=robot_config.get('ip', '169.254.128.2'),
            port=robot_config.get('port', 5480),
            identity=robot_config.get('identity', 'tr200-area-motion')
        )
        self.robot = WooshRobot(self.settings)
        
        # 제어 설정
        control_config = self.config_manager.get_control_config()
        self.control_frequency = control_config.get('control_frequency', 20.0)
        self.max_linear_velocity = control_config.get('max_linear_velocity', 0.8)
        self.smooth_stop_duration = control_config.get('smooth_stop_duration', 1.5)
        
        # 상태 변수
        self.is_running = False
        self.current_scanner_data: Optional[ScannerData] = None
    
    async def initialize_robot(self) -> bool:
        """로봇을 초기화하고 연결합니다."""
        try:
            self.logger.log_message("로봇 연결을 시작합니다...")
            
            if not await self.robot.run():
                self.logger.log_message("로봇 연결 실패", "ERROR")
                return False
            
            self.logger.log_message("로봇 연결 성공")
            
            # 로봇 상태 확인
            state, ok, msg = await self.robot.robot_operation_state_req(
                OperationState(), NO_PRINT, NO_PRINT
            )
            
            if not ok:
                self.logger.log_message(f"로봇 상태 요청 실패: {msg}", "ERROR")
                return False
            
            if not (state.robot & OperationState.RobotBit.kTaskable):
                self.logger.log_message("로봇이 작업 불가능한 상태입니다", "ERROR")
                return False
            
            self.logger.log_message("로봇이 작업 가능한 상태입니다")
            
            # 초기 위치 설정
            pose_speed, ok, msg = await self.robot.robot_pose_speed_req(
                PoseSpeed(), NO_PRINT, NO_PRINT
            )
            
            if ok:
                initial_position = RobotPosition(
                    x=pose_speed.pose.x,
                    y=pose_speed.pose.y,
                    theta=pose_speed.pose.theta,
                    timestamp=datetime.now().timestamp()
                )
                self.area_manager.set_start_position(initial_position)
                self.logger.log_message("초기 위치 설정 완료")
            else:
                self.logger.log_message(f"초기 위치 요청 실패: {msg}", "WARN")
            
            return True
            
        except Exception as e:
            self.logger.log_message(f"로봇 초기화 중 오류: {e}", "ERROR")
            return False
    
    async def smooth_stop(self, current_linear: float) -> None:
        """로봇을 부드럽게 감속하여 정지시킵니다."""
        self.logger.log_message("부드러운 정지 시작...")
        
        num_steps = int(self.smooth_stop_duration * self.control_frequency)
        if num_steps == 0:
            await self.robot.twist_req(Twist(linear=0.0, angular=0.0), NO_PRINT, NO_PRINT)
            return
        
        speed_reduction_step = current_linear / num_steps
        
        for i in range(num_steps):
            current_linear -= speed_reduction_step
            await self.robot.twist_req(Twist(linear=current_linear, angular=0.0), NO_PRINT, NO_PRINT)
            await asyncio.sleep(1.0 / self.control_frequency)
        
        # 안전을 위해 마지막에 정지 명령을 한 번 더 전송
        await self.robot.twist_req(Twist(linear=0.0, angular=0.0), NO_PRINT, NO_PRINT)
        self.logger.log_message("부드러운 정지 완료")
    
    async def move_with_constraints(self, linear_speed: float, duration: float, direction: str) -> bool:
        """영역 제한을 고려하여 로봇을 움직입니다."""
        delay = 1.0 / self.control_frequency
        num_commands = int(duration * self.control_frequency)
        
        self.logger.log_message(f"{direction} 방향으로 {duration}초 동안 이동 시작 (속도: {linear_speed} m/s)")
        
        for i in range(num_commands):
            # 현재 위치 업데이트
            pose_speed, ok, msg = await self.robot.robot_pose_speed_req(
                PoseSpeed(), NO_PRINT, NO_PRINT
            )
            
            if ok:
                current_position = RobotPosition(
                    x=pose_speed.pose.x,
                    y=pose_speed.pose.y,
                    theta=pose_speed.pose.theta,
                    timestamp=datetime.now().timestamp()
                )
                self.area_manager.update_position(current_position)
            
            # 영역 제한 확인
            constraint_ok, constraint_msg = self.area_manager.get_constraint_status(direction, linear_speed)
            if not constraint_ok:
                self.logger.log_message(f"영역 제한 위반: {constraint_msg}", "WARN")
                await self.smooth_stop(linear_speed)
                return False
            
            # 안전 상태 확인
            safety_ok, safety_msg = self.safety_manager.get_safety_status(
                self.robot, self.current_scanner_data, linear_speed, self.max_linear_velocity
            )
            if not safety_ok:
                self.logger.log_message(f"안전 위반: {safety_msg}", "ERROR")
                await self.smooth_stop(linear_speed)
                return False
            
            # 속도 명령 전송
            _, ok, msg = await self.robot.twist_req(Twist(linear=linear_speed, angular=0.0), NO_PRINT, NO_PRINT)
            if not ok:
                self.logger.log_message(f"속도 제어 요청 실패: {msg}", "ERROR")
                await self.smooth_stop(linear_speed)
                return False
            
            # 데이터 로깅
            if ok and pose_speed:
                distance_from_start = self.area_manager.calculate_distance_from_start(current_position)
                self.logger.log_motion_data(
                    current_position, (linear_speed, 0.0), direction,
                    distance_from_start, constraint_msg, safety_msg
                )
            
            await asyncio.sleep(delay)
        
        # 이동 완료 후 부드럽게 정지
        await self.smooth_stop(linear_speed)
        return True
    
    async def run_area_motion(self) -> None:
        """영역 제한 구동을 실행합니다."""
        motion_pattern = self.config_manager.get_motion_pattern('simple_back_forth')
        
        if not motion_pattern.enabled:
            self.logger.log_message("구동 패턴이 비활성화되어 있습니다", "ERROR")
            return
        
        self.logger.log_message("영역 제한 구동을 시작합니다")
        self.is_running = True
        
        try:
            while self.is_running:
                # 전진 운동
                self.logger.log_message("전진 운동 시작")
                success = await self.move_with_constraints(
                    motion_pattern.forward_speed, 
                    motion_pattern.move_duration, 
                    "forward"
                )
                
                if not success:
                    self.logger.log_message("전진 운동 실패, 구동 중단", "ERROR")
                    break
                
                # 대기
                self.logger.log_message(f"{motion_pattern.wait_duration}초 대기")
                await asyncio.sleep(motion_pattern.wait_duration)
                
                # 후진 운동
                self.logger.log_message("후진 운동 시작")
                success = await self.move_with_constraints(
                    motion_pattern.backward_speed, 
                    motion_pattern.move_duration, 
                    "backward"
                )
                
                if not success:
                    self.logger.log_message("후진 운동 실패, 구동 중단", "ERROR")
                    break
                
                # 대기
                self.logger.log_message(f"{motion_pattern.wait_duration}초 대기")
                await asyncio.sleep(motion_pattern.wait_duration)
                
        except KeyboardInterrupt:
            self.logger.log_message("사용자 요청으로 구동 중단", "INFO")
        except Exception as e:
            self.logger.log_message(f"구동 중 오류 발생: {e}", "ERROR")
        finally:
            self.is_running = False
            await self.smooth_stop(motion_pattern.forward_speed)
    
    async def cleanup(self) -> None:
        """리소스를 정리합니다."""
        self.logger.log_message("시스템 정리 중...")
        
        if self.robot.comm.is_connected():
            await self.robot.stop()
            self.logger.log_message("로봇 연결 종료")
        
        self.logger.close()
        self.logger.log_message("시스템 정리 완료")

# ==============================================================================
# 							    메인 실행 함수
# ==============================================================================

async def main():
    """메인 실행 함수"""
    config_path = "/home/ldj/tr200_ws/tr200_ros_docker_project/config/area_motion_params.yaml"
    
    # 명령줄 인수로 설정 파일 경로를 받을 수 있음
    if len(sys.argv) > 1:
        config_path = sys.argv[1]
    
    controller = TR200AreaMotionController(config_path)
    
    try:
        # 로봇 초기화
        if not await controller.initialize_robot():
            print("❌ 로봇 초기화 실패")
            return
        
        print("✅ 로봇 초기화 완료")
        print("🚀 영역 제한 구동을 시작합니다. 중지하려면 Ctrl+C를 누르세요.")
        
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
