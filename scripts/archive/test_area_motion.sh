#!/bin/bash
# test_area_motion.sh
# TR200 영역 제한 구동 테스트 스크립트

# 스크립트 디렉토리로 이동
cd "$(dirname "$0")/../.."

# 색상 정의
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

echo -e "${BLUE}========================================${NC}"
echo -e "${BLUE}  TR200 영역 제한 구동 테스트${NC}"
echo -e "${BLUE}========================================${NC}"

# 테스트용 설정 파일 생성
TEST_CONFIG="config/test_area_motion_params.yaml"
cat > "$TEST_CONFIG" << 'EOF'
# TR200 영역 제한 구동 테스트 설정

# 로봇 연결 설정
robot:
  ip: "169.254.128.2"
  port: 5480
  identity: "tr200-test-motion"

# 영역 제한 설정 (테스트용 - 작은 영역)
area_constraints:
  max_forward_distance: 1.0      # 테스트용 작은 거리
  max_backward_distance: 1.0
  safety_margin: 0.2
  detection_method: "distance"
  use_initial_position: true
  initial_position:
    x: 0.0
    y: 0.0
    theta: 0.0

# 구동 패턴 설정 (테스트용 - 느린 속도)
motion_patterns:
  simple_back_forth:
    enabled: true
    forward_speed: 0.2            # 테스트용 느린 속도
    backward_speed: -0.2
    move_duration: 3.0           # 테스트용 짧은 시간
    wait_duration: 1.0

# 제어 설정
control:
  max_linear_velocity: 0.5
  control_frequency: 20.0
  smooth_stop_duration: 1.0

# 안전 설정
safety:
  obstacle_detection_enabled: false  # 테스트용으로 비활성화
  emergency_stop_enabled: true
  max_operation_time: 60.0           # 테스트용 짧은 시간
  connection_monitoring: true
  speed_monitoring: true

# 로깅 설정
logging:
  level: "INFO"
  log_to_file: true
  log_file_path: "/home/ldj/tr200_ws/tr200_ros_docker_project/logs/test_motion.log"
  console_output: true
  detailed_status: true
  collect_motion_data: true
  data_file_path: "/home/ldj/tr200_ws/tr200_ros_docker_project/logs/test_motion_data.csv"

# 디버깅 설정
debug:
  verbose_logging: true
  simulation_mode: false
  test_mode: true
  test_duration: 30.0
EOF

echo -e "${GREEN}✅ 테스트 설정 파일 생성: $TEST_CONFIG${NC}"

# 로그 디렉토리 생성
mkdir -p logs

echo -e "${YELLOW}🧪 테스트 모드로 실행합니다...${NC}"
echo -e "${YELLOW}   - 작은 영역 (1m x 1m)${NC}"
echo -e "${YELLOW}   - 느린 속도 (0.2 m/s)${NC}"
echo -e "${YELLOW}   - 짧은 시간 (3초 이동, 1초 대기)${NC}"
echo -e "${YELLOW}   - 최대 60초 실행${NC}"
echo ""

# 테스트 실행
python3 "src/tr200_simple_control/scripts/tr200_area_constrained_motion.py" "$TEST_CONFIG"

# 실행 결과 확인
if [ $? -eq 0 ]; then
    echo -e "${GREEN}✅ 테스트가 정상적으로 완료되었습니다.${NC}"
    echo -e "${BLUE}📊 로그 파일을 확인하세요:${NC}"
    echo -e "${BLUE}   - logs/test_motion.log${NC}"
    echo -e "${BLUE}   - logs/test_motion_data.csv${NC}"
else
    echo -e "${RED}❌ 테스트 실행 중 오류가 발생했습니다.${NC}"
    echo -e "${YELLOW}💡 로그 파일을 확인하여 문제를 파악하세요.${NC}"
fi

echo -e "${BLUE}========================================${NC}"
echo -e "${BLUE}  테스트 완료${NC}"
echo -e "${BLUE}========================================${NC}"
