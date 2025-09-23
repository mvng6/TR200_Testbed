#!/bin/bash
# 컨테이너 내부에서 실행할 테스트 설정 생성 스크립트

# 컨테이너 내부에서 실행
cd /catkin_ws

# 테스트용 설정 파일 생성
cat > config/test_area_motion_params.yaml << 'EOF'
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
  log_file_path: "/catkin_ws/logs/test_motion.log"
  console_output: true
  detailed_status: true
  collect_motion_data: true
  data_file_path: "/catkin_ws/logs/test_motion_data.csv"

# 디버깅 설정
debug:
  verbose_logging: true
  simulation_mode: false
  test_mode: true
  test_duration: 30.0
EOF

echo "✅ 테스트 설정 파일이 생성되었습니다: config/test_area_motion_params.yaml"
echo "🚀 테스트를 시작하려면 다음 명령어를 실행하세요:"
echo "   python3 src/tr200_simple_control/scripts/tr200_area_constrained_motion.py config/test_area_motion_params.yaml"
