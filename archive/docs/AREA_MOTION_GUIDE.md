# TR200 영역 제한 구동 시스템 사용 가이드

## 📋 개요

이 시스템은 TR200 로봇이 특정 영역 내에서만 앞뒤로 움직이도록 제한하는 고급 제어 시스템입니다. 네비게이션 기능 없이 직접 속도 제어를 통해 안전하고 정확한 구동을 구현합니다.

## 🎯 주요 기능

### 1. 영역 제한 구동
- 시작 위치를 기준으로 최대 전진/후진 거리 제한
- 실시간 위치 모니터링을 통한 영역 벗어남 방지
- 안전 여유 거리 설정으로 충돌 방지

### 2. 다양한 구동 패턴
- **기본 왕복 패턴**: 앞뒤로 반복 이동
- **단계별 이동 패턴**: 작은 단위로 정밀 이동
- **정밀 제어 패턴**: 매우 작은 거리로 세밀한 제어

### 3. 안전 기능
- 장애물 감지 및 충돌 방지
- 속도 제한 및 모니터링
- 연결 상태 모니터링
- 비상 정지 기능

### 4. 데이터 로깅
- 실시간 구동 데이터 수집
- CSV 형태의 상세 로그
- 위치, 속도, 안전 상태 기록

## 🚀 빠른 시작

### 1. 기본 실행
```bash
cd /home/ldj/tr200_ws/tr200_ros_docker_project
./scripts/run_area_motion.sh
```

### 2. 테스트 모드 실행
```bash
./scripts/test_area_motion.sh
```

### 3. 직접 실행
```bash
python3 src/tr200_simple_control/scripts/tr200_area_constrained_motion.py config/area_motion_params.yaml
```

## ⚙️ 설정 파일 설명

### 로봇 연결 설정
```yaml
robot:
  ip: "169.254.128.2"        # TR200 로봇 IP 주소
  port: 5480                  # 통신 포트
  identity: "tr200-area-motion"  # 클라이언트 식별자
```

### 영역 제한 설정
```yaml
area_constraints:
  max_forward_distance: 2.0   # 최대 전진 거리 (m)
  max_backward_distance: 2.0   # 최대 후진 거리 (m)
  safety_margin: 0.3          # 안전 여유 거리 (m)
  detection_method: "distance" # 거리 기반 제한
  use_initial_position: true   # 시작 위치 기준 사용
```

### 구동 패턴 설정
```yaml
motion_patterns:
  simple_back_forth:
    enabled: true
    forward_speed: 0.4         # 전진 속도 (m/s)
    backward_speed: -0.3       # 후진 속도 (m/s)
    move_duration: 4.0         # 한 방향 이동 시간 (초)
    wait_duration: 2.0         # 방향 전환 대기 시간 (초)
```

### 안전 설정
```yaml
safety:
  obstacle_detection_enabled: true
  min_obstacle_distance: 0.4   # 최소 장애물 거리 (m)
  emergency_stop_enabled: true
  max_operation_time: 300.0   # 최대 작동 시간 (초)
```

## 📊 모니터링 및 로깅

### 실시간 모니터링
프로그램 실행 중 다음 정보가 실시간으로 표시됩니다:
- 현재 위치 (X, Y, Theta)
- 이동 방향 및 속도
- 영역 제한 상태
- 안전 상태
- 장애물 감지 상태

### 로그 파일
- **일반 로그**: `logs/area_motion_YYYYMMDD_HHMMSS.log`
- **데이터 로그**: `logs/motion_data_YYYYMMDD_HHMMSS.csv`

### CSV 데이터 형식
```csv
timestamp,position_x,position_y,position_theta,linear_velocity,angular_velocity,move_direction,distance_from_start,area_constraint_status,safety_status
2024-01-15T10:30:00.123,0.123,0.456,0.789,0.4,0.0,forward,0.123,전진 가능,안전상태 정상
```

## 🔧 고급 설정

### 1. 다양한 구동 패턴 활성화

#### 단계별 이동 패턴
```yaml
motion_patterns:
  step_by_step:
    enabled: true
    step_distance: 0.5         # 한 번에 이동할 거리 (m)
    step_speed: 0.3            # 단계별 이동 속도 (m/s)
    pause_between_steps: 1.0   # 단계 간 대기 시간 (초)
    max_steps_forward: 4       # 최대 전진 단계 수
    max_steps_backward: 4      # 최대 후진 단계 수
```

#### 정밀 제어 패턴
```yaml
motion_patterns:
  precise_control:
    enabled: true
    small_moves: true          # 작은 단위로 이동
    move_distance: 0.2         # 작은 이동 거리 (m)
    move_speed: 0.2            # 정밀 이동 속도 (m/s)
    verification_pause: 0.5    # 각 이동 후 확인 대기 시간 (초)
```

### 2. 안전 설정 조정

#### 장애물 감지 민감도 조정
```yaml
safety:
  obstacle_detection_enabled: true
  min_obstacle_distance: 0.5   # 더 큰 안전 거리
```

#### 속도 제한 조정
```yaml
control:
  max_linear_velocity: 0.6     # 더 낮은 최대 속도
  smooth_stop_duration: 2.0    # 더 긴 감속 시간
```

### 3. 로깅 설정

#### 상세 로깅 활성화
```yaml
logging:
  level: "DEBUG"              # 더 상세한 로그
  verbose_logging: true       # 상세 상태 출력
  detailed_status: true       # 상세 상태 정보
```

#### 데이터 수집 간격 조정
```yaml
logging:
  data_collection_interval: 0.05  # 더 빈번한 데이터 수집 (20Hz)
```

## 🚨 문제 해결

### 1. 로봇 연결 실패
```
❌ 로봇 연결 실패 (169.254.128.2:5480)
```
**해결 방법:**
- TR200 로봇이 켜져 있는지 확인
- 네트워크 연결 상태 확인
- IP 주소가 올바른지 확인 (기본값: 169.254.128.2)

### 2. 로봇이 작업 불가능한 상태
```
🔴 로봇이 'kTaskable' 상태가 아닙니다
```
**해결 방법:**
- 로봇의 비상정지 버튼 확인
- 로봇의 전원 상태 확인
- 로봇의 다른 작업이 실행 중인지 확인

### 3. 영역 제한 위반
```
⚠️ 영역 제한 위반: 전진 제한 도달
```
**해결 방법:**
- 설정 파일에서 `max_forward_distance` 값 증가
- `safety_margin` 값 감소
- 로봇을 시작 위치로 되돌리기

### 4. 속도 위반 경고
```
⚠️ 속도 위반 경고 (2/3)
```
**해결 방법:**
- 설정 파일에서 `max_linear_velocity` 값 확인
- 구동 패턴의 속도 설정 확인
- 로봇의 물리적 상태 확인

## 📈 성능 최적화

### 1. 제어 주파수 조정
```yaml
control:
  control_frequency: 30.0      # 더 높은 제어 주파수 (더 부드러운 제어)
```

### 2. 데이터 수집 최적화
```yaml
logging:
  data_collection_interval: 0.1  # 적절한 수집 간격으로 성능 최적화
```

### 3. 안전 체크 간격 조정
실제 구현에서 안전 체크를 매 제어 주기마다 하지 않고, 적절한 간격으로 조정할 수 있습니다.

## 🔄 업데이트 및 확장

### 1. 새로운 구동 패턴 추가
`TR200AreaMotionController` 클래스에 새로운 패턴 메서드를 추가할 수 있습니다.

### 2. 센서 데이터 활용
`ScannerData`를 활용하여 더 정교한 장애물 감지 시스템을 구현할 수 있습니다.

### 3. ROS 통합
ROS 토픽을 통해 외부에서 구동 명령을 받거나 상태를 모니터링할 수 있습니다.

## 📞 지원

문제가 발생하거나 추가 기능이 필요한 경우:
1. 로그 파일 확인 (`logs/` 디렉토리)
2. 설정 파일 검토 (`config/area_motion_params.yaml`)
3. 테스트 모드로 문제 재현 (`./scripts/test_area_motion.sh`)

---

**개발자**: ldj  
**버전**: v1.0.0  
**최종 업데이트**: 2024년 1월 15일
