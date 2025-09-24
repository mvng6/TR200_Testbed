# TR200 ROS + SDK 통합 제어 시스템

## 📋 프로젝트 개요

TR200 로봇의 **듀얼 Lidar 센서**를 활용한 실시간 장애물 감지 및 안전 제어 시스템입니다. Docker 기반 ROS Noetic 환경에서 Woosh SDK를 통해 TR200 로봇을 제어하며, ROS와 SDK의 장점을 모두 활용한 통합 시스템입니다.

## 🎯 핵심 기능

- **ROS + SDK 통합**: ROS의 모듈성과 SDK의 직접 제어 장점 결합
- **실시간 장애물 감지**: TR200의 **듀얼 Lidar 센서**를 활용한 360도 전후방 장애물 감지
- **지능형 속도 제어**: 거리에 따른 자동 속도 조절 (정상 → 감속 → 정지)
- **안전 우선 제어**: 위험 상황에서 즉시 비상 정지
- **부드러운 제어**: 급격한 가속/감속 방지
- **듀얼 센서 융합**: 전방/후방 센서 데이터를 통합한 정확한 장애물 감지
- **Docker 기반 환경**: Ubuntu 20.04 + ROS Noetic 환경에서 안정적 실행
- **ROS 토픽/서비스**: 실시간 모니터링 및 동적 파라미터 조정 가능

## 📁 프로젝트 구조

```
tr200_ros_docker_project/
├── README.md                         # 프로젝트 문서
├── ROS_INDUSTRY_DEVELOPMENT_GUIDE.md # 현업 ROS 개발 가이드
├── scripts/                          # 실행 스크립트
│   ├── build_docker.sh               # Docker 이미지 빌드
│   ├── run_container.sh              # Docker 컨테이너 실행
│   ├── connect_container.sh          # 추가 터미널 접속
│   ├── setup_ros.sh                  # ROS 환경 설정
│   └── test_ros_sensor_safety.sh     # ROS 통합 테스트
├── src/tr200_simple_control/        # ROS 패키지
│   ├── scripts/                      # Python 제어 스크립트
│   │   ├── tr200_ros_sensor_safety_controller.py  # ROS 통합 제어기
│   │   ├── sensor_based_safety_controller.py      # 순수 SDK 제어기
│   │   ├── robot_driver_node.py                   # 로봇 제어 노드
│   │   ├── safety_controller_node.py              # 안전 제어 노드
│   │   ├── sensor_processor_node.py              # 센서 처리 노드
│   │   └── simple_linear_motion.py               # 기본 왕복 운동
│   ├── srv/                          # ROS 서비스 정의
│   │   └── SetSafetyParams.srv       # 안전 파라미터 설정 서비스
│   ├── CMakeLists.txt                # 빌드 설정
│   └── package.xml                   # 패키지 매니페스트
├── src/woosh_robot_py/              # Woosh SDK
│   ├── README.md                    # SDK 문서
│   ├── examples/                    # 예제 코드
│   └── woosh/                       # SDK 핵심 모듈
├── config/                           # 설정 파일
│   ├── tr200_sensor_safety_params.yaml  # ROS 통합 안전 파라미터
│   ├── tr200_sensor_safety.rviz     # RViz 시각화 설정
│   ├── area_motion_params.yaml       # 영역 제한 구동 파라미터
│   ├── robot_params.yaml            # 로봇 기본 파라미터
│   └── test_safe_params.yaml        # 테스트용 안전 파라미터
├── launch/                           # ROS 런치 파일
│   ├── tr200_sensor_safety_controller.launch  # 통합 제어기 런치
│   └── tr200_modular_system.launch   # 모듈화 시스템 런치
├── docker/                           # Docker 환경
│   ├── Dockerfile                    # Docker 이미지 정의
│   ├── docker-compose.yml           # 컨테이너 오케스트레이션
│   ├── entrypoint.sh                # 컨테이너 시작 스크립트
│   ├── data/                        # 데이터 디렉토리
│   └── logs/                        # 로그 디렉토리
└── archive/                          # 이전 버전 파일들
```

## 🚀 사용 방법

### 🐳 Docker 환경 설정

#### 1. Docker 이미지 빌드 (처음 한 번만)
```bash
./scripts/build_docker.sh
```

#### 2. Docker 컨테이너 실행 (첫 번째 터미널)
```bash
./scripts/run_container.sh
```

#### 3. 추가 터미널 접속 (두 번째 터미널)
```bash
./scripts/connect_container.sh
```

### 🔧 ROS 환경 설정

#### 1. ROS 환경 설정 (컨테이너 내부에서)
```bash
./scripts/setup_ros.sh
```

#### 2. ROS Master 시작 (수동)
```bash
roscore &
```

### 🤖 TR200 로봇 제어 실행

#### 방법 1: 모듈화된 ROS 시스템 (권장)
```bash
# 모듈화된 시스템 실행 (3개 노드로 분리)
roslaunch tr200_simple_control tr200_modular_system.launch

# RViz 시각화 포함
roslaunch tr200_simple_control tr200_modular_system.launch use_rviz:=true

# 키보드 텔레옵 포함
roslaunch tr200_simple_control tr200_modular_system.launch use_teleop:=true

# 로그 레코딩 포함
roslaunch tr200_simple_control tr200_modular_system.launch record_logs:=true
```

#### 방법 2: ROS 통합 제어기
```bash
# ROS 런치 파일로 실행
roslaunch tr200_simple_control tr200_sensor_safety_controller.launch

# 또는 직접 실행
rosrun tr200_simple_control tr200_ros_sensor_safety_controller.py
```

#### 방법 3: 순수 SDK 제어기
```bash
# 순수 SDK 버전 (ROS 없이)
python3 src/tr200_simple_control/scripts/sensor_based_safety_controller.py
```

#### 방법 4: 기본 왕복 운동 (참고용)
```bash
# 기본 왕복 운동 (센서 없이)
python3 src/tr200_simple_control/scripts/simple_linear_motion.py
```

### 🔍 ROS 모니터링 및 디버깅

#### ROS 토픽 모니터링
```bash
# 안전 상태 모니터링
rostopic echo /safety_status

# 장애물 거리 모니터링
rostopic echo /obstacle_distance

# 속도 명령 모니터링
rostopic echo /cmd_vel

# 스캐너 데이터 모니터링
rostopic echo /scan
```

#### ROS 서비스 사용
```bash
# 안전 파라미터 동적 변경
rosservice call /set_safety_params "warning_distance: 0.6, danger_distance: 0.4, normal_speed: 0.15"

# 안전 모드 토글
rosservice call /toggle_safety_mode "data: true"
```

#### RViz 시각화
```bash
# RViz로 센서 데이터 시각화
rviz -d config/tr200_sensor_safety.rviz
```

## ⚙️ 설정 파라미터

### ROS 통합 안전 제어 설정
- **경고 거리 (warning_distance)**: 0.8m (이 거리에서 감속 시작)
- **위험 거리 (danger_distance)**: 0.5m (이 거리 이하에서 즉시 정지)
- **안전 거리 (safe_distance)**: 1.0m (이 거리 이상에서 정상 속도)

### 제어 설정
- **정상 속도 (normal_speed)**: 0.2 m/s
- **감속 속도 (slow_speed)**: 0.1 m/s
- **정지 속도 (stop_speed)**: 0.0 m/s
- **제어 주파수 (control_rate)**: 20Hz

### 설정 파일들
- **`config/tr200_sensor_safety_params.yaml`**: ROS 통합 안전 파라미터 (기본 설정)
- **`config/area_motion_params.yaml`**: 영역 제한 구동 파라미터
- **`config/robot_params.yaml`**: 로봇 기본 파라미터
- **`config/test_safe_params.yaml`**: 테스트용 안전 파라미터 (더 보수적 설정)

### ROS 토픽 및 서비스

#### 발행 토픽
- **`/cmd_vel`**: 속도 명령 (geometry_msgs/Twist)
- **`/scan`**: 스캐너 데이터 (sensor_msgs/LaserScan)
- **`/safety_status`**: 안전 상태 (std_msgs/String)
- **`/obstacle_distance`**: 장애물 거리 (std_msgs/Float32)

#### 구독 토픽
- **`/cmd_vel_input`**: 외부 속도 명령 입력 (geometry_msgs/Twist)

#### 제공 서비스
- **`/set_safety_params`**: 안전 파라미터 동적 설정 (tr200_simple_control/SetSafetyParams)
- **`/toggle_safety_mode`**: 안전 모드 토글 (std_srvs/SetBool)

## 📊 예상 동작

### ROS 통합 제어기 실행 시
```
[INFO] [1234567890.123]: Starting TR200 ROS Sensor Safety Controller
[INFO] [1234567890.124]: ROS Master URI: http://localhost:11311
[INFO] [1234567890.125]: Loading parameters from config file
[INFO] [1234567890.126]: Warning distance: 0.8m
[INFO] [1234567890.127]: Danger distance: 0.5m
[INFO] [1234567890.128]: Normal speed: 0.2 m/s
[INFO] [1234567890.129]: Connecting to TR200 robot...
[INFO] [1234567890.130]: TR200 robot connected successfully
[INFO] [1234567890.131]: Subscribing to scanner data...
[INFO] [1234567890.132]: Publishing to /cmd_vel topic
[INFO] [1234567890.133]: Safety controller initialized
```

### 센서 데이터 처리
```
📡 스캐너 데이터 수신: 1081개 포인트
📡 각도 범위: -3.14° ~ 3.14°
📡 거리 범위: 0.01m ~ 50.00m
============================================================
🔍 센서 분리 완료:
   전방 센서: 540개 포인트 (인덱스 270~810)
   후방 센서: 540개 포인트 (인덱스 810~270)
🔍 듀얼 센서 분석:
   전방 센서: 최소거리 1.200m, 장애물 2개
   후방 센서: 최소거리 2.500m, 장애물 0개
   전체 최소거리: 1.200m, 총 장애물: 2개
```

### ROS 토픽 출력
```
08:15:23 | forward   |  0.200 |  1.200m | 🟢 안전
08:15:24 | forward   |  0.200 |  1.100m | 🟢 안전
08:15:25 | forward   |  0.200 |  0.900m | 🟡 주의
⚠️ 주의: 장애물 근접 (거리: 0.900m)
08:15:26 | forward   |  0.100 |  0.800m | 🟡 주의
08:15:27 | forward   |  0.100 |  0.600m | 🟡 주의
08:15:28 | forward   |  0.000 |  0.500m | 🔴 위험
🚨 위험! 장애물 감지 (거리: 0.500m)
🚨 비상 정지! 이유: 장애물 감지 (거리: 0.500m)
✅ 비상 정지 완료
```

### ROS 서비스 호출 예시
```bash
# 안전 파라미터 동적 변경
$ rosservice call /set_safety_params "warning_distance: 0.6, danger_distance: 0.4, normal_speed: 0.15"
success: True
message: "Safety parameters updated successfully"

# 안전 모드 토글
$ rosservice call /toggle_safety_mode "data: true"
success: True
message: "Safety mode enabled"
```

## 🔧 파일 설명

### 핵심 파일들
- **`tr200_ros_sensor_safety_controller.py`**: **ROS + SDK 통합 제어기** (ROS 토픽/서비스 + SDK 직접 제어)
- **`sensor_based_safety_controller.py`**: **순수 SDK 제어기** (듀얼 Lidar 센서 기반 안전 제어)
- **`robot_driver_node.py`**: **로봇 제어 노드** (SDK를 통한 실제 로봇 제어)
- **`safety_controller_node.py`**: **안전 제어 노드** (센서 데이터 기반 안전 제어)
- **`sensor_processor_node.py`**: **센서 처리 노드** (Lidar 데이터 처리 및 발행)
- **`simple_linear_motion.py`**: 기본 왕복 운동 (참고용, 센서 없이 단순 이동)

### ROS 런치 파일들
- **`tr200_modular_system.launch`**: **모듈화된 시스템 런치** (3개 노드 분리 실행)
- **`tr200_sensor_safety_controller.launch`**: ROS 통합 제어기 런치 파일

### ROS 서비스 및 설정
- **`SetSafetyParams.srv`**: 안전 파라미터 설정 서비스 정의
- **`tr200_sensor_safety_params.yaml`**: ROS 통합 안전 파라미터 설정
- **`tr200_sensor_safety.rviz`**: RViz 시각화 설정

### 실행 스크립트들
- **`build_docker.sh`**: Docker 이미지 빌드
- **`run_container.sh`**: Docker 컨테이너 실행
- **`connect_container.sh`**: 추가 터미널 접속
- **`setup_ros.sh`**: ROS 환경 설정
- **`test_ros_sensor_safety.sh`**: ROS 통합 테스트

### 설정 파일들
- **`tr200_sensor_safety_params.yaml`**: ROS 통합 안전 파라미터 (기본 설정)
- **`area_motion_params.yaml`**: 영역 제한 구동 파라미터
- **`robot_params.yaml`**: 로봇 기본 파라미터
- **`test_safe_params.yaml`**: 테스트용 안전 파라미터

## 🎯 개발 목표 달성

✅ **ROS + SDK 통합**: ROS의 모듈성과 SDK의 직접 제어 장점 결합  
✅ **모듈화된 ROS 시스템**: 센서 처리, 안전 제어, 로봇 제어 노드 분리  
✅ **특정 영역 내에서만 앞뒤 구동**: 위치 기반 영역 제한 시스템 구현  
✅ **네비게이션 없이 직접 구동**: 속도 제어 기반 시스템 구현  
✅ **센서 기반 장애물 감지**: TR200 Lidar 센서 활용한 실시간 장애물 감지  
✅ **안전한 구동**: 거리 기반 자동 속도 조절 및 비상 정지  
✅ **ROS 토픽/서비스**: 실시간 모니터링 및 동적 파라미터 조정  
✅ **현업 표준**: 모듈화 설계, 파라미터 관리, 런치 파일 구조  

## 🌐 네트워크 설정

### TR200 로봇 연결
- **IP**: 169.254.128.2
- **Port**: 5480
- **네트워크 모드**: host (Docker)

### ROS 통신
- **ROS Master**: http://localhost:11311
- **ROS Hostname**: localhost

## 🐛 문제 해결

### Docker 관련
```bash
# 컨테이너 상태 확인
docker ps | grep tr200_control_container

# 컨테이너 로그 확인
docker logs tr200_control_container

# 컨테이너 강제 재시작
docker stop tr200_control_container && docker rm tr200_control_container
```

### ROS 관련
```bash
# ROS Master 재시작
pkill roscore && roscore &

# 워크스페이스 재빌드
catkin build

# 환경 변수 확인
echo $ROS_PACKAGE_PATH
echo $PYTHONPATH
```

### TR200 연결 문제
```bash
# 네트워크 연결 확인
ping 169.254.128.2

# 포트 확인
telnet 169.254.128.2 5480
```

## 📝 개발 가이드

### 새로운 노드 추가
1. `src/tr200_simple_control/scripts/`에 Python 파일 추가
2. `CMakeLists.txt`에 실행 파일 등록
3. `package.xml`에 의존성 추가
4. 워크스페이스 재빌드

### 새로운 런치 파일 추가
1. `launch/` 디렉토리에 `.launch` 파일 추가
2. 파라미터 설정 확인
3. 테스트 실행

## 🔒 보안 고려사항

- Docker 컨테이너는 `privileged` 모드로 실행됩니다
- 네트워크는 `host` 모드를 사용합니다
- TR200 로봇과의 통신은 WebSocket을 통해 이루어집니다

## 📞 지원

문제가 발생하거나 추가 기능이 필요한 경우:
1. 로그 파일 확인 (`docker/logs/` 디렉토리)
2. 설정 파일 검토 (`config/` 디렉토리)
3. Archive 폴더의 이전 버전 참조 (`archive/` 디렉토리)
4. Woosh SDK 문서 참조 (`src/woosh_robot_py/README.md`)

### 주요 문제 해결
- **로봇 연결 실패**: TR200 앱에서 로봇 상태 확인 (비상정지 해제, 작업 모드 설정)
- **kTaskable 상태 문제**: `archive/force_test_mode.py` 참조
- **센서 데이터 없음**: TR200의 Lidar 센서 상태 확인
- **ROS 통신 문제**: `roscore` 실행 확인 및 네트워크 설정 검토
- **장애물 감지 개선**: 듀얼 센서 융합, 전방/후방 섹터 분석, 강화된 비상 정지 시스템

---

**개발자**: ldj  
**버전**: v3.1.0 (모듈화된 ROS + SDK 통합 시스템)  
**최종 업데이트**: 2025년 01월 23일
