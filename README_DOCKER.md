# TR200 ROS Docker 개발 환경

## 🐳 Docker 환경 개요

이 프로젝트는 듀얼부팅 환경(22.04 Humble + 20.04 Noetic)에서 TR200 로봇을 제어하기 위한 Docker 컨테이너 환경을 제공합니다.

## 🚀 빠른 시작

### 1. Docker 컨테이너 실행
```bash
./scripts/run_container.sh
```

### 2. ROS 환경 설정 (컨테이너 내부에서)
```bash
./scripts/setup_ros.sh
```

### 3. TR200 로봇 제어 실행
```bash
# 단일 노드 버전
roslaunch tr200_simple_control tr200_sensor_safety_controller.launch

# 모듈화 버전
roslaunch tr200_simple_control tr200_modular_system.launch

# 순수 SDK 버전
rosrun tr200_simple_control sensor_based_safety_controller.py
```

## 📁 프로젝트 구조

```
tr200_ros_docker_project/
├── docker/                          # Docker 설정 파일들
│   ├── Dockerfile                   # Docker 이미지 정의
│   ├── docker-compose.yml           # Docker Compose 설정
│   ├── entrypoint.sh                # 컨테이너 진입점
│   ├── data/                        # 데이터 디렉토리
│   └── logs/                        # 로그 디렉토리
├── scripts/                         # 실행 스크립트들
│   ├── run_container.sh             # Docker 컨테이너 실행
│   ├── setup_ros.sh                 # ROS 환경 설정
│   ├── build_docker.sh              # Docker 이미지 빌드
│   └── test_ros_sensor_safety.sh    # 테스트 스크립트
├── src/                             # 소스 코드
│   ├── tr200_simple_control/        # ROS 패키지
│   └── woosh_robot_py/              # Woosh SDK
├── config/                          # 설정 파일들
└── launch/                          # 런치 파일들
```

## 🔧 환경 요구사항

### 호스트 시스템 (22.04 Humble)
- Docker
- Docker Compose
- X11 디스플레이 (GUI 지원)

### 컨테이너 환경 (20.04 Noetic)
- ROS Noetic
- Python 3.8
- TR200 SDK (Woosh)
- 필수 ROS 패키지들

## 📋 사용 가능한 명령어

### Docker 관련
```bash
./scripts/build_docker.sh           # Docker 이미지 빌드
./scripts/run_container.sh          # 컨테이너 실행
```

### ROS 관련 (컨테이너 내부에서)
```bash
catkin build                        # 워크스페이스 빌드
roslaunch tr200_simple_control tr200_sensor_safety_controller.launch
roslaunch tr200_simple_control tr200_modular_system.launch
rosrun tr200_simple_control sensor_based_safety_controller.py
rosrun tr200_simple_control simple_linear_motion.py
rviz -d /catkin_ws/config/tr200_sensor_safety.rviz
```

### 디버깅
```bash
rostopic list                       # 토픽 목록
rosnode list                        # 노드 목록
rostopic echo /safety_status        # 안전 상태 모니터링
rostopic echo /obstacle_distance    # 장애물 거리 모니터링
```

## 🌐 네트워크 설정

### TR200 로봇 연결
- IP: 169.254.128.2
- Port: 5480
- 네트워크 모드: host (Docker)

### ROS 통신
- ROS Master: http://localhost:11311
- ROS Hostname: localhost

## 🐛 문제 해결

### Docker 관련
```bash
# 컨테이너 상태 확인
docker-compose ps

# 컨테이너 로그 확인
docker-compose logs

# 컨테이너 강제 재시작
docker-compose down && docker-compose up -d
```

### ROS 관련
```bash
# ROS Master 재시작
pkill roscore && roscore &

# 워크스페이스 재빌드
catkin clean && catkin build

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

문제가 발생하면 다음을 확인하세요:
1. Docker 및 Docker Compose 설치 상태
2. TR200 로봇 네트워크 연결
3. ROS 환경 설정
4. 컨테이너 로그
