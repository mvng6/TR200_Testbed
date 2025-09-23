# TR200 로봇 제어 시스템 설치 가이드

## 📋 사전 요구사항

### 하드웨어 요구사항
- **노트북**: Ubuntu 22.04 LTS + ROS Humble
- **TR200 로봇**: 네트워크 연결 가능 (기본 IP: 169.254.128.2)
- **RAM**: 최소 8GB (권장 16GB)
- **저장공간**: 최소 10GB 여유 공간

### 소프트웨어 요구사항
- **Docker**: 최신 버전 설치됨
- **Docker Compose**: v2.0 이상
- **Git**: 소스코드 관리용

## 🚀 설치 단계

### 1. 프로젝트 클론
```bash
cd /home/ldj/tr200_ws
git clone <repository_url> tr200_ros_docker_project
cd tr200_ros_docker_project
```

### 2. Docker 이미지 빌드
```bash
# 빌드 스크립트 실행
./scripts/build_docker.sh

# 또는 수동 빌드
docker build -t tr200_ros_noetic:latest -f docker/Dockerfile .
```

### 3. 컨테이너 실행
```bash
# 대화형 모드로 실행
./scripts/run_container.sh

# 또는 Docker Compose 사용
cd docker
docker-compose up -d
```

### 4. 워크스페이스 설정 (컨테이너 내부)
```bash
# 컨테이너 내부에서 실행
./scripts/setup_workspace.sh

# 수동 설정
cd /catkin_ws
catkin_make
source devel/setup.bash
```

## 🔧 네트워크 설정

### TR200 로봇 연결 확인
```bash
# 로봇 IP 연결 테스트
ping 169.254.128.2

# 포트 연결 테스트
nc -zv 169.254.128.2 5480

# SDK 연결 테스트 (컨테이너 내부)
tr200_test
```

### 네트워크 문제 해결
1. **로봇 IP 확인**: TR200 로봇의 실제 IP 주소 확인
2. **방화벽 설정**: 필요시 방화벽 규칙 추가
3. **네트워크 인터페이스**: 올바른 네트워크 인터페이스 사용 확인

## 🐳 Docker 설정

### 사용자 권한 설정
```bash
# Docker 그룹에 사용자 추가
sudo usermod -aG docker $USER

# 로그아웃 후 재로그인 또는
newgrp docker
```

### X11 포워딩 설정 (GUI 애플리케이션용)
```bash
# X11 권한 허용
xhost +local:docker

# 환경 변수 확인
echo $DISPLAY
```

## 📦 패키지 구조 확인

### 빌드된 ROS 패키지 확인
```bash
# 컨테이너 내부에서 실행
rospack list | grep tr200

# 예상 출력:
# tr200_control /catkin_ws/src/tr200_control
# tr200_interface /catkin_ws/src/tr200_interface  
# tr200_navigation /catkin_ws/src/tr200_navigation
```

### 토픽 및 서비스 확인
```bash
# ROS 마스터 시작
roscore &

# 토픽 목록 확인
rostopic list

# 노드 목록 확인
rosnode list
```

## 🧪 기본 테스트

### 1. SDK 연결 테스트
```bash
# 컨테이너 내부에서 실행
python3 /opt/woosh_robot_py/examples/monitor_ko.py --ip 169.254.128.2 --port 5480
```

### 2. ROS 브리지 테스트
```bash
# 브리지 노드 시작
rosrun tr200_interface tr200_bridge_node.py

# 다른 터미널에서 토픽 확인
rostopic echo /robot_status
```

### 3. 속도 제어 테스트
```bash
# 속도 제어 노드 시작
rosrun tr200_control velocity_controller.py

# 속도 명령 전송
rostopic pub /cmd_vel geometry_msgs/Twist "linear: {x: 0.1} angular: {z: 0.0}"
```

## 🔍 문제 해결

### 일반적인 문제들

#### 1. Docker 빌드 실패
```bash
# 캐시 없이 재빌드
docker build --no-cache -t tr200_ros_noetic:latest -f docker/Dockerfile .

# 디스크 공간 확인
df -h
docker system prune -f
```

#### 2. 컨테이너 실행 실패
```bash
# 기존 컨테이너 정리
docker stop tr200_control_container
docker rm tr200_control_container

# 로그 확인
docker logs tr200_control_container
```

#### 3. 네트워크 연결 문제
```bash
# 네트워크 인터페이스 확인
ip addr show

# 라우팅 테이블 확인
ip route

# DNS 설정 확인
cat /etc/resolv.conf
```

#### 4. ROS 통신 문제
```bash
# ROS 환경 변수 확인
env | grep ROS

# ROS 마스터 상태 확인
rostopic list

# 노드 간 통신 확인
rosnode ping /node_name
```

### 로그 확인
```bash
# 컨테이너 로그
docker logs tr200_control_container

# ROS 로그
tail -f /catkin_ws/logs/tr200_control.log

# 시스템 로그
journalctl -u docker
```

## 📚 추가 리소스

- [Docker 공식 문서](https://docs.docker.com/)
- [ROS Noetic 문서](http://wiki.ros.org/noetic)
- [TR200 사용자 매뉴얼](../3.%20TR-200/TR-200/Tracer%20200%20package/Tracer%20200%20manuals/)

## 🆘 지원

문제가 지속되면 다음을 확인하세요:
1. `TROUBLESHOOTING.md` 파일
2. 프로젝트 이슈 트래커
3. ROS 커뮤니티 포럼
