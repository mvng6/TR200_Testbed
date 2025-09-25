# TR200 ROS Control Package

TR200 로봇을 ROS Noetic 환경에서 SDK와 연동하여 제어하는 패키지입니다.

## 📋 개요

이 패키지는 TR200 로봇을 ROS 환경에서 제어하기 위한 통합 솔루션을 제공합니다. SDK를 통한 로봇 연결, 센서 데이터 처리, 안전 제어, 원격 제어 등의 기능을 포함합니다.

## 🏗️ 시스템 구조

### 노드 구성
- **Robot Connection Node**: TR200 로봇과의 SDK 연결 관리
- **Remote Control Node**: 키보드 입력을 통한 원격 제어
- **Sensor Node**: 라이다 센서 데이터 수집 및 처리
- **Safety Controller Node**: 센서 기반 안전 제어
- **Robot Driver Node**: 로봇 구동 제어
- **Command Handler Node**: 명령 우선순위 관리
- **Status Monitor Node**: 시스템 상태 모니터링

### ROS 토픽
- `/cmd_vel`: 로봇 속도 명령
- `/remote_cmd_vel`: 원격 제어 명령
- `/scanner_data`: 라이다 센서 데이터
- `/connection_status`: 로봇 연결 상태
- `/safety_status`: 안전 상태 정보

## 🚀 빠른 시작

### 1. 환경 설정

Docker 컨테이너에서 작업:
```bash
# 컨테이너 실행
./scripts/run_container.sh

# ROS 마스터 실행
roscore

# 추가 터미널 접속
./scripts/connect_container.sh
```

### 2. 패키지 빌드

```bash
cd /catkin_ws
catkin_make
source devel/setup.bash
```

### 3. 원격 제어 실행

```bash
# 원격 제어 시스템 실행
roslaunch tr200_ros_control tr200_remote_control.launch
```

### 4. 키보드 제어

터미널을 클릭한 후 다음 키를 사용하여 로봇을 제어할 수 있습니다:

- `w`: 전진
- `s`: 후진
- `a`: 좌회전
- `d`: 우회전
- `q`: 제자리 좌회전
- `e`: 제자리 우회전
- `스페이스`: 정지
- `x`: 비상정지
- `h`: 도움말 표시
- `c`: 연결 상태 확인

## 📁 패키지 구조

```
tr200_ros_control/
├── CMakeLists.txt
├── package.xml
├── README.md
├── launch/
│   └── tr200_remote_control.launch
├── config/
│   ├── robot_params.yaml
│   ├── safety_params.yaml
│   └── sensor_params.yaml
├── scripts/
│   ├── robot_connection_node.py
│   └── remote_control_node.py
├── srv/
│   ├── SetSafetyParams.srv
│   └── GetRobotStatus.srv
└── msg/
    ├── SafetyStatus.msg
    └── RobotStatus.msg
```

## ⚙️ 설정

### 로봇 연결 설정

`config/robot_params.yaml`에서 로봇 연결 정보를 설정할 수 있습니다:

```yaml
robot:
  ip: "169.254.128.2"  # TR200 로봇 IP
  port: 5480           # TR200 로봇 포트
  identity: "tr200_ros_controller"
```

### 원격 제어 설정

런치 파일에서 원격 제어 파라미터를 조정할 수 있습니다:

```bash
roslaunch tr200_ros_control tr200_remote_control.launch \
  linear_speed:=0.3 \
  angular_speed:=0.3 \
  command_timeout:=3.0
```

## 🔧 개발

### 새로운 노드 추가

1. `scripts/` 디렉토리에 새 노드 파일 생성
2. `CMakeLists.txt`에 스크립트 설치 추가
3. 필요시 런치 파일에 노드 추가

### 메시지/서비스 추가

1. `msg/` 또는 `srv/` 디렉토리에 새 파일 생성
2. `CMakeLists.txt`에 메시지/서비스 추가
3. `package.xml`에 의존성 추가
4. `catkin_make` 실행

## 🚨 안전 주의사항

- 로봇 주변에 사람이 없는지 확인하세요
- 비상정지 버튼이 작동하는지 확인하세요
- 첫 실행 시 낮은 속도로 테스트하세요
- 센서 데이터가 정상적으로 수신되는지 확인하세요

## 📊 모니터링

### 토픽 모니터링

```bash
# 연결 상태 확인
rostopic echo /connection_status

# 원격 제어 명령 확인
rostopic echo /remote_cmd_vel

# 로봇 상태 확인
rostopic echo /robot_status
```

### 노드 상태 확인

```bash
# 실행 중인 노드 확인
rosnode list

# 노드 정보 확인
rosnode info /robot_connection_node
rosnode info /remote_control_node
```

## 🐛 트러블슈팅

### 로봇 연결 실패

1. 로봇 IP 주소 확인
2. 네트워크 연결 상태 확인
3. 로봇 전원 상태 확인
4. 방화벽 설정 확인

### 키보드 입력이 작동하지 않음

1. 터미널을 클릭하여 포커스 확인
2. 키보드 권한 확인
3. 노드 실행 상태 확인

### 센서 데이터 수신 안됨

1. 로봇 연결 상태 확인
2. 센서 하드웨어 상태 확인
3. 토픽 구독 상태 확인

## 📞 지원

문제가 발생하면 다음을 확인하세요:

1. 로그 메시지 확인
2. ROS 토픽 상태 확인
3. 노드 실행 상태 확인
4. 네트워크 연결 상태 확인

## 📄 라이선스

MIT License

## 👥 기여자

- User (KATECH 연구원) - 프로젝트 설계 및 구현

---

**버전**: 1.0.0  
**최종 업데이트**: 2024년 12월 19일
