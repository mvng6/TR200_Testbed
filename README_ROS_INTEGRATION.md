# TR200 ROS 통합 센서 기반 안전 제어 시스템

## 🎯 개요

기존의 순수 SDK 기반 `sensor_based_safety_controller.py`를 ROS 환경으로 통합한 시스템입니다. ROS 생태계의 장점을 활용하면서도 Woosh SDK의 실시간 성능을 유지합니다.

## 🏗️ 시스템 아키텍처

```
ROS 환경
├── tr200_ros_sensor_safety_controller.py (메인 노드)
├── ROS 토픽 (/cmd_vel, /scan, /safety_status 등)
├── ROS 서비스 (/set_safety_params, /emergency_stop)
└── Woosh SDK 브리지 (비동기 통신)

TR200 로봇
├── 라이다 센서 (전방/후방)
├── 모터 제어
└── WebSocket 통신
```

## 🚀 주요 기능

### 1. **듀얼 센서 처리**
- 전방/후방 라이다 센서 데이터 분리 처리
- ROS LaserScan 메시지로 변환하여 퍼블리시

### 2. **실시간 안전 제어**
- 위험/경고/안전 구역 자동 판단
- 적응적 속도 제어 (정상/감속/정지)
- 비상 정지 기능

### 3. **ROS 통합**
- 표준 ROS 토픽 및 서비스 제공
- 외부 제어 명령 수신 가능
- 실시간 상태 모니터링

### 4. **동적 파라미터 조정**
- 런타임 안전 파라미터 변경
- 서비스를 통한 실시간 설정 업데이트

## 📁 파일 구조

```
src/tr200_simple_control/
├── scripts/
│   ├── tr200_ros_sensor_safety_controller.py  # ROS 통합 메인 노드
│   ├── sensor_based_safety_controller.py      # 기존 SDK 버전
│   └── ...
├── srv/
│   └── SetSafetyParams.srv                   # 안전 파라미터 설정 서비스
├── CMakeLists.txt
└── package.xml

launch/
└── tr200_sensor_safety_controller.launch     # 런치 파일

config/
├── tr200_sensor_safety_params.yaml           # 파라미터 설정
└── tr200_sensor_safety.rviz                  # RViz 설정

scripts/
└── test_ros_sensor_safety.sh                 # 테스트 스크립트
```

## 🛠️ 설치 및 실행

### 1. **워크스페이스 빌드**
```bash
cd /catkin_ws
catkin_make
source devel/setup.bash
```

### 2. **기본 실행**
```bash
# 기본 센서 안전 제어
roslaunch tr200_simple_control tr200_sensor_safety_controller.launch

# RViz 시각화와 함께
roslaunch tr200_simple_control tr200_sensor_safety_controller.launch use_rviz:=true

# 키보드 텔레옵과 함께
roslaunch tr200_simple_control tr200_sensor_safety_controller.launch use_teleop:=true
```

### 3. **테스트 스크립트 사용**
```bash
./scripts/test_ros_sensor_safety.sh
```

## 📊 ROS 토픽 및 서비스

### **퍼블리시 토픽**
- `/scan` (sensor_msgs/LaserScan): 라이다 센서 데이터
- `/safety_status` (std_msgs/String): 안전 상태 ("🟢 안전", "🟡 주의", "🔴 위험")
- `/obstacle_distance` (std_msgs/Float32): 최소 장애물 거리
- `/emergency_stop` (std_msgs/Bool): 비상 정지 상태
- `/robot_state` (std_msgs/String): 로봇 상태 정보

### **서브스크라이브 토픽**
- `/external_cmd_vel` (geometry_msgs/Twist): 외부 속도 명령

### **서비스**
- `/set_safety_params` (tr200_simple_control/SetSafetyParams): 안전 파라미터 설정
- `/emergency_stop` (std_srvs/SetBool): 비상 정지 제어

## ⚙️ 파라미터 설정

### **기본 파라미터**
```yaml
# 로봇 연결
robot_ip: "169.254.128.2"
robot_port: 5480

# 센서 설정 (미터)
min_obstacle_distance: 0.5    # 위험 구역
warning_distance: 0.8         # 주의 구역
safe_distance: 1.0            # 안전 구역

# 제어 설정 (m/s)
normal_speed: 0.2             # 정상 속도
slow_speed: 0.1               # 감속 속도
control_frequency: 20.0       # 제어 주파수
```

### **런타임 파라미터 변경**
```bash
# 서비스를 통한 파라미터 변경
rosservice call /set_safety_params "min_obstacle_distance: 0.3
warning_distance: 0.6
safe_distance: 0.9
normal_speed: 0.15
slow_speed: 0.05"
```

## 🔧 사용 예제

### **1. 기본 자동 구동**
```bash
roslaunch tr200_simple_control tr200_sensor_safety_controller.launch
```

### **2. 외부 제어와 함께**
```bash
# 터미널 1: 안전 제어 시스템
roslaunch tr200_simple_control tr200_sensor_safety_controller.launch

# 터미널 2: 외부 제어
rostopic pub /external_cmd_vel geometry_msgs/Twist "linear: {x: 0.3, y: 0.0, z: 0.0}
angular: {x: 0.0, y: 0.0, z: 0.0}"
```

### **3. 상태 모니터링**
```bash
# 안전 상태 확인
rostopic echo /safety_status

# 장애물 거리 확인
rostopic echo /obstacle_distance

# 로봇 상태 확인
rostopic echo /robot_state
```

### **4. 비상 정지**
```bash
# 비상 정지 활성화
rosservice call /emergency_stop "data: true"

# 비상 정지 해제
rosservice call /emergency_stop "data: false"
```

## 🔍 디버깅 및 모니터링

### **ROS 도구 사용**
```bash
# 토픽 목록 확인
rostopic list

# 노드 상태 확인
rosnode list
rosnode info /tr200_sensor_safety_controller

# 파라미터 확인
rosparam list | grep tr200_sensor_safety_controller

# 서비스 목록 확인
rosservice list | grep tr200_sensor_safety_controller
```

### **로그 확인**
```bash
# 노드 로그 확인
rosnode info /tr200_sensor_safety_controller

# ROS 로그 레벨 변경
rosparam set /rosout/log_level DEBUG
```

## 🚨 주의사항

1. **로봇 연결**: TR200 로봇이 켜져 있고 네트워크가 연결되어 있어야 합니다.
2. **안전 거리**: 환경에 맞게 안전 파라미터를 조정하세요.
3. **비상 정지**: 위험 상황에서는 즉시 비상 정지 서비스를 사용하세요.
4. **외부 제어**: 외부 명령이 있을 때는 자동 제어가 비활성화됩니다.

## 🔄 기존 SDK 버전과의 차이점

| 기능 | SDK 버전 | ROS 통합 버전 |
|------|----------|---------------|
| **통신 방식** | 직접 SDK | ROS + SDK 브리지 |
| **실시간 성능** | ⭐⭐⭐⭐⭐ | ⭐⭐⭐⭐ |
| **ROS 생태계** | ❌ 불가 | ✅ 완전 통합 |
| **외부 제어** | ❌ 불가 | ✅ 가능 |
| **모니터링** | 콘솔 출력 | ROS 도구 |
| **확장성** | 제한적 | 높음 |

## 🎯 활용 방안

1. **연구 개발**: ROS 생태계의 다양한 패키지와 통합
2. **멀티 로봇**: 여러 TR200 로봇의 협조 제어
3. **시각화**: RViz를 통한 실시간 센서 데이터 시각화
4. **데이터 수집**: ROS bag을 통한 센서 데이터 기록
5. **외부 제어**: 다른 ROS 노드에서의 제어 명령 수신

이 시스템을 통해 TR200 로봇을 ROS 생태계의 일부로 활용할 수 있으며, 기존 SDK의 실시간 성능을 유지하면서도 ROS의 확장성과 표준화의 장점을 모두 얻을 수 있습니다.
