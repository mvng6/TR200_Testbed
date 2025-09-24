# 🏭 현업 ROS 개발 가이드

## 📋 목차
1. [ROS 개발 환경 구축](#ros-개발-환경-구축)
2. [코드베이스 구조 설계](#코드베이스-구조-설계)
3. [패키지 의존성 관리](#패키지-의존성-관리)
4. [파라미터 관리](#파라미터-관리)
5. [모듈화 설계](#모듈화-설계)
6. [디버깅 및 모니터링](#디버깅-및-모니터링)
7. [배포 및 유지보수](#배포-및-유지보수)
8. [현업 모범 사례](#현업-모범-사례)

---

## 🚀 ROS 개발 환경 구축

### 1. 워크스페이스 구조
```
catkin_ws/                    # 메인 워크스페이스
├── src/                      # 소스 코드
│   ├── robot_control/        # 로봇 제어 패키지
│   ├── sensor_processing/    # 센서 처리 패키지
│   ├── navigation/           # 내비게이션 패키지
│   └── common_msgs/          # 공통 메시지 패키지
├── build/                    # 빌드 파일 (자동 생성)
├── devel/                    # 개발 환경 (자동 생성)
└── install/                  # 설치 파일 (자동 생성)
```

### 2. 개발 도구 설정
```bash
# 필수 도구 설치
sudo apt install ros-noetic-desktop-full
sudo apt install python3-catkin-tools
sudo apt install ros-noetic-rqt-common-plugins

# 개발 도구
sudo apt install ros-noetic-rviz
sudo apt install ros-noetic-rqt-graph
sudo apt install ros-noetic-rosbag
```

### 3. 환경 설정
```bash
# ~/.bashrc에 추가
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
echo "export ROS_HOSTNAME=localhost" >> ~/.bashrc
echo "export ROS_MASTER_URI=http://localhost:11311" >> ~/.bashrc
```

---

## 🏗️ 코드베이스 구조 설계

### 1. 패키지 구조 표준
```
robot_control/
├── package.xml               # 패키지 메타데이터
├── CMakeLists.txt           # 빌드 설정
├── launch/                   # 런치 파일
│   ├── robot_control.launch
│   ├── simulation.launch
│   └── real_robot.launch
├── config/                   # 설정 파일
│   ├── robot_params.yaml
│   ├── control_params.yaml
│   └── safety_params.yaml
├── scripts/                  # Python 스크립트
│   ├── robot_controller.py
│   ├── safety_monitor.py
│   └── data_logger.py
├── src/                      # C++ 소스 (선택적)
│   └── robot_control_node.cpp
├── msg/                      # 커스텀 메시지
│   └── RobotStatus.msg
├── srv/                      # 커스텀 서비스
│   └── SetControlParams.srv
└── test/                     # 테스트 파일
    └── test_robot_control.py
```

### 2. 네이밍 컨벤션
```python
# 패키지명: snake_case
robot_control, sensor_processing, navigation

# 노드명: descriptive_snake_case
robot_controller, sensor_processor, navigation_planner

# 토픽명: 기능_기술
/cmd_vel, /scan, /odom, /camera/image_raw

# 서비스명: 동작_대상
/set_control_params, /get_robot_status, /emergency_stop
```

---

## 📦 패키지 의존성 관리

### 1. package.xml 작성 가이드
```xml
<?xml version="1.0"?>
<package format="2">
  <name>robot_control</name>
  <version>1.0.0</version>
  <description>Robot control package for TR200</description>
  
  <maintainer email="developer@company.com">Developer</maintainer>
  <license>MIT</license>
  
  <!-- 빌드 도구 -->
  <buildtool_depend>catkin</buildtool_depend>
  
  <!-- 빌드 의존성 -->
  <build_depend>rospy</build_depend>
  <build_depend>geometry_msgs</build_depend>
  <build_depend>sensor_msgs</build_depend>
  <build_depend>std_msgs</build_depend>
  <build_depend>message_generation</build_depend>
  
  <!-- 실행 의존성 -->
  <exec_depend>rospy</exec_depend>
  <exec_depend>geometry_msgs</exec_depend>
  <exec_depend>sensor_msgs</exec_depend>
  <exec_depend>std_msgs</exec_depend>
  <exec_depend>message_runtime</exec_depend>
  
  <!-- 테스트 의존성 -->
  <test_depend>rostest</test_depend>
</package>
```

### 2. 의존성 확인 방법
```bash
# 패키지 의존성 확인
rosdep check --from-paths src --ignore-src

# 의존성 자동 설치
rosdep install --from-paths src --ignore-src -r -y

# 패키지 검색
apt search ros-noetic-geometry-msgs
```

### 3. 일반적인 ROS 패키지들
```bash
# 기본 패키지
rospy, roscpp, std_msgs

# 메시지 패키지
geometry_msgs, sensor_msgs, nav_msgs, tf2_msgs

# 시각화 패키지
rviz, rqt-common-plugins, rqt-graph

# 내비게이션 패키지
move_base, amcl, map_server

# 센서 패키지
laser_scan_matcher, pointcloud_to_laserscan
```

---

## ⚙️ 파라미터 관리

### 1. 파라미터 우선순위
```bash
# 우선순위 (높음 → 낮음)
1. 런치 파일 <param> 태그
2. 런치 파일 <arg> 태그  
3. rosparam set 명령어
4. YAML 파일 (명시적 로드)
5. 코드 내 기본값
```

### 2. 파라미터 파일 구조
```yaml
# config/robot_params.yaml
# 로봇 하드웨어 설정
robot:
  ip: "192.168.1.100"
  port: 5480
  model: "TR200"
  
# 제어 설정
control:
  max_linear_speed: 0.5
  max_angular_speed: 1.0
  control_frequency: 20.0
  
# 안전 설정
safety:
  min_obstacle_distance: 0.5
  warning_distance: 0.8
  emergency_stop_enabled: true
  
# 센서 설정
sensors:
  lidar:
    enabled: true
    topic: "/scan"
    frame_id: "laser_link"
  camera:
    enabled: false
    topic: "/camera/image_raw"
```

### 3. 런치 파일에서 파라미터 로드
```xml
<launch>
    <!-- YAML 파일 로드 -->
    <rosparam file="$(find robot_control)/config/robot_params.yaml" />
    
    <!-- 개별 파라미터 설정 -->
    <param name="robot_ip" value="192.168.1.100" />
    
    <!-- 조건부 파라미터 -->
    <arg name="simulation" default="false" />
    <param if="$(arg simulation)" name="use_sim_time" value="true" />
    <param unless="$(arg simulation)" name="use_sim_time" value="false" />
</launch>
```

---

## 🔧 모듈화 설계

### 1. 노드 분리 원칙
```python
# ❌ 나쁜 예: 모든 기능이 하나의 노드에
class RobotController:
    def __init__(self):
        # 센서 처리
        # 안전 제어
        # 내비게이션
        # 통신
        # 로깅
        # ...

# ✅ 좋은 예: 기능별 노드 분리
class SensorProcessor:      # 센서 데이터 처리만
class SafetyController:     # 안전 제어만
class NavigationPlanner:   # 경로 계획만
class CommunicationHub:    # 통신만
class DataLogger:          # 로깅만
```

### 2. 토픽 기반 통신
```python
# 센서 처리 노드
class SensorProcessor:
    def __init__(self):
        rospy.init_node('sensor_processor')
        self.scan_pub = rospy.Publisher('/processed_scan', LaserScan)
        self.scan_sub = rospy.Subscriber('/raw_scan', LaserScan, self.process)

# 안전 제어 노드
class SafetyController:
    def __init__(self):
        rospy.init_node('safety_controller')
        self.cmd_vel_pub = rospy.Publisher('/safe_cmd_vel', Twist)
        self.scan_sub = rospy.Subscriber('/processed_scan', LaserScan, self.control)
```

### 3. 서비스 기반 제어
```python
# 설정 서비스
class ConfigService:
    def __init__(self):
        rospy.init_node('config_service')
        self.set_params_service = rospy.Service('/set_control_params', 
                                               SetControlParams, 
                                               self.set_params)

# 상태 조회 서비스
class StatusService:
    def __init__(self):
        rospy.init_node('status_service')
        self.get_status_service = rospy.Service('/get_robot_status', 
                                               GetRobotStatus, 
                                               self.get_status)
```

---

## 🐛 디버깅 및 모니터링

### 1. 디버깅 도구
```bash
# 노드 상태 확인
rosnode list
rosnode info /robot_controller

# 토픽 확인
rostopic list
rostopic echo /cmd_vel
rostopic hz /scan

# 서비스 확인
rosservice list
rosservice call /set_control_params "max_speed: 0.5"

# 파라미터 확인
rosparam list
rosparam get /robot_controller/max_speed
```

### 2. 시각화 도구
```bash
# 노드 그래프
rosrun rqt_graph rqt_graph

# 토픽 모니터
rosrun rqt_topic rqt_topic

# 파라미터 편집
rosrun rqt_reconfigure rqt_reconfigure

# RViz 시각화
rosrun rviz rviz
```

### 3. 로깅 및 모니터링
```python
# Python 로깅
import rospy

# 로그 레벨 설정
rospy.init_node('robot_controller', log_level=rospy.DEBUG)

# 로그 출력
rospy.loginfo("Robot started successfully")
rospy.logwarn("Low battery warning")
rospy.logerr("Emergency stop activated")
rospy.logdebug("Sensor data received: %s", sensor_data)
```

### 4. 데이터 수집
```bash
# ROS bag으로 데이터 수집
rosbag record -a -o robot_data

# 특정 토픽만 수집
rosbag record /scan /cmd_vel /odom -o sensor_data

# 데이터 재생
rosbag play robot_data.bag
```

---

## 🚀 배포 및 유지보수

### 1. 버전 관리
```bash
# Git을 사용한 버전 관리
git init
git add .
git commit -m "Initial ROS package setup"

# 태그를 사용한 버전 관리
git tag -a v1.0.0 -m "First stable release"
git push origin v1.0.0
```

### 2. 테스트
```python
# 단위 테스트
import unittest
import rospy
from robot_control.robot_controller import RobotController

class TestRobotController(unittest.TestCase):
    def setUp(self):
        rospy.init_node('test_robot_controller')
        self.controller = RobotController()
    
    def test_parameter_loading(self):
        self.assertEqual(self.controller.max_speed, 0.5)
    
    def test_safety_check(self):
        result = self.controller.check_safety(0.3)
        self.assertTrue(result)
```

### 3. 문서화
```python
# 코드 문서화
class RobotController:
    """
    TR200 로봇 제어 클래스
    
    이 클래스는 TR200 로봇의 기본적인 제어 기능을 제공합니다.
    
    Attributes:
        max_speed (float): 최대 속도 (m/s)
        safety_enabled (bool): 안전 기능 활성화 여부
        
    Example:
        >>> controller = RobotController()
        >>> controller.set_speed(0.3)
    """
    
    def set_speed(self, speed):
        """
        로봇 속도 설정
        
        Args:
            speed (float): 설정할 속도 (m/s)
            
        Raises:
            ValueError: 속도가 범위를 벗어날 때
        """
        if speed > self.max_speed:
            raise ValueError("Speed exceeds maximum limit")
        # ...
```

---

## 🏭 현업 모범 사례

### 1. 코드 품질
```python
# ✅ 좋은 예: 명확한 변수명과 함수명
def calculate_safe_distance(current_speed, reaction_time):
    """안전 거리 계산"""
    return current_speed * reaction_time + SAFETY_MARGIN

# ❌ 나쁜 예: 의미 없는 변수명
def calc_dist(speed, time):
    return speed * time + 0.5
```

### 2. 에러 처리
```python
# ✅ 좋은 예: 적절한 예외 처리
def connect_to_robot(self, ip, port):
    try:
        self.robot = WooshRobot(CommuSettings(addr=ip, port=port))
        connected = await self.robot.run()
        if not connected:
            raise ConnectionError(f"Failed to connect to robot at {ip}:{port}")
        rospy.loginfo(f"Successfully connected to robot at {ip}:{port}")
    except Exception as e:
        rospy.logerr(f"Robot connection error: {e}")
        raise
```

### 3. 성능 최적화
```python
# ✅ 좋은 예: 적절한 주파수 설정
rate = rospy.Rate(20)  # 20Hz - 센서 데이터 처리에 적합

# ✅ 좋은 예: 큐 크기 조정
pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)  # 제어 명령은 최신만

# ✅ 좋은 예: 메모리 효율적인 데이터 처리
def process_laser_scan(self, scan_msg):
    # 필요한 데이터만 추출
    ranges = [r for r in scan_msg.ranges if 0.1 < r < 10.0]
    return min(ranges) if ranges else float('inf')
```

### 4. 보안 고려사항
```python
# ✅ 좋은 예: 파라미터 검증
def validate_speed(self, speed):
    if not isinstance(speed, (int, float)):
        raise TypeError("Speed must be a number")
    if speed < 0 or speed > self.max_speed:
        raise ValueError(f"Speed must be between 0 and {self.max_speed}")
    return speed
```

### 5. 확장성 고려
```python
# ✅ 좋은 예: 플러그인 아키텍처
class SafetyPlugin:
    def check_safety(self, sensor_data):
        raise NotImplementedError

class ObstacleAvoidance(SafetyPlugin):
    def check_safety(self, sensor_data):
        # 장애물 회피 로직
        pass

class SpeedLimit(SafetyPlugin):
    def check_safety(self, sensor_data):
        # 속도 제한 로직
        pass
```

---

## 📚 추가 학습 자료

### 1. 공식 문서
- [ROS Wiki](http://wiki.ros.org/)
- [ROS Tutorials](http://wiki.ros.org/ROS/Tutorials)
- [ROS Best Practices](http://wiki.ros.org/BestPractices)

### 2. 개발 도구
- [rqt](http://wiki.ros.org/rqt) - ROS 개발 도구 모음
- [rosbag](http://wiki.ros.org/rosbag) - 데이터 수집 및 재생
- [rostest](http://wiki.ros.org/rostest) - 테스트 프레임워크

### 3. 커뮤니티
- [ROS Discourse](https://discourse.ros.org/)
- [ROS Answers](https://answers.ros.org/)
- [GitHub ROS Packages](https://github.com/ros)

---

## 🎯 체크리스트

### 개발 시작 전
- [ ] 워크스페이스 구조 설계
- [ ] 패키지 의존성 확인
- [ ] 네이밍 컨벤션 결정
- [ ] 버전 관리 설정

### 개발 중
- [ ] 모듈화 설계 적용
- [ ] 적절한 로깅 구현
- [ ] 에러 처리 구현
- [ ] 파라미터 검증

### 배포 전
- [ ] 테스트 코드 작성
- [ ] 문서화 완료
- [ ] 성능 최적화
- [ ] 보안 검토

이 가이드를 참고하여 체계적이고 확장 가능한 ROS 시스템을 구축하세요! 🚀
