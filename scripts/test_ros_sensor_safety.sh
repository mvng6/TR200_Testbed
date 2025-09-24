#!/bin/bash
# TR200 ROS 센서 기반 안전 제어 시스템 테스트 스크립트

echo "🚀 TR200 ROS 센서 기반 안전 제어 시스템 테스트 시작"
echo "=================================================="

# ROS 환경 설정
source /opt/ros/noetic/setup.bash
source /catkin_ws/devel/setup.bash

# 로봇 IP 확인
ROBOT_IP="169.254.128.2"
echo "🔍 로봇 연결 확인 중... (IP: $ROBOT_IP)"

# 로봇 연결 테스트
if ping -c 1 $ROBOT_IP > /dev/null 2>&1; then
    echo "✅ 로봇 연결 확인됨"
else
    echo "❌ 로봇 연결 실패. 로봇이 켜져 있고 네트워크가 연결되어 있는지 확인하세요."
    exit 1
fi

# ROS 마스터 시작 확인
echo "🔍 ROS 마스터 상태 확인 중..."
if rostopic list > /dev/null 2>&1; then
    echo "✅ ROS 마스터 실행 중"
else
    echo "⚠️ ROS 마스터가 실행되지 않음. roscore를 시작합니다..."
    roscore &
    sleep 3
fi

# 워크스페이스 빌드
echo "🔨 워크스페이스 빌드 중..."
cd /catkin_ws
catkin_make
if [ $? -eq 0 ]; then
    echo "✅ 워크스페이스 빌드 성공"
else
    echo "❌ 워크스페이스 빌드 실패"
    exit 1
fi

# 환경 재설정
source devel/setup.bash

echo ""
echo "🎯 테스트 옵션을 선택하세요:"
echo "1) 기본 센서 안전 제어 실행"
echo "2) RViz 시각화와 함께 실행"
echo "3) 키보드 텔레옵과 함께 실행"
echo "4) 파라미터 테스트"
echo "5) 서비스 테스트"
echo "6) 종료"
echo ""

read -p "선택 (1-6): " choice

case $choice in
    1)
        echo "🚀 기본 센서 안전 제어 실행 중..."
        roslaunch tr200_simple_control tr200_sensor_safety_controller.launch
        ;;
    2)
        echo "🚀 RViz 시각화와 함께 실행 중..."
        roslaunch tr200_simple_control tr200_sensor_safety_controller.launch use_rviz:=true
        ;;
    3)
        echo "🚀 키보드 텔레옵과 함께 실행 중..."
        roslaunch tr200_simple_control tr200_sensor_safety_controller.launch use_teleop:=true
        ;;
    4)
        echo "🔧 파라미터 테스트 중..."
        echo "현재 파라미터 설정:"
        rosparam list | grep tr200_sensor_safety_controller
        echo ""
        echo "파라미터 값 확인:"
        rosparam get /tr200_sensor_safety_controller/min_obstacle_distance
        rosparam get /tr200_sensor_safety_controller/warning_distance
        rosparam get /tr200_sensor_safety_controller/normal_speed
        ;;
    5)
        echo "🔧 서비스 테스트 중..."
        echo "사용 가능한 서비스:"
        rosservice list | grep tr200_sensor_safety_controller
        echo ""
        echo "안전 파라미터 설정 서비스 테스트:"
        rosservice call /set_safety_params "min_obstacle_distance: 0.3
warning_distance: 0.6
safe_distance: 0.9
normal_speed: 0.15
slow_speed: 0.05"
        ;;
    6)
        echo "👋 테스트 종료"
        exit 0
        ;;
    *)
        echo "❌ 잘못된 선택입니다."
        exit 1
        ;;
esac

echo ""
echo "✅ 테스트 완료"
