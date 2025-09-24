#!/bin/bash

# 간단한 ROS 환경 설정 스크립트

echo "🔧 ROS 환경을 설정합니다..."

# ROS 환경 로드
source /opt/ros/noetic/setup.bash

# 워크스페이스로 이동
cd /catkin_ws

# devel 환경 설정 (존재하는 경우)
if [ -f "devel/setup.bash" ]; then
    source devel/setup.bash
    echo "✅ 기존 워크스페이스 환경 로드 완료"
else
    echo "⚠️  워크스페이스가 빌드되지 않았습니다. catkin_make를 실행하세요."
fi

# PYTHONPATH 설정
export PYTHONPATH="/catkin_ws/src/woosh_robot_py:${PYTHONPATH}"

echo "✅ ROS 환경 설정 완료!"
echo "📁 현재 위치: $(pwd)"
echo "📁 워크스페이스 경로: /catkin_ws"
echo "🚀 센서 기반 제어 실행: ./scripts/run_sensor_controller.sh"