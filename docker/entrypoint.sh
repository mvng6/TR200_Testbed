#!/bin/bash

# TR200 ROS Docker 컨테이너 엔트리포인트 스크립트

set -e

echo "🚀 TR200 ROS Noetic 컨테이너를 시작합니다..."

# ROS 환경 설정
source /opt/ros/noetic/setup.bash

# Catkin 워크스페이스가 빌드되어 있다면 소스
if [ -f "/catkin_ws/devel/setup.bash" ]; then
    echo "📦 Catkin 워크스페이스를 로드합니다..."
    source /catkin_ws/devel/setup.bash
fi

# ROS 마스터 상태 확인 및 시작
echo "🔍 ROS 마스터 상태를 확인합니다..."
if ! pgrep -f "rosmaster" > /dev/null; then
    echo "🎯 ROS 마스터를 시작합니다..."
    roscore &
    sleep 3
fi

# TR200 로봇 연결 테스트
echo "🤖 TR200 로봇 연결을 테스트합니다..."
TR200_IP=${TR200_ROBOT_IP:-169.254.128.2}
TR200_PORT=${TR200_ROBOT_PORT:-5480}

if ping -c 1 -W 3 $TR200_IP > /dev/null 2>&1; then
    echo "✅ TR200 로봇 ($TR200_IP)에 연결 가능합니다."
    
    # 포트 연결 테스트
    if nc -z -w3 $TR200_IP $TR200_PORT > /dev/null 2>&1; then
        echo "✅ TR200 로봇 포트 ($TR200_PORT)가 열려있습니다."
    else
        echo "⚠️  TR200 로봇 포트 ($TR200_PORT)에 연결할 수 없습니다."
    fi
else
    echo "❌ TR200 로봇 ($TR200_IP)에 연결할 수 없습니다."
    echo "   네트워크 설정을 확인해주세요."
fi

# 워크스페이스 권한 설정 (현재 사용자로)
if [ -d "/catkin_ws" ]; then
    echo "🔧 워크스페이스 권한을 설정합니다..."
    chown -R $(id -u):$(id -g) /catkin_ws 2>/dev/null || true
fi

# 개발 도구 설정
echo "🛠️  개발 환경을 설정합니다..."

# Git 설정 (개발용) - 현재 사용자 홈 디렉토리 사용
if [ ! -f "$HOME/.gitconfig" ]; then
    git config --global user.name "TR200 Developer"
    git config --global user.email "developer@tr200.local"
    git config --global init.defaultBranch main
fi

# 유용한 별칭 설정 - 현재 사용자의 bashrc에 추가
if ! grep -q "# TR200 aliases" "$HOME/.bashrc"; then
    cat >> "$HOME/.bashrc" << 'EOF'

# TR200 aliases
alias tr200_build='cd /catkin_ws && catkin_make'
alias tr200_source='source /catkin_ws/devel/setup.bash'
alias tr200_test='python3 /opt/woosh_robot_py/examples/monitor_ko.py --ip $TR200_ROBOT_IP --port $TR200_ROBOT_PORT'
alias tr200_move='python3 /opt/woosh_robot_py/examples/tr200_move_to_goal.py --ip $TR200_ROBOT_IP --port $TR200_ROBOT_PORT'
alias ros_topics='rostopic list'
alias ros_nodes='rosnode list'
EOF
fi

echo "✅ 컨테이너 초기화가 완료되었습니다!"
echo ""
echo "🎯 사용 가능한 명령어:"
echo "  tr200_build  - 워크스페이스 빌드"
echo "  tr200_test   - 로봇 연결 테스트"
echo "  tr200_move   - 로봇 이동 명령"
echo "  ros_topics   - ROS 토픽 목록"
echo "  ros_nodes    - ROS 노드 목록"
echo ""

# 전달된 명령 실행
exec "$@"