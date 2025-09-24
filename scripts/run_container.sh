#!/bin/bash

# TR200 Docker 컨테이너 실행 스크립트

set -e

# 색상 정의
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

echo -e "${BLUE}🚀 TR200 ROS Noetic 컨테이너를 실행합니다...${NC}"
echo -e "${BLUE}=============================================${NC}"
echo ""

# 현재 스크립트 위치 확인
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"

echo -e "${YELLOW}📁 프로젝트 루트: $PROJECT_DIR${NC}"

# 환경 변수 설정
export USER_ID=$(id -u)
export GROUP_ID=$(id -g)
export DISPLAY=${DISPLAY:-:0}

# X11 권한 설정 (GUI 애플리케이션용)
xhost +local:docker > /dev/null 2>&1 || echo -e "${YELLOW}⚠️  X11 권한 설정을 건너뜁니다.${NC}"

# 기존 컨테이너 정리
if docker ps -a | grep -q tr200_control_container; then
    echo -e "${YELLOW}🧹 기존 컨테이너를 정리합니다...${NC}"
    docker stop tr200_control_container > /dev/null 2>&1 || true
    docker rm tr200_control_container > /dev/null 2>&1 || true
fi

# 로그 및 데이터 디렉토리 생성
mkdir -p "$PROJECT_DIR/docker/logs"
mkdir -p "$PROJECT_DIR/docker/data"

echo -e "${YELLOW}🐳 컨테이너를 시작합니다...${NC}"

# Docker 컨테이너 실행
docker run -it \
    --name tr200_control_container \
    --hostname tr200-docker \
    --network host \
    --privileged \
    -e DISPLAY=$DISPLAY \
    -e ROS_HOSTNAME=localhost \
    -e ROS_MASTER_URI=http://localhost:11311 \
    -e TR200_ROBOT_IP=169.254.128.2 \
    -e TR200_ROBOT_PORT=5480 \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    -v "$PROJECT_DIR/src":/catkin_ws/src:rw \
    -v "$PROJECT_DIR/launch":/catkin_ws/launch:rw \
    -v "$PROJECT_DIR/config":/catkin_ws/config:rw \
    -v "$PROJECT_DIR/scripts":/catkin_ws/scripts:rw \
    -v "$PROJECT_DIR/docker/logs":/catkin_ws/logs:rw \
    -v "$PROJECT_DIR/docker/data":/catkin_ws/data:rw \
    tr200_ros_noetic:latest \
    bash -c "
        echo -e '${BLUE}🔧 ROS 환경을 자동 설정합니다...${NC}'
        cd /catkin_ws
        source /opt/ros/noetic/setup.bash
        source devel/setup.bash 2>/dev/null || echo -e '${YELLOW}⚠️  devel/setup.bash가 없습니다. catkin_make를 실행하세요.${NC}'
        export PYTHONPATH=\"/catkin_ws/src/woosh_robot_py:\${PYTHONPATH}\"       
        
        echo -e '${GREEN}✅ ROS 환경 설정 완료!${NC}'
        echo -e '${BLUE}🚀 ROS 환경 재설정: ./scripts/setup_ros.sh${NC}'
        echo -e '${BLUE}📁 현재 위치: \$(pwd)${NC}'
        echo ''
        exec bash
    "

echo -e "${GREEN}✅ 컨테이너가 종료되었습니다.${NC}"

# X11 권한 복원
xhost -local:docker > /dev/null 2>&1 || true