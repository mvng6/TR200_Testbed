#!/bin/bash

# TR200 Docker 이미지 빌드 스크립트

set -e

echo "🐳 TR200 ROS Noetic Docker 이미지를 빌드합니다..."

# 현재 스크립트 위치 확인
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"

echo "📁 프로젝트 디렉토리: $PROJECT_DIR"

# Woosh SDK 복사 확인
if [ ! -d "$PROJECT_DIR/src/woosh_robot_py" ]; then
    echo "📦 Woosh SDK를 복사합니다..."
    cp -r /home/ldj/tr200_ws/Mobile-Robot-TR200/woosh_robot_py "$PROJECT_DIR/src/"
    echo "✅ Woosh SDK 복사 완료"
fi

# 사용자 ID/GID 설정 (권한 문제 방지)
export USER_ID=$(id -u)
export GROUP_ID=$(id -g)

echo "👤 사용자 ID: $USER_ID, 그룹 ID: $GROUP_ID"

# Docker 이미지 빌드
cd "$PROJECT_DIR"

echo "🔨 Docker 이미지를 빌드합니다..."
docker build \
    --build-arg USER_ID=$USER_ID \
    --build-arg GROUP_ID=$GROUP_ID \
    -t tr200_ros_noetic:latest \
    -f docker/Dockerfile \
    .

echo "✅ Docker 이미지 빌드가 완료되었습니다!"

# 이미지 정보 출력
echo ""
echo "📊 빌드된 이미지 정보:"
docker images | grep tr200_ros_noetic

echo ""
echo "🚀 다음 단계:"
echo "  ./scripts/run_container.sh  # 컨테이너 실행"
echo "  또는"
echo "  docker-compose -f docker/docker-compose.yml up -d  # Docker Compose로 실행"
