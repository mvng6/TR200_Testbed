#!/bin/bash
# TR200 ROS Docker 이미지 빌드 스크립트

set -e

echo "🔨 TR200 ROS Docker 이미지 빌드 스크립트"
echo "========================================"
echo ""

# 현재 스크립트 위치 확인
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"

echo "📁 프로젝트 루트: $PROJECT_DIR"

# 기존 컨테이너 정리
if docker ps -a | grep -q tr200_control_container; then
    echo "🧹 기존 컨테이너를 정리합니다..."
    docker stop tr200_control_container > /dev/null 2>&1 || true
    docker rm tr200_control_container > /dev/null 2>&1 || true
fi

# 기존 이미지 정리 (선택사항)
if docker images | grep -q tr200_ros_noetic; then
    echo "🧹 기존 이미지를 정리합니다..."
    docker rmi tr200_ros_noetic:latest > /dev/null 2>&1 || true
fi

# Docker 이미지 빌드 (run_container.sh와 동일한 방식)
echo "🔨 Docker 이미지 빌드 중..."
docker build -t tr200_ros_noetic:latest -f "$PROJECT_DIR/docker/Dockerfile" "$PROJECT_DIR"

echo "✅ Docker 이미지 빌드 완료!"
echo ""
echo "💡 다음 명령어로 컨테이너를 실행하세요:"
echo "   ./scripts/run_container.sh"
