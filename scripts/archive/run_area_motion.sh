#!/bin/bash
# tr200_area_motion.sh
# TR200 영역 제한 구동 실행 스크립트

# 스크립트 디렉토리로 이동
cd "$(dirname "$0")/../.."

# 색상 정의
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

echo -e "${BLUE}========================================${NC}"
echo -e "${BLUE}  TR200 영역 제한 구동 시스템 시작${NC}"
echo -e "${BLUE}========================================${NC}"

# 설정 파일 경로
CONFIG_FILE="config/area_motion_params.yaml"
SCRIPT_FILE="src/tr200_simple_control/scripts/tr200_area_constrained_motion.py"

# 설정 파일 존재 확인
if [ ! -f "$CONFIG_FILE" ]; then
    echo -e "${RED}❌ 설정 파일을 찾을 수 없습니다: $CONFIG_FILE${NC}"
    exit 1
fi

# 스크립트 파일 존재 확인
if [ ! -f "$SCRIPT_FILE" ]; then
    echo -e "${RED}❌ 스크립트 파일을 찾을 수 없습니다: $SCRIPT_FILE${NC}"
    exit 1
fi

# 로그 디렉토리 생성
mkdir -p logs

echo -e "${GREEN}✅ 파일 확인 완료${NC}"
echo -e "${YELLOW}📁 설정 파일: $CONFIG_FILE${NC}"
echo -e "${YELLOW}📁 스크립트 파일: $SCRIPT_FILE${NC}"
echo -e "${YELLOW}📁 로그 디렉토리: logs/${NC}"

# Python 환경 확인
echo -e "${BLUE}🔍 Python 환경 확인 중...${NC}"
python3 --version
if [ $? -ne 0 ]; then
    echo -e "${RED}❌ Python3가 설치되어 있지 않습니다.${NC}"
    exit 1
fi

# 필요한 패키지 확인
echo -e "${BLUE}🔍 필요한 패키지 확인 중...${NC}"
python3 -c "import yaml, asyncio" 2>/dev/null
if [ $? -ne 0 ]; then
    echo -e "${YELLOW}⚠️ 필요한 패키지가 설치되어 있지 않습니다. 설치를 시도합니다...${NC}"
    pip3 install pyyaml
fi

# Woosh SDK 경로 확인
WOOSH_SDK_PATH="src/woosh_robot_py"
if [ ! -d "$WOOSH_SDK_PATH" ]; then
    echo -e "${RED}❌ Woosh SDK를 찾을 수 없습니다: $WOOSH_SDK_PATH${NC}"
    echo -e "${YELLOW}💡 Mobile-Robot-TR200/woosh_robot_py를 src/로 복사해주세요.${NC}"
    exit 1
fi

# PYTHONPATH 설정
export PYTHONPATH="${PWD}/src/woosh_robot_py:${PYTHONPATH}"

echo -e "${GREEN}✅ 환경 설정 완료${NC}"
echo -e "${BLUE}🚀 TR200 영역 제한 구동을 시작합니다...${NC}"
echo -e "${YELLOW}💡 중지하려면 Ctrl+C를 누르세요.${NC}"
echo ""

# 스크립트 실행
python3 "$SCRIPT_FILE" "$CONFIG_FILE"

# 실행 결과 확인
if [ $? -eq 0 ]; then
    echo -e "${GREEN}✅ 프로그램이 정상적으로 종료되었습니다.${NC}"
else
    echo -e "${RED}❌ 프로그램 실행 중 오류가 발생했습니다.${NC}"
fi

echo -e "${BLUE}========================================${NC}"
echo -e "${BLUE}  TR200 영역 제한 구동 시스템 종료${NC}"
echo -e "${BLUE}========================================${NC}"
