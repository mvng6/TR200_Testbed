# TR200 센서 기반 안전 제어 시스템

## 📋 프로젝트 개요

TR200 로봇의 Lidar 센서를 활용한 실시간 장애물 감지 및 안전 제어 시스템입니다. Docker 기반 ROS Noetic 환경에서 Woosh SDK를 통해 TR200 로봇을 제어합니다.

## 🎯 핵심 기능

- **실시간 장애물 감지**: TR200의 Lidar 센서를 활용한 360도 장애물 감지
- **지능형 속도 제어**: 거리에 따른 자동 속도 조절 (정상 → 감속 → 정지)
- **안전 우선 제어**: 위험 상황에서 즉시 비상 정지
- **부드러운 제어**: 급격한 가속/감속 방지
- **Docker 기반 환경**: Ubuntu 20.04 + ROS Noetic 환경에서 안정적 실행

## 📁 프로젝트 구조

```
tr200_ros_docker_project/
├── README.md                         # 프로젝트 문서
├── scripts/                          # 실행 스크립트
│   ├── build_docker.sh               # Docker 이미지 빌드
│   ├── run_container.sh              # Docker 컨테이너 실행
│   ├── run_sensor_controller.sh      # 센서 기반 제어 실행
│   ├── setup_ros.sh                  # ROS 환경 설정
│   └── archive/                      # 이전 버전 스크립트들
├── src/tr200_simple_control/scripts/ # Python 제어 스크립트
│   ├── sensor_based_safety_controller.py    # 기본 센서 기반 제어
│   ├── advanced_sensor_controller.py       # 고급 센서 기반 제어
│   ├── simple_linear_motion.py             # 기본 왕복 운동
│   └── archive/                             # 개발 과정 파일들
├── src/woosh_robot_py/              # Woosh SDK
│   ├── README.md                    # SDK 문서
│   ├── examples/                    # 예제 코드
│   └── woosh/                       # SDK 핵심 모듈
├── config/                           # 설정 파일
│   ├── area_motion_params.yaml       # 영역 제한 구동 파라미터
│   ├── robot_params.yaml            # 로봇 기본 파라미터
│   └── test_safe_params.yaml        # 테스트용 안전 파라미터
├── docker/                           # Docker 환경
│   ├── Dockerfile                    # Docker 이미지 정의
│   ├── docker-compose.yml           # 컨테이너 오케스트레이션
│   └── entrypoint.sh                # 컨테이너 시작 스크립트
├── launch/                           # ROS 런치 파일
└── archive/                          # 이전 버전 파일들
```

## 🚀 사용 방법

### 1. Docker 컨테이너 실행 (첫 번째 터미널)
```bash
cd /home/ldj/tr200_ws/tr200_ros_docker_project
./scripts/run_container.sh
```

### 2. 컨테이너 진입 방법

#### 방법 1: run_docker_tr200 명령어 사용 (권장)
```bash
# 두 번째 터미널에서
run_docker_tr200
```

#### 방법 2: docker exec 명령어 사용
```bash
# 두 번째 터미널에서
docker exec -it tr200_control_container /bin/bash
```

### 3. 센서 기반 안전 제어 실행
```bash
# 컨테이너 내부에서 (자동으로 ROS 환경이 설정됨)
./scripts/run_sensor_controller.sh

# 또는 직접 실행
python3 src/tr200_simple_control/scripts/sensor_based_safety_controller.py

# 또는 고급 센서 기반 제어
python3 src/tr200_simple_control/scripts/advanced_sensor_controller.py

# 또는 기본 왕복 운동 (참고용)
python3 src/tr200_simple_control/scripts/simple_linear_motion.py
```

### 4. ROS 환경 재설정 (필요한 경우)
```bash
# 컨테이너 내부에서
./scripts/setup_ros.sh
```

## ⚙️ 설정 파라미터

### 센서 기반 안전 제어 설정
- **최소 장애물 거리**: 0.3m (이 거리 이하에서 즉시 정지)
- **경고 거리**: 0.5m (이 거리에서 감속 시작)
- **안전 거리**: 0.8m (이 거리 이상에서 정상 속도)

### 제어 설정
- **정상 속도**: 0.2 m/s
- **감속 속도**: 0.1 m/s
- **정지 속도**: 0.0 m/s
- **제어 주파수**: 20Hz

### 설정 파일들
- **`config/area_motion_params.yaml`**: 영역 제한 구동 파라미터 (기본 설정)
- **`config/robot_params.yaml`**: 로봇 기본 파라미터
- **`config/test_safe_params.yaml`**: 테스트용 안전 파라미터 (더 보수적 설정)

## 📊 예상 동작

```
🎯 센서 기반 안전 제어 시스템을 시작합니다!
   최소 장애물 거리: 0.3m
   경고 거리: 0.5m
   안전 거리: 0.8m
   정상 속도: 0.2 m/s
   감속 속도: 0.1 m/s

08:15:23 | 방향: forward | 속도: 0.200 | 거리: 1.250m | 상태: 🟢 안전
08:15:24 | 방향: forward | 속도: 0.200 | 거리: 0.650m | 상태: 🟢 안전
08:15:25 | 방향: forward | 속도: 0.200 | 거리: 0.450m | 상태: 🟡 주의
⚠️ 주의: 장애물 근접 (거리: 0.450m)
08:15:26 | 방향: forward | 속도: 0.100 | 거리: 0.350m | 상태: 🟡 주의
08:15:27 | 방향: forward | 속도: 0.100 | 거리: 0.280m | 상태: 🔴 위험
🚨 위험! 장애물 감지 (거리: 0.280m)
🚨 비상 정지! 이유: 장애물 감지 (거리: 0.280m)
✅ 비상 정지 완료
```

## 🔧 파일 설명

### 핵심 파일들
- **`sensor_based_safety_controller.py`**: 기본 센서 기반 안전 제어 시스템 (Lidar 센서 활용)
- **`advanced_sensor_controller.py`**: 고급 센서 기반 제어 (섹터별 분석, 지능형 회피)
- **`simple_linear_motion.py`**: 기본 왕복 운동 (참고용, 센서 없이 단순 이동)

### 실행 스크립트들
- **`run_container.sh`**: Docker 컨테이너 실행
- **`run_sensor_controller.sh`**: 센서 기반 제어 실행
- **`build_docker.sh`**: Docker 이미지 빌드
- **`setup_ros.sh`**: ROS 환경 설정

### 설정 파일들
- **`area_motion_params.yaml`**: 영역 제한 구동 파라미터 (기본 설정)
- **`robot_params.yaml`**: 로봇 기본 파라미터
- **`test_safe_params.yaml`**: 테스트용 안전 파라미터

### Archive 파일들 (개발 과정)
- **`bypass_taskable_test.py`**: kTaskable 상태 우회 테스트
- **`integrated_safety_controller.py`**: 위치 기반 영역 제한 제어
- **`tr200_area_constrained_motion.py`**: 원본 영역 제한 구동 시스템
- **`real_time_monitor.py`**: 실시간 모니터링 시스템
- **`simple_monitor.py`**: 간단한 모니터링
- **`force_test_mode.py`**: 강제 테스트 모드
- **`detailed_robot_diagnosis.py`**: 로봇 진단 도구
- **`velocity_bridge.py`**: ROS 브리지 (미완성)

## 🎯 개발 목표 달성

✅ **특정 영역 내에서만 앞뒤 구동**: 위치 기반 영역 제한 시스템 구현  
✅ **네비게이션 없이 직접 구동**: 속도 제어 기반 시스템 구현  
✅ **센서 기반 장애물 감지**: TR200 Lidar 센서 활용한 실시간 장애물 감지  
✅ **안전한 구동**: 거리 기반 자동 속도 조절 및 비상 정지  

## 📞 지원

문제가 발생하거나 추가 기능이 필요한 경우:
1. 로그 파일 확인 (`docker/logs/` 디렉토리)
2. 설정 파일 검토 (`config/` 디렉토리)
3. Archive 폴더의 이전 버전 참조 (`archive/` 디렉토리)
4. Woosh SDK 문서 참조 (`src/woosh_robot_py/README.md`)

### 주요 문제 해결
- **로봇 연결 실패**: TR200 앱에서 로봇 상태 확인 (비상정지 해제, 작업 모드 설정)
- **kTaskable 상태 문제**: `archive/force_test_mode.py` 참조
- **센서 데이터 없음**: TR200의 Lidar 센서 상태 확인

---

**개발자**: ldj  
**버전**: v2.0.0 (센서 기반 안전 제어)  
**최종 업데이트**: 2024년 1월 15일
