# TR200 로봇 제어 시스템 - Docker 기반 ROS Noetic 환경

## 📋 프로젝트 개요

이 프로젝트는 Docker를 활용하여 Ubuntu 20.04 + ROS Noetic 환경을 구축하고, TR200 로봇을 ROS와 Woosh SDK를 통해 제어하는 통합 시스템을 개발하는 것을 목표로 합니다.

### 🎯 최종 목표
- 노트북(Ubuntu 22.04 + ROS Humble)에서 Docker 컨테이너(Ubuntu 20.04 + ROS Noetic) 실행
- TR200 로봇과 네트워크 통신을 통한 원격 제어
- 좌표 기반 내비게이션 및 속도 제어 명령 실행
- ROS 토픽/서비스와 Woosh SDK의 통합 활용

### 🏗️ 시스템 아키텍처
```
Host System (Ubuntu 22.04 + ROS Humble)
├── Docker Container (Ubuntu 20.04 + ROS Noetic)
│   ├── Woosh Robot SDK
│   ├── ROS Noetic Packages
│   ├── TR200 Control Nodes
│   └── Custom Control Scripts
└── Network Bridge → TR200 Robot (169.254.128.2:5480)
```

## 📁 프로젝트 구조

```
tr200_ros_docker_project/
├── PROJECT.md                 # 이 파일
├── docker/
│   ├── Dockerfile             # ROS Noetic + SDK 환경 이미지
│   ├── docker-compose.yml     # 컨테이너 오케스트레이션
│   └── entrypoint.sh          # 컨테이너 시작 스크립트
├── src/
│   ├── woosh_robot_py/        # Woosh SDK (복사본)
│   ├── tr200_control/         # ROS 패키지: TR200 제어
│   ├── tr200_navigation/      # ROS 패키지: 내비게이션
│   └── tr200_interface/       # ROS 패키지: SDK-ROS 브리지
├── launch/
│   ├── tr200_bringup.launch   # 전체 시스템 런치
│   ├── tr200_control.launch   # 제어 노드만 런치
│   └── tr200_navigation.launch # 내비게이션 노드만 런치
├── config/
│   ├── robot_params.yaml      # 로봇 파라미터
│   └── network_config.yaml    # 네트워크 설정
├── scripts/
│   ├── build_docker.sh        # Docker 이미지 빌드
│   ├── run_container.sh       # 컨테이너 실행
│   └── setup_workspace.sh     # 워크스페이스 설정
└── docs/
    ├── SETUP.md               # 설치 및 설정 가이드
    ├── USAGE.md               # 사용법 가이드
    └── TROUBLESHOOTING.md     # 문제 해결 가이드
```

## 🚀 개발 단계별 계획

### Phase 1: Docker 환경 구축 (1-2일)
- [ ] **1.1 Docker 기본 환경 설정**
  - Ubuntu 20.04 기반 이미지 선택
  - ROS Noetic 설치 스크립트 작성
  - 필수 패키지 및 의존성 설치

- [ ] **1.2 Woosh SDK 통합**
  - SDK 소스코드를 컨테이너에 복사
  - Python 의존성 설치 (requirements.txt)
  - SDK 동작 테스트

- [ ] **1.3 네트워크 설정**
  - 호스트-컨테이너 간 네트워크 브리지 설정
  - TR200 로봇과의 통신 테스트 (169.254.128.2:5480)
  - 포트 포워딩 및 방화벽 설정

### Phase 2: ROS 패키지 개발 (2-3일)
- [ ] **2.1 tr200_interface 패키지**
  - Woosh SDK와 ROS 간 브리지 노드 개발
  - SDK 명령을 ROS 토픽/서비스로 변환
  - 로봇 상태를 ROS 메시지로 퍼블리시

- [ ] **2.2 tr200_control 패키지**
  - 속도 제어 노드 (cmd_vel 토픽 구독)
  - 로봇 상태 모니터링 노드
  - 안전 기능 (충돌 방지, 속도 제한)

- [ ] **2.3 tr200_navigation 패키지**
  - 좌표 기반 내비게이션 노드
  - 목표점 설정 서비스
  - 경로 계획 및 실행 상태 피드백

### Phase 3: 통합 및 테스트 (1-2일)
- [ ] **3.1 시스템 통합**
  - 모든 노드를 통합하는 launch 파일 작성
  - 파라미터 설정 및 최적화
  - 로그 및 디버깅 시스템 구축

- [ ] **3.2 기능 테스트**
  - 기본 연결 및 통신 테스트
  - 속도 제어 명령 테스트
  - 좌표 기반 내비게이션 테스트
  - 에러 처리 및 복구 테스트

- [ ] **3.3 사용자 인터페이스**
  - 명령줄 도구 개발
  - RViz 시각화 설정
  - 간단한 GUI 도구 (선택사항)

### Phase 4: 문서화 및 배포 (1일)
- [ ] **4.1 문서 작성**
  - 설치 가이드 (SETUP.md)
  - 사용법 가이드 (USAGE.md)
  - 문제 해결 가이드 (TROUBLESHOOTING.md)

- [ ] **4.2 배포 준비**
  - Docker 이미지 최적화
  - 자동화 스크립트 완성
  - 예제 코드 및 데모 준비

## 🛠️ 기술 스택

### 컨테이너 환경
- **OS**: Ubuntu 20.04 LTS
- **ROS**: ROS Noetic Ninjemys
- **Python**: 3.8+
- **Docker**: Latest stable version

### 주요 라이브러리
- **Woosh Robot SDK**: 로봇 제어 및 통신
- **rospy**: ROS Python 클라이언트
- **geometry_msgs**: ROS 기하학적 메시지
- **nav_msgs**: ROS 내비게이션 메시지
- **std_msgs**: ROS 표준 메시지

### 네트워크 프로토콜
- **WebSocket**: SDK-로봇 간 통신
- **TCP/IP**: ROS 노드 간 통신
- **Protocol Buffers**: 메시지 직렬화

## 📋 요구사항

### 하드웨어 요구사항
- **노트북**: Ubuntu 22.04 + ROS Humble 설치됨
- **TR200 로봇**: 네트워크 연결 가능 (IP: 169.254.128.2)
- **네트워크**: 로봇과 노트북 간 통신 가능한 환경

### 소프트웨어 요구사항
- **Docker**: 설치 완료
- **Docker Compose**: 설치 완료
- **Git**: 소스코드 관리용
- **텍스트 에디터**: 코드 편집용

## 🎯 핵심 기능

### 1. 로봇 제어 기능
```bash
# 속도 제어
rostopic pub /cmd_vel geometry_msgs/Twist "linear: {x: 0.5} angular: {z: 0.2}"

# 좌표 이동
rosservice call /move_to_goal "x: 1.5 y: 2.0 theta: 1.57"

# 로봇 상태 확인
rostopic echo /robot_status
```

### 2. 모니터링 기능
```bash
# 실시간 위치 확인
rostopic echo /robot_pose

# 배터리 상태 확인
rostopic echo /battery_status

# 작업 진행 상태 확인
rostopic echo /task_status
```

### 3. 안전 기능
- 충돌 방지 시스템
- 속도 제한 및 가속도 제한
- 비상 정지 기능
- 연결 끊김 감지 및 자동 재연결

## 🔧 개발 환경 설정

### 1. 프로젝트 클론 및 초기 설정
```bash
cd /home/ldj/tr200_ws
git clone <repository_url> tr200_ros_docker_project
cd tr200_ros_docker_project
chmod +x scripts/*.sh
```

### 2. Docker 이미지 빌드
```bash
./scripts/build_docker.sh
```

### 3. 컨테이너 실행
```bash
./scripts/run_container.sh
```

### 4. ROS 워크스페이스 설정
```bash
# 컨테이너 내부에서 실행
./scripts/setup_workspace.sh
```

## 📊 진행 상황 추적

### 완료된 작업
- [x] 프로젝트 계획 수립
- [x] 요구사항 분석
- [x] 시스템 아키텍처 설계

### 진행 중인 작업
- [ ] Docker 환경 구축
- [ ] ROS 패키지 개발
- [ ] 시스템 통합

### 예정된 작업
- [ ] 테스트 및 검증
- [ ] 문서화
- [ ] 배포 준비

## 🚨 주의사항

1. **네트워크 설정**: TR200 로봇과의 네트워크 연결이 안정적이어야 합니다.
2. **포트 충돌**: 호스트의 ROS Humble과 컨테이너의 ROS Noetic 간 포트 충돌 방지가 필요합니다.
3. **권한 관리**: Docker 컨테이너 내에서 네트워크 및 디바이스 접근 권한 설정이 중요합니다.
4. **버전 호환성**: Woosh SDK와 ROS Noetic 간 Python 버전 호환성을 확인해야 합니다.

## 📞 지원 및 문의

프로젝트 진행 중 문제가 발생하면 다음을 참조하세요:
- `docs/TROUBLESHOOTING.md`: 일반적인 문제 해결 방법
- `docs/SETUP.md`: 상세한 설치 가이드
- `docs/USAGE.md`: 사용법 및 예제

---

**프로젝트 시작일**: 2025년 9월 23일  
**예상 완료일**: 2025년 9월 29일 (6일)  
**개발자**: ldj  
**버전**: v1.0.0-dev
