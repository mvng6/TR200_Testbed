# TR200 로봇 상태 메시지
# 로봇의 전체 상태 정보를 전달

Header header

# 연결 상태
bool connected             # 로봇 연결 상태
string connection_status   # 연결 상태 설명
float32 connection_uptime  # 연결 지속 시간 (초)

# 로봇 정보
string robot_id           # 로봇 ID
string robot_mode         # 로봇 모드
string operation_state    # 작업 상태

# 위치 및 속도
geometry_msgs/Pose2D pose # 현재 위치 (x, y, theta)
geometry_msgs/Twist velocity # 현재 속도 (linear, angular)

# 배터리 정보
float32 battery_level     # 배터리 잔량 (%)
string battery_status     # 배터리 상태

# 시스템 정보
float32 cpu_usage         # CPU 사용률 (%)
float32 memory_usage      # 메모리 사용률 (%)
string system_status      # 시스템 상태
