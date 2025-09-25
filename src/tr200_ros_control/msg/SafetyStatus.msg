# TR200 안전 상태 메시지
# 안전 제어 시스템의 상태 정보를 전달

Header header

# 안전 상태
string status              # "SAFE", "WARNING", "DANGER", "EMERGENCY"
float32 min_distance       # 최소 장애물 거리 (m)
int32 obstacle_count      # 감지된 장애물 개수
bool emergency_stop        # 비상 정지 상태

# 센서 정보
bool front_sensor_active   # 전방 센서 활성 상태
bool rear_sensor_active    # 후방 센서 활성 상태
float32 front_min_distance # 전방 최소 거리 (m)
float32 rear_min_distance # 후방 최소 거리 (m)

# 제어 정보
float32 current_speed      # 현재 속도 (m/s)
string current_direction   # 현재 방향 ("forward", "backward", "stop")
bool safety_override       # 안전 제어 우선순위
