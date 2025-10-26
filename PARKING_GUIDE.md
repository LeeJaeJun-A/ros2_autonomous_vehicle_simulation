# 🚗 Camera + LiDAR 통합 주차 알고리즘 가이드

## 📋 개요

이 주차 알고리즘은 **Camera 센서와 LiDAR 센서를 통합**하여 정확한 후면 주차를 수행합니다.

### 🎯 주요 기능

1. **LiDAR 기반 주차 공간 감지**
   - 오른쪽 영역(230~300도) 장애물 감지로 주차 공간 탐지
   - 후방 장애물 각도 측정으로 주차 공간 중앙 위치 파악

2. **Camera 기반 정밀 정렬**
   - YOLOv8로 주차 공간 차선 검출
   - Bird's eye view 변환으로 정확한 위치 계산
   - 좌우 오프셋(lateral offset) 계산으로 미세 조정

3. **8단계 주차 시퀀스**
   - 초기 직진 → 좌회전 → 후진(LiDAR 조향) → 미세조정(Camera 조향) → 주차완료 → 탈출

---

## 📁 추가된 파일 목록

### 1. LiDAR Perception Package
```
src/lidar_perception_pkg/lidar_perception_pkg/
├── lib/
│   └── lidar_perception_func_lib.py          # 새로 추가 (StabilityDetector 등)
└── lidar_obstacle_detector_node_parking.py   # 새로 추가 (주차용 장애물 감지)
```

### 2. Camera Perception Package
```
src/camera_perception_pkg/camera_perception_pkg/
└── parking_lane_detector_node.py             # 새로 추가 (주차 차선 검출)
```

### 3. Decision Making Package
```
src/decision_making_pkg/decision_making_pkg/
└── motion_planner_node_parking.py            # 새로 추가 (통합 주차 제어)
```

### 4. 수정된 파일
- `src/lidar_perception_pkg/setup.py` - 주차 노드 등록
- `src/camera_perception_pkg/setup.py` - 주차 노드 등록
- `src/decision_making_pkg/setup.py` - 주차 노드 등록
- `src/simulation_pkg/launch/parking_sim.launch.py` - 주차 노드들 추가

---

## 🏗️ 시스템 아키텍처

```
┌─────────────────────┐
│   LiDAR Sensor      │
│     (/scan)         │
└──────────┬──────────┘
           │
           ▼
┌─────────────────────┐
│ lidar_processor_    │
│      node           │
└──────────┬──────────┘
           │ /lidar_processed
           ▼
┌─────────────────────────────────────────┐
│ lidar_obstacle_detector_node_parking    │
│                                         │
│ • 오른쪽 장애물 감지 (Bool)              │
│ • 후방 장애물 각도 (Float32 x2)          │
└──────────┬──────────────────────────────┘
           │
           │ /lidar_obstacle_info
           │ /obstacle_start_angle
           │ /obstacle_end_angle
           │
           ▼
┌─────────────────────┐        ┌──────────────────────┐
│  Camera Sensor      │        │   yolov8_node        │
│  (이미지 토픽)        │───────▶│  (객체 검출)          │
└─────────────────────┘        └──────────┬───────────┘
                                          │ /detections
                                          ▼
                               ┌─────────────────────────┐
                               │ parking_lane_detector_  │
                               │        node             │
                               │                         │
                               │ • 차선 검출              │
                               │ • lateral offset 계산   │
                               └──────────┬──────────────┘
                                          │
                                          │ /parking_lane_info
                                          │ /parking_lateral_offset
                                          ▼
                               ┌─────────────────────────────────┐
                               │  motion_planner_node_parking    │
                               │                                 │
                               │  LiDAR + Camera 센서 통합       │
                               │  8단계 주차 제어 로직            │
                               └──────────┬──────────────────────┘
                                          │ /topic_control_signal
                                          ▼
                               ┌─────────────────────────┐
                               │ simulation_sender_node  │
                               └──────────┬──────────────┘
                                          │ /cmd_vel
                                          ▼
                               ┌─────────────────────────┐
                               │   Vehicle Control       │
                               └─────────────────────────┘
```

---

## 🚀 실행 방법

### 1. 빌드
```bash
cd ~/ros2_autonomous_vehicle_simulation
colcon build --packages-select lidar_perception_pkg camera_perception_pkg decision_making_pkg simulation_pkg
source install/setup.bash
```

### 2. 주차 시뮬레이션 실행
```bash
# Gazebo와 주차 환경 자동으로 종료 후 실행
sudo killall -9 gazebo gzserver gzclient
ros2 launch simulation_pkg parking_sim.launch.py
```

### 3. 실행되는 노드 목록
launch 파일이 자동으로 다음 노드들을 실행합니다:
- `load_ego_car_parking_node` - 주차용 ego vehicle 로드
- `load_obstable_car_node` - 주차된 차량들 로드
- `load_traffic_light_node` - 신호등 로드
- `sim_simulation_sender_node` - 시뮬레이션 제어 신호 전송
- **`lidar_processor_node`** - LiDAR 데이터 전처리
- **`lidar_obstacle_detector_node_parking`** - 주차용 장애물 감지 (새로 추가)
- **`yolov8_node`** - YOLOv8 객체 검출
- **`parking_lane_detector_node`** - 주차 차선 검출 (새로 추가)
- **`motion_planner_node_parking`** - 통합 주차 제어 (새로 추가)

---

## 🎮 주차 알고리즘 동작 과정

### 상태 1: `initial_forward` (초기 직진)
- **동작**: 차량이 직진하며 주차 공간 탐색
- **센서**: LiDAR 오른쪽 영역 (230~300도) 감지
- **전환 조건**: 오른쪽 장애물 3회 연속 감지 시
- **다음 상태**: `turning_left`

### 상태 2: `turning_left` (좌회전)
- **동작**: 주차 준비 위치로 좌회전
- **기간**: 4.4초
- **조향**: -9 (좌회전)
- **다음 상태**: `reversing`

### 상태 3: `reversing` (후진 + LiDAR 조향)
- **동작**: 후진하며 주차 공간 진입
- **센서**: LiDAR 후방 장애물 시작/끝 각도
- **조향 로직**:
  - 평균 각도 > 183° → 우회전 (오른쪽 공간이 넓음)
  - 평균 각도 < 177° → 좌회전 (왼쪽 공간이 넓음)
  - 177° ~ 183° → 직진 (중앙 정렬)
- **기간**: 12초
- **다음 상태**: `fine_tuning`

### 상태 4: `fine_tuning` (미세 조정 + Camera 조향)
- **동작**: Camera 차선 정보로 정밀 정렬
- **센서**: Camera lateral offset
- **조향 로직**:
  - lateral offset > 20px → 조향 조정
  - lateral offset ≤ 20px → 직진
- **속도**: 후진 속도의 50%
- **기간**: 3초
- **다음 상태**: `parked`

### 상태 5: `parked` (주차 완료)
- **동작**: 정지 대기
- **기간**: 3초
- **다음 상태**: `exit_forward`

### 상태 6: `exit_forward` (탈출 전진)
- **동작**: 주차 공간에서 전진
- **기간**: 1초
- **다음 상태**: `exit_turn`

### 상태 7: `exit_turn` (탈출 우회전)
- **동작**: 우회전하며 주차 공간 탈출
- **기간**: 5초
- **다음 상태**: `exit_straight`

### 상태 8: `exit_straight` (탈출 직진)
- **동작**: 계속 직진 (사용자가 중단할 때까지)

---

## ⚙️ 파라미터 튜닝

주차 성능을 최적화하려면 다음 파라미터들을 조정할 수 있습니다:

### motion_planner_node_parking.py
```python
# 속도 파라미터
FORWARD_SPEED_INIT = 100        # 초기 직진 속도
REVERSE_SPEED = -80             # 후진 속도

# 타이밍 파라미터
LEFT_TURN_DURATION = 4.4        # 좌회전 시간
REVERSING_DURATION = 12.0       # 후진 시간
FINE_TUNING_DURATION = 3.0      # 미세 조정 시간

# 조향 임계값
STEERING_ANGLE_THRESHOLD_HIGH = 183.0
STEERING_ANGLE_THRESHOLD_LOW = 177.0

# Camera 게인
LATERAL_OFFSET_THRESHOLD = 20.0
CAMERA_STEERING_GAIN = 0.02
```

### lidar_obstacle_detector_node_parking.py
```python
# 오른쪽 장애물 감지 영역
RIGHT_DETECTION_START_ANGLE = 230.0
RIGHT_DETECTION_END_ANGLE = 300.0
RIGHT_DETECTION_RANGE_MIN = 0.5
RIGHT_DETECTION_RANGE_MAX = 2.5

# 안정성 파라미터
CONSECUTIVE_DETECTION_COUNT = 3  # 연속 감지 횟수
```

### parking_lane_detector_node.py
```python
# 차선 검출 파라미터
TARGET_POINT_Y_RANGE = range(5, 155, 30)
LANE_WIDTH = 300
THETA_LIMIT = 70

# Bird's eye view 변환 좌표
SRC_POINTS = [[238, 316], [402, 313], [501, 476], [155, 476]]
```

---

## 🔍 디버깅 및 모니터링

### ROS2 토픽 확인
```bash
# 모든 토픽 확인
ros2 topic list

# 주차 관련 주요 토픽
ros2 topic echo /lidar_obstacle_info              # 오른쪽 장애물 감지 (Bool)
ros2 topic echo /obstacle_start_angle             # 후방 장애물 시작 각도
ros2 topic echo /obstacle_end_angle               # 후방 장애물 끝 각도
ros2 topic echo /parking_lane_info                # 주차 차선 정보
ros2 topic echo /parking_lateral_offset           # 좌우 오프셋
ros2 topic echo /topic_control_signal             # 제어 명령
```

### 노드 로그 확인
```bash
# 특정 노드의 로그만 보기
ros2 run lidar_perception_pkg lidar_obstacle_detector_node_parking --ros-args --log-level debug
ros2 run camera_perception_pkg parking_lane_detector_node --ros-args --log-level debug
ros2 run decision_making_pkg motion_planner_node_parking --ros-args --log-level debug
```

### 이미지 시각화
주차 중 Camera 처리 과정을 보려면:
- `parking_lane_detector_node.py`에서 `SHOW_IMAGE = True` 설정
- OpenCV 창에서 다음 이미지 확인:
  - `Parking - Lane Edge`: 차선 에지
  - `Parking - Bird View`: Bird's eye view 변환
  - `Parking - ROI`: ROI 영역

---

## 🐛 문제 해결

### 문제 1: 차량이 주차 공간을 못 찾음
**원인**: LiDAR 오른쪽 감지 범위 문제
**해결**: `lidar_obstacle_detector_node_parking.py`의 `RIGHT_DETECTION_START_ANGLE`, `RIGHT_DETECTION_END_ANGLE` 조정

### 문제 2: 후진 시 비뚤어짐
**원인**: LiDAR 각도 임계값 부적절
**해결**: `motion_planner_node_parking.py`의 `STEERING_ANGLE_THRESHOLD_HIGH/LOW` 조정

### 문제 3: 미세 조정 시 계속 흔들림
**원인**: Camera 게인이 너무 높음
**해결**: `CAMERA_STEERING_GAIN` 값을 낮춤 (예: 0.02 → 0.01)

### 문제 4: Camera 차선을 못 찾음
**원인**: Bird's eye view 변환 좌표 부적절
**해결**: `parking_lane_detector_node.py`의 `SRC_POINTS` 좌표를 카메라 설정에 맞게 조정

---

## 📊 센서 통합 전략

### LiDAR의 역할
✅ **장점**: 거리 측정 정확, 주변 환경 360도 감지
- 주차 공간 탐지 (오른쪽 장애물)
- 후방 장애물과의 거리 및 각도 측정
- 대략적인 조향 결정

### Camera의 역할
✅ **장점**: 차선 인식, 세밀한 위치 정렬
- 주차 공간 차선 검출
- 좌우 오프셋 계산
- 미세 조정 단계에서 정밀 제어

### 통합 효과
🎯 **LiDAR**: 빠른 주차 공간 진입 (거칠지만 빠름)
🎯 **Camera**: 정확한 최종 정렬 (느리지만 정확함)
🎯 **결과**: 빠르고 정확한 주차!

---

## 🎓 추가 개선 아이디어

1. **초음파 센서 추가**: 좁은 공간 감지
2. **딥러닝 end-to-end 학습**: 전체 주차 과정 학습
3. **Adaptive 파라미터**: 주차 공간 크기에 따라 파라미터 자동 조정
4. **평행 주차 지원**: 현재는 후면 주차만 지원
5. **장애물 회피**: 동적 장애물 감지 및 회피

---

## 📝 참고 자료

- `src_answer/` 폴더: 원본 참고 코드
- ROS2 Humble 문서: https://docs.ros.org/en/humble/
- YOLOv8 문서: https://docs.ultralytics.com/

---

## 👥 기여자

이 주차 알고리즘은 `src_answer` 폴더의 코드를 참고하여 Camera 센서 통합 기능을 추가하여 개선되었습니다.

---

**Happy Parking! 🚗💨**

