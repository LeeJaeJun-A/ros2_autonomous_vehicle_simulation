"""
Motion Planner Node for Parking
Camera + LiDAR 센서를 통합한 주차 제어 노드

센서 통합:
- LiDAR: 주변 장애물 감지 및 주차 공간 위치 파악
- Camera: 차선 검출을 통한 정확한 주차 위치 정렬

주차 단계:
1. initial_forward: 초기 직진하며 주차 공간 탐색
2. turning_left: 좌회전하여 주차 준비 위치로 이동
3. reversing: 후진하며 LiDAR 각도 기반 조향
4. fine_tuning: Camera 차선 정보로 미세 조정
5. parked: 주차 완료 및 대기
6. exit_forward: 주차 공간 탈출 (전진)
7. exit_turn: 주차 공간 탈출 (회전)
8. exit_straight: 주차 공간 탈출 (직진)
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSReliabilityPolicy

from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32, Bool
from interfaces_pkg.msg import MotionCommand, LaneInfo
import numpy as np

#--------------- Node Specific Parameters ---------------
SUB_LIDAR_TOPIC_NAME = "scan"
SUB_OBSTACLE_INFO_TOPIC_NAME = "lidar_obstacle_info"
SUB_OBSTACLE_START_ANGLE_TOPIC_NAME = "obstacle_start_angle"
SUB_OBSTACLE_END_ANGLE_TOPIC_NAME = "obstacle_end_angle"
SUB_LANE_INFO_TOPIC_NAME = "parking_lane_info"
SUB_LATERAL_OFFSET_TOPIC_NAME = "parking_lateral_offset"
SUB_REAR_WALL_DISTANCE_TOPIC_NAME = "rear_wall_distance"
SUB_LANE_END_DETECTED_TOPIC_NAME = "parking_lane_end_detected"

PUB_TOPIC_NAME = "topic_control_signal"

TIMER_PERIOD = 0.1  # 타이머 주기 (초)
STEERING_FACTOR = 9  # 조향 스케일 팩터
MAX_ANGLE_BUFFER_SIZE = 20  # 각도 데이터 버퍼 최대 크기 (노이즈 방지)

#--------------- Motion Parameters ---------------
FORWARD_SPEED_INIT = 100        # 초기 직진 속도
REVERSE_SPEED = -80             # 후진 속도
STOP_SPEED = 0                  # 정지 속도

#--------------- Turning Parameters ---------------
LEFT_TURN_DURATION = 4.4        # 좌회전 지속 시간 (초)
TURN_STEERING = -9              # 좌회전 조향 값
TURN_SPEED = 200                # 회전 속도
RIGHT_TURN_STEERING = 9         # 우회전 조향 값
RIGHT_TURN_DURATION = 5.0       # 우회전 지속 시간 (초)

#--------------- Rear LiDAR Processing Parameters ---------------
REAR_ANGLE_CENTER = 180.0       # 후방 기준 각도 (degree)
REAR_LIDAR_ANGLE_MIN = 100.0    # 후방 LiDAR 감지 최소 각도
REAR_LIDAR_ANGLE_MAX = 260.0    # 후방 LiDAR 감지 최대 각도

# 조향 결정 임계값
STEERING_ANGLE_THRESHOLD_HIGH = 183.0  # 이 값보다 크면 우회전
STEERING_ANGLE_THRESHOLD_LOW = 177.0   # 이 값보다 작으면 좌회전

#--------------- Camera-based Fine Tuning Parameters ---------------
LATERAL_OFFSET_THRESHOLD = 20.0  # 좌우 오프셋 임계값 (픽셀)
CAMERA_STEERING_GAIN = 0.02      # Camera 오프셋 -> 조향 변환 게인

#--------------- Safety Parameters ---------------
REAR_WALL_SAFE_DISTANCE = 0.6    # 후방 벽 안전 거리 (m) - 이 거리보다 가까우면 멈춤

#--------------- Parking Sequence Timing ---------------
INITIAL_FORWARD_MIN_DURATION = 2.0   # 초기 직진 최소 시간 (장애물 감지 대기)
REVERSING_DURATION = 12.0            # 후진 지속 시간 (초)
FINE_TUNING_DURATION = 3.0           # 미세 조정 시간 (초)
PARKED_WAIT_DURATION = 3.0           # 주차 완료 후 대기 시간 (초)

#--------------- Exit Sequence Parameters ---------------
EXIT_FORWARD_SPEED = 200             # 탈출 전진 속도
EXIT_FORWARD_DURATION = 1.0          # 탈출 전진 시간 (초)
EXIT_STRAIGHT_DURATION = 5.0         # 탈출 후 직진 시간 (초)
#----------------------------------------------


class ParkingMotionPlanner(Node):
    def __init__(self):
        super().__init__('motion_planner_node_parking')

        # 파라미터 선언
        self.timer_period = self.declare_parameter('timer', TIMER_PERIOD).value

        # QoS 설정
        self.qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=1
        )

        # ===== 센서 데이터 변수 =====
        # LiDAR 관련
        self.lidar_data = None
        self.right_obstacle_detected = False
        self.received_start_angles = []
        self.received_end_angles = []
        self.rear_wall_distance = float('inf')  # 후방 벽까지의 거리 (m)

        # Camera 관련
        self.lane_info = None
        self.lateral_offset = 0.0  # 좌우 오프셋 (픽셀)
        self.lane_end_detected = False  # 주차선 끝 감지 여부

        # ===== 제어 명령 변수 =====
        self.steering_command = 0.0
        self.left_speed_command = 0.0
        self.right_speed_command = 0.0

        # ===== 주차 상태 머신 =====
        self.parking_state = 'initial_forward'

        # ===== 타이밍 변수 =====
        self.initial_forward_start_time = None
        self.left_turn_start_time = None
        self.reversing_start_time = None
        self.fine_tuning_start_time = None
        self.parked_start_time = None
        self.exit_forward_start_time = None
        self.exit_turn_start_time = None
        self.exit_straight_start_time = None

        # Camera 타임아웃 추적
        self.last_camera_update_time = None

        # ===== 구독자 설정 =====
        # LiDAR 구독자
        self.lidar_sub = self.create_subscription(
            LaserScan,
            SUB_LIDAR_TOPIC_NAME,
            self.lidar_callback,
            self.qos_profile
        )
        self.obstacle_info_sub = self.create_subscription(
            Bool,
            SUB_OBSTACLE_INFO_TOPIC_NAME,
            self.obstacle_info_callback,
            self.qos_profile
        )
        self.start_angle_sub = self.create_subscription(
            Float32,
            SUB_OBSTACLE_START_ANGLE_TOPIC_NAME,
            self.start_angle_callback,
            self.qos_profile
        )
        self.end_angle_sub = self.create_subscription(
            Float32,
            SUB_OBSTACLE_END_ANGLE_TOPIC_NAME,
            self.end_angle_callback,
            self.qos_profile
        )

        # Camera 구독자
        self.lane_info_sub = self.create_subscription(
            LaneInfo,
            SUB_LANE_INFO_TOPIC_NAME,
            self.lane_info_callback,
            self.qos_profile
        )
        self.lateral_offset_sub = self.create_subscription(
            Float32,
            SUB_LATERAL_OFFSET_TOPIC_NAME,
            self.lateral_offset_callback,
            self.qos_profile
        )

        # 안전 거리 구독자
        self.rear_distance_sub = self.create_subscription(
            Float32,
            SUB_REAR_WALL_DISTANCE_TOPIC_NAME,
            self.rear_distance_callback,
            self.qos_profile
        )

        # 주차선 끝 감지 구독자
        self.lane_end_sub = self.create_subscription(
            Bool,
            SUB_LANE_END_DETECTED_TOPIC_NAME,
            self.lane_end_callback,
            self.qos_profile
        )

        # ===== 발행자 설정 =====
        self.publisher = self.create_publisher(
            MotionCommand,
            PUB_TOPIC_NAME,
            self.qos_profile
        )

        # ===== 타이머 설정 =====
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.initial_forward_start_time = self.get_clock().now().nanoseconds / 1e9

        self.get_logger().info('Parking Motion Planner Node initialized')
        self.get_logger().info(f'Initial state: {self.parking_state}')

    # ==================== 콜백 함수들 ====================

    def lidar_callback(self, msg: LaserScan):
        """LiDAR 원시 데이터 수신"""
        self.lidar_data = msg

    def obstacle_info_callback(self, msg: Bool):
        """오른쪽 장애물 감지 정보 수신"""
        self.right_obstacle_detected = msg.data
        if msg.data:
            self.get_logger().info(f"[STATE: {self.parking_state}] Right obstacle detected!")

    def start_angle_callback(self, msg: Float32):
        """후방 장애물 시작 각도 수신 (버퍼 크기 제한으로 노이즈 방지)"""
        angle = msg.data
        # reversing 상태일 때만 후방 각도 수집
        if self.parking_state in ['reversing', 'fine_tuning']:
            if REAR_LIDAR_ANGLE_MIN <= angle <= REAR_LIDAR_ANGLE_MAX:
                self.received_start_angles.append(angle)
                # 버퍼 크기 제한 (최근 데이터만 유지)
                if len(self.received_start_angles) > MAX_ANGLE_BUFFER_SIZE:
                    self.received_start_angles.pop(0)
                self.get_logger().debug(f"Received start angle: {angle:.2f}°")

    def end_angle_callback(self, msg: Float32):
        """후방 장애물 끝 각도 수신 (버퍼 크기 제한으로 노이즈 방지)"""
        angle = msg.data
        # reversing 상태일 때만 후방 각도 수집
        if self.parking_state in ['reversing', 'fine_tuning']:
            if REAR_LIDAR_ANGLE_MIN <= angle <= REAR_LIDAR_ANGLE_MAX:
                self.received_end_angles.append(angle)
                # 버퍼 크기 제한 (최근 데이터만 유지)
                if len(self.received_end_angles) > MAX_ANGLE_BUFFER_SIZE:
                    self.received_end_angles.pop(0)
                self.get_logger().debug(f"Received end angle: {angle:.2f}°")

    def lane_info_callback(self, msg: LaneInfo):
        """주차 차선 정보 수신"""
        self.lane_info = msg

    def lateral_offset_callback(self, msg: Float32):
        """좌우 오프셋 수신 (Camera 기반)"""
        self.lateral_offset = msg.data
        # Camera 데이터 수신 시간 기록 (타임아웃 감지용)
        self.last_camera_update_time = self.get_clock().now().nanoseconds / 1e9

    def rear_distance_callback(self, msg: Float32):
        """후방 벽까지의 거리 수신"""
        self.rear_wall_distance = msg.data

    def lane_end_callback(self, msg: Bool):
        """주차선 끝 감지 수신"""
        self.lane_end_detected = msg.data
        if msg.data:
            self.get_logger().info(f"[STATE: {self.parking_state}] 🛑 Parking lane END detected by camera!")

    # ==================== 메인 타이머 콜백 ====================

    def timer_callback(self):
        """메인 제어 루프"""
        now = self.get_clock().now().nanoseconds / 1e9

        # 상태별 제어 로직 실행
        if self.parking_state == 'initial_forward':
            self.state_initial_forward(now)
        elif self.parking_state == 'turning_left':
            self.state_turning_left(now)
        elif self.parking_state == 'reversing':
            self.state_reversing(now)
        elif self.parking_state == 'fine_tuning':
            self.state_fine_tuning(now)
        elif self.parking_state == 'parked':
            self.state_parked(now)
        elif self.parking_state == 'exit_forward':
            self.state_exit_forward(now)
        elif self.parking_state == 'exit_turn':
            self.state_exit_turn(now)
        elif self.parking_state == 'exit_straight':
            self.state_exit_straight(now)

        # 모션 명령 메시지 발행
        self.publish_motion_command()

    # ==================== 상태별 제어 함수들 ====================

    def state_initial_forward(self, now):
        """
        상태 1: 초기 직진
        - 오른쪽 장애물(주차된 차량) 감지 대기
        - 일정 시간 후 감지 시작
        """
        if self.initial_forward_start_time is None:
            self.initial_forward_start_time = now

        self.steering_command = 0.0
        self.left_speed_command = FORWARD_SPEED_INIT
        self.right_speed_command = FORWARD_SPEED_INIT

        # 초기 대기 시간 이후 장애물 감지 반응
        elapsed = now - self.initial_forward_start_time

        # 주기적 상태 로그 (1초마다)
        if int(elapsed) % 2 == 0 and elapsed - int(elapsed) < 0.1:
            self.get_logger().info(
                f"[initial_forward] Elapsed: {elapsed:.1f}s, "
                f"Right obstacle: {self.right_obstacle_detected}"
            )

        if elapsed >= INITIAL_FORWARD_MIN_DURATION:
            if self.right_obstacle_detected:
                self.get_logger().info("Right obstacle detected! Starting left turn.")
                self.parking_state = 'turning_left'
                self.left_turn_start_time = now

    def state_turning_left(self, now):
        """
        상태 2: 좌회전
        - 주차 공간 앞에 위치하도록 좌회전
        """
        elapsed = now - self.left_turn_start_time

        if elapsed >= LEFT_TURN_DURATION:
            self.get_logger().info("Left turn completed. Starting reverse.")
            self.parking_state = 'reversing'
            self.reversing_start_time = now
            self.steering_command = 0.0
            # 각도 수집 초기화
            self.received_start_angles = []
            self.received_end_angles = []
            self.get_logger().info("Collecting rear obstacle angle data...")
        else:
            # 좌회전 실행
            self.steering_command = TURN_STEERING
            self.left_speed_command = TURN_SPEED / 3
            self.right_speed_command = TURN_SPEED * 0.85

    def state_reversing(self, now):
        """
        상태 3: 후진 및 조향
        - LiDAR 후방 장애물 각도를 기반으로 조향
        - 주차 공간 중앙으로 정렬
        - 후방 벽 거리 체크로 안전 정지
        """
        elapsed = now - self.reversing_start_time

        # 1순위: Camera 주차선 끝 감지
        if self.lane_end_detected:
            self.get_logger().warn(
                f"🅿️  Parking lane END detected! Switching to fine tuning for final adjustment."
            )
            self.parking_state = 'fine_tuning'
            self.fine_tuning_start_time = now
            return

        # 2순위: 안전 거리 체크 - 후방 벽이 너무 가까우면 즉시 미세 조정으로 전환
        if self.rear_wall_distance < REAR_WALL_SAFE_DISTANCE:
            self.get_logger().warn(
                f"⚠️  Rear wall too close! Distance: {self.rear_wall_distance:.2f}m "
                f"< Safe: {REAR_WALL_SAFE_DISTANCE}m. Switching to fine tuning."
            )
            self.parking_state = 'fine_tuning'
            self.fine_tuning_start_time = now
            return

        self.left_speed_command = REVERSE_SPEED
        self.right_speed_command = REVERSE_SPEED

        # LiDAR 각도 데이터로 조향 결정 (median 사용으로 노이즈 제거)
        if len(self.received_start_angles) >= 2 and len(self.received_end_angles) >= 2:
            # 극값(max/min) 대신 중간값(median) 사용 → 아웃라이어 영향 감소
            median_end_angle = np.median(self.received_end_angles)
            median_start_angle = np.median(self.received_start_angles)
            steering_angle_deg = (median_end_angle + median_start_angle) / 2.0

            if steering_angle_deg > STEERING_ANGLE_THRESHOLD_HIGH:
                self.steering_command = 1.0  # 우회전
                self.get_logger().info(
                    f"Reversing: Right (median={steering_angle_deg:.2f}°, "
                    f"start={median_start_angle:.1f}°, end={median_end_angle:.1f}°)"
                )
            elif steering_angle_deg < STEERING_ANGLE_THRESHOLD_LOW:
                self.steering_command = -1.0  # 좌회전
                self.get_logger().info(
                    f"Reversing: Left (median={steering_angle_deg:.2f}°, "
                    f"start={median_start_angle:.1f}°, end={median_end_angle:.1f}°)"
                )
            else:
                self.steering_command = 0.0  # 직진
                self.get_logger().info(
                    f"Reversing: Straight (median={steering_angle_deg:.2f}°, "
                    f"start={median_start_angle:.1f}°, end={median_end_angle:.1f}°)"
                )
        else:
            # 초기 후진은 직진
            self.steering_command = 0.0
            self.get_logger().debug("Reversing straight (waiting for angle data)")

        # 후진 완료 후 미세 조정 단계로 전환
        if elapsed >= REVERSING_DURATION:
            self.get_logger().info("Reversing completed. Starting fine tuning with camera.")
            self.get_logger().info(
                f"Collected {len(self.received_start_angles)} start angles, "
                f"{len(self.received_end_angles)} end angles"
            )
            self.parking_state = 'fine_tuning'
            self.fine_tuning_start_time = now

    def state_fine_tuning(self, now):
        """
        상태 4: 미세 조정 (Camera 기반, 실패 시 LiDAR 폴백)
        - 차선 lateral offset을 사용하여 정확한 위치 조정
        - Camera 타임아웃 시 LiDAR 데이터로 폴백
        - 천천히 후진하며 조향
        - 후방 벽 안전 거리 체크
        """
        elapsed = now - self.fine_tuning_start_time

        # 1순위: Camera 주차선 끝 감지 - 정확한 위치에 도달
        if self.lane_end_detected:
            self.get_logger().warn(
                f"🅿️  Parking lane END reached! Perfect parking position. Parking completed!"
            )
            self.parking_state = 'parked'
            self.parked_start_time = now
            self.steering_command = 0.0
            self.left_speed_command = STOP_SPEED
            self.right_speed_command = STOP_SPEED
            return

        # 2순위: 안전 거리 체크 - 최소 안전 거리에 도달하면 즉시 주차 완료
        if self.rear_wall_distance < REAR_WALL_SAFE_DISTANCE:
            self.get_logger().warn(
                f"🅿️  Reached safe distance! Distance: {self.rear_wall_distance:.2f}m. "
                f"Parking completed!"
            )
            self.parking_state = 'parked'
            self.parked_start_time = now
            self.steering_command = 0.0
            self.left_speed_command = STOP_SPEED
            self.right_speed_command = STOP_SPEED
            return

        # Camera 데이터 유효성 확인 (타임아웃 체크)
        camera_available = True
        if self.last_camera_update_time is None:
            camera_available = False
            self.get_logger().warn("No camera data received yet")
        elif (now - self.last_camera_update_time) > 1.0:
            camera_available = False
            self.get_logger().warn("Camera timeout (>1s), using LiDAR fallback")

        # 느린 속도로 후진
        self.left_speed_command = REVERSE_SPEED / 2
        self.right_speed_command = REVERSE_SPEED / 2

        # === Camera 기반 조향 (우선순위 1) ===
        if camera_available:
            if abs(self.lateral_offset) > LATERAL_OFFSET_THRESHOLD:
                # 오프셋이 큰 경우 조향
                steering_adjustment = self.lateral_offset * CAMERA_STEERING_GAIN
                # 조향 제한 (-1.0 ~ 1.0)
                self.steering_command = np.clip(steering_adjustment, -1.0, 1.0)
                self.get_logger().info(
                    f"Fine tuning (Camera): offset={self.lateral_offset:.1f}px, "
                    f"steering={self.steering_command:.2f}"
                )
            else:
                # 오프셋이 작으면 직진 (중앙 정렬 완료)
                self.steering_command = 0.0
                self.get_logger().info("Fine tuning (Camera): centered")

        # === LiDAR 폴백 조향 (Camera 실패 시) ===
        else:
            if len(self.received_start_angles) >= 2 and len(self.received_end_angles) >= 2:
                median_end_angle = np.median(self.received_end_angles)
                median_start_angle = np.median(self.received_start_angles)
                steering_angle_deg = (median_end_angle + median_start_angle) / 2.0

                # fine_tuning에서는 약한 조향 (0.5 대신 1.0)
                if steering_angle_deg > STEERING_ANGLE_THRESHOLD_HIGH:
                    self.steering_command = 0.5  # 약한 우회전
                elif steering_angle_deg < STEERING_ANGLE_THRESHOLD_LOW:
                    self.steering_command = -0.5  # 약한 좌회전
                else:
                    self.steering_command = 0.0

                self.get_logger().info(
                    f"Fine tuning (LiDAR fallback): median={steering_angle_deg:.2f}°, "
                    f"steering={self.steering_command:.2f}"
                )
            else:
                # 각도 데이터도 없으면 직진
                self.steering_command = 0.0
                self.get_logger().warn("Fine tuning: No sensor data, going straight")

        # 미세 조정 완료 후 주차 완료 상태로 전환
        if elapsed >= FINE_TUNING_DURATION:
            self.get_logger().info("Fine tuning completed. Parking finished!")
            self.parking_state = 'parked'
            self.parked_start_time = now
            self.steering_command = 0.0
            self.left_speed_command = STOP_SPEED
            self.right_speed_command = STOP_SPEED

    def state_parked(self, now):
        """
        상태 5: 주차 완료
        - 정지 상태 유지
        - 일정 시간 후 탈출 시퀀스 시작
        """
        elapsed = now - self.parked_start_time

        self.steering_command = 0.0
        self.left_speed_command = STOP_SPEED
        self.right_speed_command = STOP_SPEED

        if elapsed >= PARKED_WAIT_DURATION:
            self.get_logger().info("Starting exit sequence - moving forward")
            self.parking_state = 'exit_forward'
            self.exit_forward_start_time = now

    def state_exit_forward(self, now):
        """
        상태 6: 탈출 전진
        - 주차 공간에서 전진
        """
        elapsed = now - self.exit_forward_start_time

        self.steering_command = 0.0
        self.left_speed_command = EXIT_FORWARD_SPEED
        self.right_speed_command = EXIT_FORWARD_SPEED

        if elapsed >= EXIT_FORWARD_DURATION:
            self.get_logger().info("Exit forward completed. Starting right turn.")
            self.parking_state = 'exit_turn'
            self.exit_turn_start_time = now

    def state_exit_turn(self, now):
        """
        상태 7: 탈출 우회전
        - 주차 공간을 벗어나기 위해 우회전
        """
        elapsed = now - self.exit_turn_start_time

        self.steering_command = RIGHT_TURN_STEERING
        self.left_speed_command = TURN_SPEED
        self.right_speed_command = TURN_SPEED / 3

        if elapsed >= RIGHT_TURN_DURATION:
            self.get_logger().info("Exit turn complete. Moving straight. Parking mission finished!")
            self.parking_state = 'exit_straight'
            self.exit_straight_start_time = now

    def state_exit_straight(self, now):
        """
        상태 8: 탈출 직진
        - 주차 공간을 완전히 벗어남
        """
        self.steering_command = 0.0
        self.left_speed_command = EXIT_FORWARD_SPEED
        self.right_speed_command = EXIT_FORWARD_SPEED

        # 계속 직진 (종료 조건은 사용자가 직접 중단)

    # ==================== 발행 함수 ====================

    def publish_motion_command(self):
        """모션 명령 메시지 생성 및 발행"""
        motion_command_msg = MotionCommand()
        motion_command_msg.steering = int(self.steering_command * STEERING_FACTOR)
        motion_command_msg.left_speed = int(self.left_speed_command)
        motion_command_msg.right_speed = int(self.right_speed_command)
        self.publisher.publish(motion_command_msg)

        self.get_logger().debug(
            f"State: {self.parking_state} | "
            f"Steering: {motion_command_msg.steering} | "
            f"Speed: L={motion_command_msg.left_speed}, R={motion_command_msg.right_speed}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = ParkingMotionPlanner()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n\nShutdown\n\n")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

