"""
LiDAR Obstacle Detector Node for Parking
주차를 위한 LiDAR 기반 장애물 감지 노드

기능:
1. 오른쪽 영역(230~300도)의 장애물 감지 -> Bool 메시지로 발행
2. 후방 영역의 장애물 시작/끝 각도 감지 -> Float32 메시지로 발행
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float32
import numpy as np

from rclpy.qos import QoSProfile
from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSReliabilityPolicy

from .lib import lidar_perception_func_lib as LPFL

#---------------Variable Setting---------------
SUB_TOPIC_NAME = 'lidar_processed'  # 구독할 LiDAR 토픽 이름

PUB_OBSTACLE_INFO_TOPIC_NAME = 'lidar_obstacle_info'  # 장애물 유무 발행 토픽
PUB_START_ANGLE_TOPIC_NAME = 'obstacle_start_angle'  # 장애물 시작 각도 토픽
PUB_END_ANGLE_TOPIC_NAME = 'obstacle_end_angle'  # 장애물 끝 각도 토픽

# 오른쪽 장애물 감지 파라미터
RIGHT_DETECTION_START_ANGLE = 230.0  # 오른쪽 감지 시작 각도
RIGHT_DETECTION_END_ANGLE = 300.0    # 오른쪽 감지 끝 각도
RIGHT_DETECTION_RANGE_MIN = 0.5      # 오른쪽 감지 최소 거리 (m)
RIGHT_DETECTION_RANGE_MAX = 2.5      # 오른쪽 감지 최대 거리 (m)
CONSECUTIVE_DETECTION_COUNT = 3      # 안정적 감지를 위한 연속 감지 횟수

# 후방 장애물 각도 감지 파라미터
REAR_START_ANGLE = 0.0        # 후방 감지 시작 각도 (파라미터로 설정 가능)
REAR_END_ANGLE = 359.0        # 후방 감지 끝 각도 (파라미터로 설정 가능)
REAR_RANGE_MIN = 0.5          # 후방 감지 최소 거리 (m)
REAR_RANGE_MAX = 3.0          # 후방 감지 최대 거리 (m)
ANGLE_MERGE_THRESHOLD = 10.5  # 세그먼트 병합 임계값 (도)
#------------------------------------------------


class ParkingObstacleDetector(Node):
    def __init__(self):
        super().__init__('lidar_obstacle_detector_node_parking')

        # 파라미터 선언
        self.declare_parameter('start_angle', REAR_START_ANGLE)
        self.declare_parameter('end_angle', REAR_END_ANGLE)
        self.declare_parameter('range_min', REAR_RANGE_MIN)
        self.declare_parameter('range_max', REAR_RANGE_MAX)
        self.declare_parameter('angle_threshold', ANGLE_MERGE_THRESHOLD)

        # QoS 설정
        self.qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=1
        )

        # 구독자 설정
        self.subscriber = self.create_subscription(
            LaserScan,
            SUB_TOPIC_NAME,
            self.lidar_callback,
            self.qos_profile
        )

        # 발행자 설정
        self.obstacle_publisher = self.create_publisher(
            Bool,
            PUB_OBSTACLE_INFO_TOPIC_NAME,
            self.qos_profile
        )
        self.start_angle_publisher = self.create_publisher(
            Float32,
            PUB_START_ANGLE_TOPIC_NAME,
            self.qos_profile
        )
        self.end_angle_publisher = self.create_publisher(
            Float32,
            PUB_END_ANGLE_TOPIC_NAME,
            self.qos_profile
        )

        # 안정적 감지를 위한 StabilityDetector 초기화
        self.detection_checker = LPFL.StabilityDetector(
            consec_count=CONSECUTIVE_DETECTION_COUNT
        )

        self.get_logger().info('Parking Obstacle Detector Node initialized')

    def lidar_callback(self, msg: LaserScan):
        """LiDAR 데이터 수신 콜백 함수"""
        ranges = np.array(msg.ranges)
        angle_min = msg.angle_min
        angle_increment = msg.angle_increment

        # 유효한 데이터 인덱스 찾기
        valid_indices = np.where(np.isfinite(ranges))[0]
        if not valid_indices.size:
            self.get_logger().debug("No valid LiDAR range data received.")
            return

        # 디버깅: 주요 방향의 장애물 거리 확인 (주기적으로)
        if not hasattr(self, '_debug_counter'):
            self._debug_counter = 0
        self._debug_counter += 1

        if self._debug_counter % 20 == 0:  # 2초마다 (10Hz * 20)
            angles_to_check = [0, 90, 180, 270]
            debug_info = []
            for check_angle in angles_to_check:
                idx = int((np.radians(check_angle) - angle_min) / angle_increment)
                if 0 <= idx < len(ranges) and np.isfinite(ranges[idx]):
                    debug_info.append(f"{check_angle}°:{ranges[idx]:.2f}m")
            self.get_logger().info(f"LiDAR ranges - {', '.join(debug_info)}")

        # ===== 1. 오른쪽 장애물 감지 (Bool 메시지 발행) =====
        self.detect_right_obstacle(valid_indices, ranges, angle_min, angle_increment)

        # ===== 2. 후방 장애물 각도 감지 (Float32 메시지 발행) =====
        self.detect_rear_obstacle_angles(valid_indices, ranges, angle_min, angle_increment)

    def detect_right_obstacle(self, valid_indices, ranges, angle_min, angle_increment):
        """
        오른쪽 영역(230~300도)의 장애물을 감지하여 Bool 메시지로 발행
        """
        right_detected = False
        right_obstacles = []  # 디버깅용

        for index in valid_indices:
            angle_deg = np.degrees(angle_min + index * angle_increment)
            distance = ranges[index]

            # 오른쪽 감지 영역 체크
            if (RIGHT_DETECTION_START_ANGLE <= angle_deg <= RIGHT_DETECTION_END_ANGLE and
                RIGHT_DETECTION_RANGE_MIN <= distance <= RIGHT_DETECTION_RANGE_MAX):
                right_detected = True
                right_obstacles.append((angle_deg, distance))

        # 디버깅 로그 (오른쪽 영역에서 발견된 장애물)
        if right_obstacles:
            self.get_logger().info(
                f"Right obstacles found: {len(right_obstacles)} points, "
                f"First: {right_obstacles[0][0]:.1f}° @ {right_obstacles[0][1]:.2f}m"
            )

        # StabilityDetector를 통한 안정적 감지
        detection_result = self.detection_checker.check_consecutive_detections(right_detected)

        # Bool 메시지 발행
        obstacle_bool_msg = Bool()
        obstacle_bool_msg.data = detection_result
        self.obstacle_publisher.publish(obstacle_bool_msg)

        if detection_result:
            self.get_logger().info(f"Right obstacle detected (stable) - {len(right_obstacles)} points")
        elif right_detected:
            self.get_logger().debug(f"Right obstacle detected but not stable yet")

    def detect_rear_obstacle_angles(self, valid_indices, ranges, angle_min, angle_increment):
        """
        후방 영역의 장애물 시작/끝 각도를 감지하여 Float32 메시지로 발행
        """
        # 파라미터 읽기
        start_angle_param = self.get_parameter('start_angle').get_parameter_value().double_value
        end_angle_param = self.get_parameter('end_angle').get_parameter_value().double_value
        range_min_param = self.get_parameter('range_min').get_parameter_value().double_value
        range_max_param = self.get_parameter('range_max').get_parameter_value().double_value
        angle_threshold_deg = self.get_parameter('angle_threshold').get_parameter_value().double_value

        # 후방 영역 필터링
        filtered_indices = []
        for index in valid_indices:
            angle_deg = np.degrees(angle_min + index * angle_increment)
            distance = ranges[index]

            if (start_angle_param <= angle_deg <= end_angle_param and
                range_min_param <= distance <= range_max_param):
                filtered_indices.append(index)

        if not filtered_indices:
            self.get_logger().debug(
                f"No rear obstacle data in range "
                f"({start_angle_param:.1f}~{end_angle_param:.1f} deg, "
                f"{range_min_param:.1f}~{range_max_param:.1f} m)"
            )
            return

        # 세그먼트 분리: 연속된 인덱스를 그룹화
        object_segments = self.segment_obstacles(filtered_indices)

        # 세그먼트 병합: 가까운 각도의 세그먼트를 병합
        merged_segments = self.merge_segments(
            object_segments,
            angle_min,
            angle_increment,
            angle_threshold_deg
        )

        # 각 세그먼트의 시작/끝 각도 발행
        for start_idx, end_idx in merged_segments:
            start_angle_deg = np.degrees(angle_min + start_idx * angle_increment)
            end_angle_deg = np.degrees(angle_min + end_idx * angle_increment)
            min_range_segment = np.min(ranges[start_idx:end_idx+1])

            self.get_logger().info(
                f'Rear Obstacle - Angles: [{start_angle_deg:.2f}° ~ {end_angle_deg:.2f}°], '
                f'Min Range: {min_range_segment:.2f} m'
            )

            # 시작 각도 발행
            start_angle_msg = Float32()
            start_angle_msg.data = float(start_angle_deg)
            self.start_angle_publisher.publish(start_angle_msg)

            # 끝 각도 발행
            end_angle_msg = Float32()
            end_angle_msg.data = float(end_angle_deg)
            self.end_angle_publisher.publish(end_angle_msg)

    def segment_obstacles(self, filtered_indices):
        """
        연속된 인덱스를 세그먼트로 그룹화
        """
        if not filtered_indices:
            return []

        object_segments = []
        start_index = filtered_indices[0]
        last_index = start_index

        for i in range(1, len(filtered_indices)):
            if filtered_indices[i] == last_index + 1:
                last_index = filtered_indices[i]
            else:
                object_segments.append((start_index, last_index))
                start_index = filtered_indices[i]
                last_index = start_index

        object_segments.append((start_index, last_index))
        return object_segments

    def merge_segments(self, object_segments, angle_min, angle_increment, angle_threshold_deg):
        """
        가까운 각도의 세그먼트를 병합
        """
        if not object_segments:
            return []

        merged_segments = []
        current_start, current_end = object_segments[0]

        for i in range(1, len(object_segments)):
            next_start, next_end = object_segments[i]
            current_end_angle = np.degrees(angle_min + current_end * angle_increment)
            next_start_angle = np.degrees(angle_min + next_start * angle_increment)

            # 세그먼트 간 각도 차이가 임계값보다 작으면 병합
            if (next_start == current_end + 1 or
                (next_start_angle - current_end_angle) <= angle_threshold_deg):
                current_end = next_end
            else:
                merged_segments.append((current_start, current_end))
                current_start, current_end = next_start, next_end

        merged_segments.append((current_start, current_end))
        return merged_segments


def main(args=None):
    rclpy.init(args=args)
    node = ParkingObstacleDetector()

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

