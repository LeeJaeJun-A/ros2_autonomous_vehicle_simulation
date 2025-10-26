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
SUB_TOPIC_NAME = 'lidar_processed'  # 구독할 라이다 토픽 이름

PUB_TOPIC_NAME = 'lidar_obstacle_info'  # 물체 감지 여부 퍼블리시
PUB_START_ANGLE_TOPIC_NAME = 'obstacle_start_angle'
PUB_END_ANGLE_TOPIC_NAME = 'obstacle_end_angle'
#------------------------------------------------

class ObjectDetection(Node):
    def __init__(self):
        super().__init__('lidar_obstacle_detector_node')

        self.declare_parameter('start_angle', 0.0)
        self.declare_parameter('end_angle', 359.0)
        self.declare_parameter('range_min', 0.5)
        self.declare_parameter('range_max', 3.0)
        self.declare_parameter('angle_threshold', 10.5)  # 병합 임계값 (deg)

        self.qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=1
        )

        self.subscriber = self.create_subscription(LaserScan, SUB_TOPIC_NAME, self.lidar_callback, self.qos_profile)
        self.obstacle_publisher = self.create_publisher(Bool, PUB_TOPIC_NAME, self.qos_profile)
        self.start_angle_publisher = self.create_publisher(Float32, PUB_START_ANGLE_TOPIC_NAME, self.qos_profile)
        self.end_angle_publisher = self.create_publisher(Float32, PUB_END_ANGLE_TOPIC_NAME, self.qos_profile)

        self.detection_checker = LPFL.StabilityDetector(consec_count=3)  # <이거 원래 5엿음 내일의 나 기억해

    def lidar_callback(self, msg):
        ranges = np.array(msg.ranges)
        angle_min = msg.angle_min
        angle_increment = msg.angle_increment
        angle_threshold_deg = self.get_parameter('angle_threshold').get_parameter_value().double_value

        # [기존 유지] 파라미터 읽기
        start_angle_param = self.get_parameter('start_angle').get_parameter_value().double_value
        end_angle_param = self.get_parameter('end_angle').get_parameter_value().double_value
        range_min_param = self.get_parameter('range_min').get_parameter_value().double_value
        range_max_param = self.get_parameter('range_max').get_parameter_value().double_value

        valid_indices = np.where(np.isfinite(ranges))[0]
        if not valid_indices.size:
            self.get_logger().info("No valid range data received.")
            return

        # [🆕 추가] 오른쪽 영역에 대한 Bool 퍼블리시 처리
        right_start_angle = 230.0
        right_end_angle = 300.0
        right_range_min = 0.5
        right_range_max = 2.5

        right_detected = False
        for index in valid_indices:
            angle_deg = np.degrees(angle_min + index * angle_increment)
            distance = ranges[index]
            if (right_start_angle <= angle_deg <= right_end_angle) and right_range_min <= distance <= right_range_max:
                right_detected = True
                break

        detection_result = self.detection_checker.check_consecutive_detections(right_detected)

        obstacle_bool_msg = Bool()
        obstacle_bool_msg.data = detection_result
        self.obstacle_publisher.publish(obstacle_bool_msg)

        # [기존 유지] 전방위 각도 필터링
        filtered_indices = []
        for index in valid_indices:
            angle_deg = np.degrees(angle_min + index * angle_increment)
            distance = ranges[index]
            if (start_angle_param <= angle_deg <= end_angle_param) and range_min_param <= distance <= range_max_param:
                filtered_indices.append(index)

        if not filtered_indices:
            self.get_logger().info(
                f"No relevant data to log (excluding {start_angle_param:.2f} ~ {end_angle_param:.2f} deg, "
                f"{range_min_param:.2f} ~ {range_max_param:.2f} m).")
            return

        filtered_indices = np.array(filtered_indices)
        if not filtered_indices.size:
            return

        # [기존 유지] 세그먼트 분리 및 병합
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

        merged_segments = []
        if object_segments:
            current_start, current_end = object_segments[0]
            for i in range(1, len(object_segments)):
                next_start, next_end = object_segments[i]
                current_end_angle = np.degrees(angle_min + current_end * angle_increment)
                next_start_angle = np.degrees(angle_min + next_start * angle_increment)
                if next_start == current_end + 1 or (next_start_angle - current_end_angle) <= angle_threshold_deg:
                    current_end = next_end
                else:
                    merged_segments.append((current_start, current_end))
                    current_start, current_end = next_start, next_end
            merged_segments.append((current_start, current_end))

        # [기존 유지] 시작/끝 각도 퍼블리시
        for start_idx, end_idx in merged_segments:
            start_angle_deg = np.degrees(angle_min + start_idx * angle_increment)
            end_angle_deg = np.degrees(angle_min + end_idx * angle_increment)
            min_range_segment = np.min(ranges[start_idx:end_idx+1])

            self.get_logger().info(
                f'Angles: [{start_angle_deg:.2f} ~ {end_angle_deg:.2f}] deg, Min Range: {min_range_segment:.2f} m')

            start_angle_msg = Float32()
            start_angle_msg.data = float(start_angle_deg)
            self.start_angle_publisher.publish(start_angle_msg)

            end_angle_msg = Float32()
            end_angle_msg.data = float(end_angle_deg)
            self.end_angle_publisher.publish(end_angle_msg)

def main(args=None):
    rclpy.init(args=args)
    object_detection_node = ObjectDetection()
    rclpy.spin(object_detection_node)
    object_detection_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()