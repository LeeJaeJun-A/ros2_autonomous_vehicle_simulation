import rclpy
import math
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSDurabilityPolicy, QoSReliabilityPolicy
from std_msgs.msg import String, Bool
from interfaces_pkg.msg import PathPlanningResult, DetectionArray, MotionCommand
from .lib import decision_making_func_lib as DMFL

#--------------- 변수 설정 ---------------
SUB_DETECTION_TOPIC_NAME = "detections"
SUB_PATH_TOPIC_NAME = "path_planning_result"
SUB_TRAFFIC_LIGHT_TOPIC_NAME = "yolov8_traffic_light_info"
SUB_LIDAR_OBSTACLE_TOPIC_NAME = "lidar_obstacle_info"
PUB_TOPIC_NAME = "topic_control_signal"
TIMER = 0.1  # 타이머 주기 (초)

class MotionPlanningNode(Node):
    NEAR_Y_MIN = 380  # 횡단보도 근접 y값 기준
    FAR_Y_MAX = 300   # 횡단보도 멀어짐 y값 기준

    def __init__(self):
        super().__init__('motion_planner_node')

        # 파라미터 선언 및 토픽 이름 초기화
        self.sub_detection_topic = self.declare_parameter('sub_detection_topic', SUB_DETECTION_TOPIC_NAME).value
        self.sub_path_topic = self.declare_parameter('sub_lane_topic', SUB_PATH_TOPIC_NAME).value
        self.sub_traffic_light_topic = self.declare_parameter('sub_traffic_light_topic', SUB_TRAFFIC_LIGHT_TOPIC_NAME).value
        self.sub_lidar_obstacle_topic = self.declare_parameter('sub_lidar_obstacle_topic', SUB_LIDAR_OBSTACLE_TOPIC_NAME).value
        self.pub_topic = self.declare_parameter('pub_topic', PUB_TOPIC_NAME).value
        self.timer_period = self.declare_parameter('timer', TIMER).value

        # ROS 2 QoS 설정
        self.qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=1
        )

        # 변수 초기화
        self.detection_data = None
        self.path_data = None
        self.traffic_light_data = None
        self.lidar_data = None
        self.crosswalk_present = False

        self.steering_command = 0
        self.left_speed_command = 0
        self.right_speed_command = 0

        self.lane_control = False  # False: 2차선, True: 1차선
        self.traffic_light_flag_toggled = False
        self.crosswalk_near = False
        self.crosswalk_far = False
        self.cross_walk_flag_toggled = False
        self.prev_delta = 0.0
        self.traffic_light = False
        self.traffic_light_bbox = None
        self.steering_chop_flag = False

        # 구독자 설정
        self.detection_sub = self.create_subscription(DetectionArray, self.sub_detection_topic, self.detection_callback, self.qos_profile)
        self.path_sub = self.create_subscription(PathPlanningResult, self.sub_path_topic, self.path_callback, self.qos_profile)
        self.traffic_light_sub = self.create_subscription(String, self.sub_traffic_light_topic, self.traffic_light_callback, self.qos_profile)
        self.lidar_sub = self.create_subscription(Bool, self.sub_lidar_obstacle_topic, self.lidar_callback, self.qos_profile)

        # 퍼블리셔 설정
        self.publisher = self.create_publisher(MotionCommand, self.pub_topic, self.qos_profile)
        self.flag_pub = self.create_publisher(Bool, "flag_topic", self.qos_profile)

        # 타이머 설정 (주기적으로 timer_callback 실행)
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

    # detection 토픽 콜백 함수
    def detection_callback(self, msg: DetectionArray):
        self.detection_data = msg
        self.traffic_light = False
        self.traffic_light_bbox = None

        # 교통 신호등 정보 추출
        for det in msg.detections:
            if det.class_name == "traffic_light":
                self.traffic_light = True
                cx = det.bbox.center.position.x
                cy = det.bbox.center.position.y
                w = det.bbox.size.x
                h = det.bbox.size.y
                self.prev_h = h
                self.traffic_light_bbox = (cx, cy, w, h)
                self.get_logger().info(f"[DEBUG] h={h:.1f}, prev_h={self.prev_h}, toggled={self.traffic_light_flag_toggled}")

        # bbox 하단 y 좌표 계산 함수
        def bottom_y(det):
            return det.bbox.center.position.y + det.bbox.size.y / 2

        # 횡단보도 근접 판단
        self.crosswalk_near = any(
            det.class_name == "traffic_light_data" and bottom_y(det) >= self.NEAR_Y_MIN
            for det in msg.detections
        )

        # 횡단보도 지나감 판단
        self.crosswalk_far = any(
            det.class_name == "traffic_light_data" and bottom_y(det) <= self.FAR_Y_MAX
            for det in msg.detections
        )

    # 경로 토픽 콜백 함수
    def path_callback(self, msg: PathPlanningResult):
        self.path_data = list(zip(msg.x_points, msg.y_points))

    def traffic_light_callback(self, msg: String):
        self.traffic_light_data = msg

    def lidar_callback(self, msg: Bool):
        self.lidar_data = msg

    # 간단한 steering 계산 함수 (1차선용)
    def compute_simple_steering_lane1(self) -> int:
        if not self.path_data or len(self.path_data) < 10:
            return 0
        target_slope = DMFL.calculate_slope_between_points(self.path_data[-5], self.path_data[-1])
        steer = int(target_slope * 40 / 100)
        return max(-9, min(9, steer))

    # 간단한 steering 계산 함수 (2차선용)
    def compute_simple_steering_lane2(self) -> int:
        if not self.path_data or len(self.path_data) < 10:
            return 0
        target_slope = DMFL.calculate_slope_between_points(self.path_data[-5], self.path_data[-1])
        steer = int(target_slope * 30 / 100)
        return max(-9, min(9, steer))

    # 메인 제어 루프
    def timer_callback(self):
        # 신호등 bbox 높이를 기반으로 차선 변경
        if self.traffic_light_bbox is not None:
            x, y, w, h = self.traffic_light_bbox
            if h > 25 and not self.traffic_light_flag_toggled:
                self.lane_control = not self.lane_control
                self.traffic_light_flag_toggled = True
            if h < 25 and self.traffic_light_flag_toggled:
                self.traffic_light_flag_toggled = False

        # 2차선 로직
        if self.lane_control is False:
            if self.path_data is None:
                self.steering_command = 0
            else:
                self.steering_command = self.compute_simple_steering_lane2()
                if self.traffic_light:
                    x, y, w, h = self.traffic_light_bbox
                    if h >= 30:
                        self.left_speed_command = 80
                        self.right_speed_command = 80
                else:
                    self.left_speed_command = 150
                    self.right_speed_command = 150

        # 1차선 로직
        if self.lane_control:
            if self.path_data is None:
                self.steering_command = 0
            else:
                self.steering_command = self.compute_simple_steering_lane1()
                if self.traffic_light:
                    x, y, w, h = self.traffic_light_bbox
                    if h >= 30:
                        self.left_speed_command = 80
                        self.right_speed_command = 80
                else:
                    self.left_speed_command = 100
                    self.right_speed_command = 100

        # 현재 명령 출력 로그
        self.get_logger().info(f"steering: {self.steering_command}, "
                               f"left_speed: {self.left_speed_command}, "
                               f"right_speed: {self.right_speed_command}")

        # 모션 명령 메시지 퍼블리시
        motion_command_msg = MotionCommand()
        motion_command_msg.steering = self.steering_command
        motion_command_msg.left_speed = self.left_speed_command
        motion_command_msg.right_speed = self.right_speed_command
        self.publisher.publish(motion_command_msg)

        # 현재 차선 상태 플래그 퍼블리시
        flag_msg = Bool()
        flag_msg.data = self.lane_control
        self.flag_pub.publish(flag_msg)

        # 차선 상태 출력 로그
        self.get_logger().info(
            f"[Debug] lane_control={self.lane_control} (use_lane={'lane1' if self.lane_control else 'lane2'})"
        )

# 노드 실행 메인 함수

def main(args=None):
    rclpy.init(args=args)
    node = MotionPlanningNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n\nshutdown\n\n")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

