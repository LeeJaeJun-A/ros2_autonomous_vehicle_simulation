import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String, Bool, Int8

from rclpy.qos import QoSProfile
from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSReliabilityPolicy
from rclpy.qos import qos_profile_sensor_data   # ← 이미 rclpy에서 준비돼 있음
from .lib import lidar_perception_func_lib as LPFL

#---------------Variable Setting---------------
# Subscribe할 토픽 이름
SUB_TOPIC_NAME = 'lidar_processed'  # 구독할 토픽 이름

# Publish할 토픽 이름
PUB_TOPIC_NAME = 'lidar_obstacle_info'  # 물체 감지 여부를 퍼블리시할 토픽 이름

#----------------------------------------------

class StabilityDetector:
    def __init__(self, consec_count=3):
        self.consec_count = consec_count
        self.counter = 0

    def check_consecutive_detections(self, detected: bool) -> bool:
        if detected:
            self.counter += 1
        else:
            self.counter = 0
        return self.counter >= self.consec_count
    

class ObjectDetection(Node):
    def __init__(self):
        super().__init__('lidar_obstacle_detector_node')

        self.qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=1
        )

        self.subscriber = self.create_subscription(LaserScan, SUB_TOPIC_NAME, self.lidar_callback, self.qos_profile)
        self.publisher = self.create_publisher(Int8, PUB_TOPIC_NAME, self.qos_profile) 

        # self.detection_checker = LPFL.StabilityDetector(consec_count=3) # 연속적으로 몇 번 감지 여부를 확인할지 설정
        self.detector_left  = StabilityDetector(consec_count=3)
        self.detector_right = StabilityDetector(consec_count=3)


    def lidar_callback(self, msg):
        ranges = msg.ranges
        front_result = False
        rear_result = False

        # ── (A) 전방 / 측면 장애물 검사 ─────────────────────────
        LEFT_detected = LPFL.detect_object(
            ranges     = ranges,
            start_angle= 30,      # 0°~30° 예시
            end_angle  = 60,
            range_min  = 0.8,
            range_max  = 3.0
        )
        front_result = self.detector_left.check_consecutive_detections(LEFT_detected)

        # ── (B) 우측 후방 장애물 검사 ────────────────────────
        RIGHT_detected = LPFL.detect_object(
            ranges     = ranges,
            start_angle= 260,    # 150°~180° 정도가 로봇 기준 “오른쪽 뒤”
            end_angle  = 300,
            range_min  = 0.2,
            range_max  = 1.0     # 뒤쪽은 여유 있게
        )
        rear_result = self.detector_right.check_consecutive_detections(RIGHT_detected)


#####################중요,,,,,,,,, 여기서 각도 설정할때 0도 기준으로 차 머리 앞쪽으로 각도가 증가함.

        detection_result = 0

        if rear_result is True:
            detection_result = 1

        elif front_result is True:
            detection_result = 2

        detection_msg = Int8()
        detection_msg.data = detection_result
        self.publisher.publish(detection_msg)

        # ranges는 라이다 센서값 입력                                                        
        self.get_logger().info("lidar_operating")
        
        self.get_logger().info(
            f"L:{LEFT_detected} R:{RIGHT_detected} | "
            f"cntL:{self.detector_left.counter} cntR:{self.detector_right.counter} | "
            f"front:{front_result} rear:{rear_result} -> {detection_result}"
        )      
          # 각도 범위 지정
        # 예시 1) 
        # start_angle을 355도로, end_angle을 4도로 설정하면, 
        # 355도에서 4도까지의 모든 각도(355, 356, 357, 358, 359, 0, 1, 2, 3, 4도)가 포함.
        # 
        # 예시 2)
        # start_angle을 0도로, end_angle을 30도로 설정하면, 
        # 0도에서 30도까지의 모든 각도(0, 1, 2, ..., 30도)가 포함.
        # 
        # 예시 3)
        # start_angle을 180도로, end_angle을 190도로 설정하면, 
        # 180도에서 190도까지의 모든 각도(180, 181, 182, ..., 190도)가 포함. 

        # 거리범위 지정 
        # range_min보다 크거나 같고, range_max보다 작거나 같은 거리값을 포함.

        # 각도범위 및 거리범위를 둘 다 만족하는 범위에 라이다 센서값이 존재하면 True, 아니면 False 리턴. 

        good = []
        for i,d in enumerate(ranges):
            if 0.8 <= d <= 4.0:      # 지금 주신 range_min/max
                good.append(i)
        self.get_logger().info(f"[DEBUG] pass-dist idx = {good[:20]} … {good[-20:]}")


def main(args=None):
    rclpy.init(args=args)
    object_detection_node = ObjectDetection()
    rclpy.spin(object_detection_node)
    object_detection_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
