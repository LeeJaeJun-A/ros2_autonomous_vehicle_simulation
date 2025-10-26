"""
Parking Lane Detector Node
주차를 위한 카메라 기반 차선 검출 노드

기능:
1. YOLOv8 Detection에서 주차 공간의 차선 추출
2. Bird's eye view 변환으로 차선의 정확한 위치 파악
3. 차량과 주차 공간 중앙선 간의 오프셋 계산
4. LaneInfo 메시지로 차선 중심점 및 각도 정보 발행
"""

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSReliabilityPolicy

from cv_bridge import CvBridge

from sensor_msgs.msg import Image
from std_msgs.msg import Float32, Bool
from interfaces_pkg.msg import TargetPoint, LaneInfo, DetectionArray
from .lib import camera_perception_func_lib as CPFL

#---------------Variable Setting---------------
# Subscribe할 토픽 이름
SUB_DETECTION_TOPIC_NAME = "detections"
SUB_PARKING_MODE_TOPIC_NAME = "parking_mode"  # 주차 모드 활성화 신호

# Publish할 토픽 이름
PUB_LANE_INFO_TOPIC_NAME = "parking_lane_info"
PUB_ROI_IMAGE_TOPIC_NAME = "parking_roi_image"
PUB_LATERAL_OFFSET_TOPIC_NAME = "parking_lateral_offset"  # 좌우 오프셋
PUB_LANE_END_DETECTED_TOPIC_NAME = "parking_lane_end_detected"  # 주차선 끝 감지

# 화면에 이미지를 처리하는 과정을 띄울것인지 여부
SHOW_IMAGE = True

# 주차 차선 검출 파라미터
PARKING_LANE_CLASS = 'lane2'  # 주차 공간 차선 클래스 이름
TARGET_POINT_Y_RANGE = range(5, 155, 30)  # 목표 지점 Y 좌표 범위
LANE_WIDTH = 300  # 차선 폭 (픽셀)
DETECTION_THICKNESS = 10  # 검출 두께 (픽셀)
THETA_LIMIT = 70  # 기울기 각도 제한 (차량 앞부분 제외)

# Bird's eye view 변환 파라미터
# 실제 카메라 설정에 따라 조정 필요
SRC_POINTS = [[238, 316], [402, 313], [501, 476], [155, 476]]  # 원본 이미지 좌표
#----------------------------------------------


class ParkingLaneDetector(Node):
    def __init__(self):
        super().__init__('parking_lane_detector_node')

        # 파라미터 선언
        self.sub_detection_topic = self.declare_parameter(
            'sub_detection_topic', SUB_DETECTION_TOPIC_NAME
        ).value
        self.pub_lane_info_topic = self.declare_parameter(
            'pub_lane_info_topic', PUB_LANE_INFO_TOPIC_NAME
        ).value
        self.show_image = self.declare_parameter('show_image', SHOW_IMAGE).value

        self.cv_bridge = CvBridge()

        # QoS 설정
        self.qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=1
        )

        # 구독자 설정
        self.detection_subscriber = self.create_subscription(
            DetectionArray,
            self.sub_detection_topic,
            self.detection_callback,
            self.qos_profile
        )

        # 발행자 설정
        self.lane_info_publisher = self.create_publisher(
            LaneInfo,
            self.pub_lane_info_topic,
            self.qos_profile
        )
        self.roi_image_publisher = self.create_publisher(
            Image,
            PUB_ROI_IMAGE_TOPIC_NAME,
            self.qos_profile
        )
        self.lateral_offset_publisher = self.create_publisher(
            Float32,
            PUB_LATERAL_OFFSET_TOPIC_NAME,
            self.qos_profile
        )
        self.lane_end_publisher = self.create_publisher(
            Bool,
            PUB_LANE_END_DETECTED_TOPIC_NAME,
            self.qos_profile
        )

        # 내부 변수
        self.parking_mode_active = True  # 주차 모드 활성화 (기본값)
        self.image_center_x = None  # 이미지 중심 X 좌표

        self.get_logger().info('Parking Lane Detector Node initialized')

    def detection_callback(self, detection_msg: DetectionArray):
        """YOLOv8 Detection 수신 콜백 함수"""
        if len(detection_msg.detections) == 0:
            self.get_logger().debug("No detections received")
            return

        # 주차 차선 에지 이미지 생성
        lane_edge_image = CPFL.draw_edges(
            detection_msg,
            cls_name=PARKING_LANE_CLASS,
            color=255
        )

        if lane_edge_image is None or lane_edge_image.size == 0:
            self.get_logger().debug(f"No {PARKING_LANE_CLASS} detected")
            return

        # 이미지 크기 확인
        h, w = lane_edge_image.shape[0], lane_edge_image.shape[1]
        if self.image_center_x is None:
            self.image_center_x = w / 2.0

        # Bird's eye view 변환
        dst_mat = [
            [round(w * 0.3), round(h * 0.0)],
            [round(w * 0.7), round(h * 0.0)],
            [round(w * 0.7), h],
            [round(w * 0.3), h]
        ]

        bird_image = CPFL.bird_convert(
            lane_edge_image,
            srcmat=SRC_POINTS,
            dstmat=dst_mat
        )

        # ROI 영역 추출 (하단 부분)
        roi_image = CPFL.roi_rectangle_below(bird_image, cutting_idx=300)

        # 디버깅용 이미지 표시
        if self.show_image:
            cv2.imshow('Parking - Lane Edge', lane_edge_image)
            cv2.imshow('Parking - Bird View', bird_image)
            cv2.imshow('Parking - ROI', roi_image)
            cv2.waitKey(1)

        # ROI 이미지를 uint8로 변환 및 발행
        roi_image = cv2.convertScaleAbs(roi_image)
        try:
            roi_image_msg = self.cv_bridge.cv2_to_imgmsg(roi_image, encoding="mono8")
            self.roi_image_publisher.publish(roi_image_msg)
        except Exception as e:
            self.get_logger().error(f"Failed to convert and publish ROI image: {e}")

        # 주 차선 기울기 계산
        grad = CPFL.dominant_gradient(roi_image, theta_limit=THETA_LIMIT)

        # 여러 높이에서 차선 중심점 검출
        target_points = []
        lateral_offsets = []

        for target_point_y in TARGET_POINT_Y_RANGE:
            target_point_x = CPFL.get_lane_center(
                roi_image,
                detection_height=target_point_y,
                detection_thickness=DETECTION_THICKNESS,
                road_gradient=grad,
                lane_width=LANE_WIDTH
            )

            target_point = TargetPoint()
            target_point.target_x = round(target_point_x)
            target_point.target_y = round(target_point_y)
            target_points.append(target_point)

            # 좌우 오프셋 계산 (ROI 이미지 중심 기준)
            roi_center_x = roi_image.shape[1] / 2.0
            lateral_offset = target_point_x - roi_center_x
            lateral_offsets.append(lateral_offset)

        # LaneInfo 메시지 생성 및 발행
        lane_info = LaneInfo()
        lane_info.slope = grad
        lane_info.target_points = target_points
        self.lane_info_publisher.publish(lane_info)

        # 평균 좌우 오프셋 발행 (주차 정렬 제어용)
        if lateral_offsets:
            avg_lateral_offset = np.mean(lateral_offsets)
            lateral_offset_msg = Float32()
            lateral_offset_msg.data = float(avg_lateral_offset)
            self.lateral_offset_publisher.publish(lateral_offset_msg)

            self.get_logger().info(
                f"Parking Lane - Slope: {grad:.2f}°, "
                f"Lateral Offset: {avg_lateral_offset:.1f} px"
            )

        # 주차선 끝 감지 (ROI 이미지 하단부 체크)
        lane_end_detected = self.detect_lane_end(roi_image)
        lane_end_msg = Bool()
        lane_end_msg.data = lane_end_detected
        self.lane_end_publisher.publish(lane_end_msg)

        if lane_end_detected:
            self.get_logger().warn("🛑 Parking lane END detected! Stop reversing!")

    def detect_lane_end(self, roi_image):
        """
        ROI 이미지 하단부에서 주차선 끝을 감지
        하단 20% 영역에 많은 흰색 픽셀이 있으면 주차선 끝으로 판단
        """
        h, w = roi_image.shape[0], roi_image.shape[1]

        # 하단 20% 영역만 확인
        bottom_region_start = int(h * 0.8)
        bottom_region = roi_image[bottom_region_start:h, :]

        # 흰색 픽셀 개수 세기 (임계값 200 이상)
        white_pixels = np.sum(bottom_region > 200)
        total_pixels = bottom_region.size

        # 흰색 픽셀 비율 계산
        white_ratio = white_pixels / total_pixels if total_pixels > 0 else 0

        # 흰색 픽셀이 20% 이상이면 주차선 끝으로 판단
        is_lane_end = white_ratio > 0.20

        if is_lane_end:
            self.get_logger().debug(
                f"Lane end detection: white_ratio={white_ratio:.2%} > 20%"
            )

        return is_lane_end


def main(args=None):
    rclpy.init(args=args)
    node = ParkingLaneDetector()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n\nShutdown\n\n")
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

