import rclpy
import math         ##math 추가.

from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSReliabilityPolicy

from std_msgs.msg import String, Bool
from interfaces_pkg.msg import PathPlanningResult, DetectionArray, MotionCommand
from .lib import decision_making_func_lib as DMFL

#---------------Variable Setting---------------
SUB_DETECTION_TOPIC_NAME = "detections"
SUB_PATH_TOPIC_NAME = "path_planning_result"
SUB_TRAFFIC_LIGHT_TOPIC_NAME = "yolov8_traffic_light_info"
SUB_LIDAR_OBSTACLE_TOPIC_NAME = "lidar_obstacle_info"
PUB_TOPIC_NAME = "topic_control_signal"

#----------------------------------------------

# 모션 플랜 발행 주기 (초) - 소수점 필요 (int형은 반영되지 않음)
TIMER = 0.1

class MotionPlanningNode(Node):
    # 근접/멀리 감지 기준 (y 좌표, 픽셀 단위)
    NEAR_Y_MIN = 380    # 이 값 이상 내려오면 “근접”
    FAR_Y_MAX  = 300    # 이 값 이하로 올라가면 “완전히 지나감”
    ###------------------------------------

    def __init__(self):
        super().__init__('motion_planner_node')

        # 토픽 이름 설정
        self.sub_detection_topic = self.declare_parameter('sub_detection_topic', SUB_DETECTION_TOPIC_NAME).value
        self.sub_path_topic = self.declare_parameter('sub_lane_topic', SUB_PATH_TOPIC_NAME).value
        self.sub_traffic_light_topic = self.declare_parameter('sub_traffic_light_topic', SUB_TRAFFIC_LIGHT_TOPIC_NAME).value
        self.sub_lidar_obstacle_topic = self.declare_parameter('sub_lidar_obstacle_topic', SUB_LIDAR_OBSTACLE_TOPIC_NAME).value
        self.pub_topic = self.declare_parameter('pub_topic', PUB_TOPIC_NAME).value
        
        self.timer_period = self.declare_parameter('timer', TIMER).value

        # QoS 설정
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
        self.crosswalk_present   = False   # ←★ 초기값 False

        self.steering_command = 0
        self.left_speed_command = 0
        self.right_speed_command = 0
        
        ###########################3 추가 코드 시작
        #lane selecting 변수
        self.lane_control = False # false is lane2, true is lane 1.
        # 신호등 토글 관련 상태 변수 추가
        self.traffic_light_flag_toggled = False
        # 횡단보도 관련 상태 변수 추가.
        self.crosswalk_near = False
        self.crosswalk_far  = False
        self.cross_walk_flag_toggled = False
        self.prev_delta = 0.0
        self.traffic_light = False
        self.traffic_light_bbox = None  # 초기화
        self.steering_chop_flag = False
        ###########################3 추가 코드 끝

        # 서브스크라이버 설정
        self.detection_sub = self.create_subscription(DetectionArray, self.sub_detection_topic, self.detection_callback, self.qos_profile)
        self.path_sub = self.create_subscription(PathPlanningResult, self.sub_path_topic, self.path_callback, self.qos_profile)
        self.traffic_light_sub = self.create_subscription(String, self.sub_traffic_light_topic, self.traffic_light_callback, self.qos_profile)
        self.lidar_sub = self.create_subscription(Bool, self.sub_lidar_obstacle_topic, self.lidar_callback, self.qos_profile)

        # 퍼블리셔 설정
        self.publisher = self.create_publisher(MotionCommand, self.pub_topic, self.qos_profile)

        ###########################3 추가 코드 시작
        # 추가: flag 퍼블리셔 추가 (Bool 메시지, 토픽 이름은 "flag_topic")
        self.flag_pub = self.create_publisher(Bool, "flag_topic", self.qos_profile)
        ###########################3 추가 코드 끝


        # 타이머 설정
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

    # ── DetectionArray 콜백(횡단보도 인식) ────────────────────────────────
    def detection_callback(self, msg: DetectionArray):
        self.detection_data = msg

        self.traffic_light = False #초기화
        self.traffic_light_bbox = None

        for det in msg.detections:
            if det.class_name == "traffic_light":
                self.traffic_light = True
                cx = det.bbox.center.position.x
                cy = det.bbox.center.position.y
                w  = det.bbox.size.x
                h  = det.bbox.size.y
                self.prev_h = h
                self.traffic_light_bbox = (cx, cy, w, h)

        # bbox 하단(y) 계산
        def bottom_y(det):
            return det.bbox.center.position.y + det.bbox.size.y / 2

        # “근접” 횡단보도 판정
        self.crosswalk_near = any(
            det.class_name == "traffic_light_data" and bottom_y(det) >= self.NEAR_Y_MIN  ##횡단보도에서 신호등으로 변경상태.
            for det in msg.detections
        )
        
        # “지나감” 판정
        self.crosswalk_far = any(
            det.class_name == "traffic_light_data" and bottom_y(det) <= self.FAR_Y_MAX  ##횡단보도에서 신호등으로 변경상태.
            for det in msg.detections
        )
        for det in msg.detections:
            if det.class_name == "traffic_light":
                x = det.bbox.center.position.x
                y = det.bbox.center.position.y

    def path_callback(self, msg: PathPlanningResult):
        self.path_data = list(zip(msg.x_points, msg.y_points))
                
    def traffic_light_callback(self, msg: String):
        self.traffic_light_data = msg

    def lidar_callback(self, msg: Bool):
        self.lidar_data = msg
        

# ---------------------------------------------Stanley method 수정    
    def compute_steering_with_stanley1(self, v: float = 255.0, k: float = 0.80) -> int:  #For lane1.
        #1) 기울기 extract.
        if not self.path_data or len(self.path_data) <= 5:
            return 0

        else:
            target_slope = DMFL.calculate_slope_between_points(self.path_data[-8], self.path_data[-3])

        #2) 횡방향 오차 smoothing.
        x_sum = 0
        for i in range(5, 0, -1):
            x_target_sub, _ = self.path_data[-i]
            x_sum = x_target_sub + x_sum
        x_target = x_sum / abs(5)      

        #3) 횡방향 오차 보정치 (실길이)
        if target_slope > 0: 
            e_y_cm = (x_target - 480) / (640/50.0)

            # 4) Stanley 제어 공식을 적용
            cross_rad = math.atan2(k * e_y_cm, max(v, 1.0))
            cross_deg = math.degrees(cross_rad)

            # 5) 각도 매핑.
            delta = target_slope + cross_deg
            steer = round((delta) * 110/800)

#           if self.traffic_light_bbox is not None:
#                steer = steer + 1


        elif target_slope < 0: 
            e_y_cm = (x_target - 160) / (640/50.0)

            # 4) Stanley 제어 공식을 적용
            cross_rad = math.atan2(k * e_y_cm, max(v, 1.0))
            cross_deg = math.degrees(cross_rad)

            # 5) 각도 매핑.
            delta = target_slope + cross_deg
            steer = round((delta) * 110/800)

        if steer == -6 or steer == -5: #S자 구간 전용 steer boosting.
            return (steer -1)

        # if self.traffic_light_bbox is not None:
        #     x, y, w, h = self.traffic_light_bbox
        #     if h<40: #횡단보도 보면 급조향.
        #         steer = steer -1

        #6) 리턴 및 각도 리밋.
        return max(-9, min(9, steer))

    
    def compute_steering_with_stanley2(self, v: float = 255.0, k: float = 0.80) -> int: #For lane2.
        #1) 기울기 extract.
        if not self.path_data or len(self.path_data) <= 5:
            return 0

        else:
            target_slope = DMFL.calculate_slope_between_points(self.path_data[-8], self.path_data[-3])

        #2) 횡방향 오차 smoothing.
        x_sum = 0 
        for i in range(5, 0, -1):
            x_target_sub, _ = self.path_data[-i]
            x_sum = x_target_sub + x_sum
        x_target = x_sum / abs(5)      

        #3) 횡방향 오차 보정치 (실길이)
        if target_slope > 0: 
            e_y_cm = (x_target - 480) / (640/50.0)

        elif target_slope < 0: 
            e_y_cm = (x_target - 160) / (640/50.0)

        # 4) Stanley 제어 공식을 적용
        cross_rad = math.atan2(k * e_y_cm, max(v, 1.0))
        cross_deg = math.degrees(cross_rad)

        # 5) 각도 매핑.
        delta = target_slope + cross_deg
        steer = round((delta) * 100/800)

        # if steer >= 3 and steer < 5:
        #     steer -= 1
        # elif steer <= -3 and steer > -5:
        #     steer += 1

        # steer = self.apply_steering_chop(steer, delta)

        #6) 리턴 및 각도 리밋.
        return max(-9, min(9, steer))
    # -----------------------------------------------------------------------------------



    def timer_callback(self):
#--------------------------------------횡단보도 차선 변경 로직.
        # # # ── 1) 횡단보도 근접 토글/리셋
        # if self.traffic_light and not self.traffic_light_flag_toggled:   #수정 상태 점검 필요.
        #     x, y, w, h = self.traffic_light_bbox
        #     if self.lane_control and h >= 30:
        #         self.lane_control = not self.lane_control   #이게 차선변경
        #         self.traffic_light_flag_toggled = True      #toggle로 중복 방지.

        #     if not self.lane_control and h >= 20:
        #         self.lane_control = not self.lane_control   #이게 차선변경
        #         self.traffic_light_flag_toggled = True      #toggle로 중복 방지.


        # elif self.traffic_light and self.traffic_light_flag_toggled:
        #     x, y, w, h = self.traffic_light_bbox
        #     if not self.lane_control and h < 30:
        #         self.traffic_light_flag_toggled = False
        #     if self.lane_control and h < 20:
        #         self.traffic_light_flag_toggled = False

        

        if self.traffic_light_bbox is not None:
            x, y, w, h = self.traffic_light_bbox
            if h > 30 and not self.traffic_light_flag_toggled:
                self.lane_control = not self.lane_control
                self.traffic_light_flag_toggled = True

            # 신호등이 멀어지면 → flag 초기화
            if h < 25 and self.traffic_light_flag_toggled:
                self.traffic_light_flag_toggled = False
#--------------------------------------------------------------------------------------

    #    # ── 2) 장애물 감지
    #     if self.lidar_data is not None and self.lidar_data.data is True:
    #         # 라이다가 장애물을 감지한 경우
    #         self.steering_command = 0 
    #         self.left_speed_command = 0 
    #         self.right_speed_command = 0 

        # ── 3) 신호등 적색 정지
        # elif self.traffic_light_data is not None and self.traffic_light_data.data == 'Red':
            # 빨간색 신호등을 감지한 경우
            # ###########################3 추가 코드 시작
            # # 신호등이 처음 보이면 토글
            # if not self.traffic_light_flag_toggled:
            #     self.lane_control = not self.lane_control #이게 차선변경
            #     self.traffic_light_flag_toggled = True
            # ###########################3 추가 코드 끝

        #     for detection in self.detection_data.detections:
        #         if detection.class_name=='traffic_light':
        #             x_min = int(detection.bbox.center.position.x - detection.bbox.size.x / 2) # bbox의 좌측상단 꼭짓점 x좌표
        #             x_max = int(detection.bbox.center.position.x + detection.bbox.size.x / 2) # bbox의 우측하단 꼭짓점 x좌표
        #             y_min = int(detection.bbox.center.position.y - detection.bbox.size.y / 2) # bbox의 좌측상단 꼭짓점 y좌표
        #             y_max = int(detection.bbox.center.position.y + detection.bbox.size.y / 2) # bbox의 우측하단 꼭짓점 y좌표

        #             if y_max < 150:
        #                 # 신호등 위치에 따른 정지명령 결정
        #                 self.steering_command = 0 
        #                 self.left_speed_command = 0 
        #                 self.right_speed_command = 0

        #── 4) 정상 주행 (Stanley 적용)
        if self.lane_control is False: #2차선.
            if self.path_data is None:
                self.steering_command = 0
            else:
                current_v = (self.left_speed_command + self.right_speed_command)/2.0
                self.steering_command = self.compute_steering_with_stanley2(current_v, 0.8)

                if self.traffic_light == True:
                    x, y, w, h = self.traffic_light_bbox
                    if h >=20.0:
                        self.left_speed_command = 80  # 예시 속도 값 (255가 최대 속도)
                        self.right_speed_command = 80  # 예시 속도 값 (255가 최대 속도)

                elif self.traffic_light == False:
                    self.left_speed_command = 255  # 예시 속도 값 (255가 최대 속도)
                    self.right_speed_command = 255  # 예시 속도 값 (255가 최대 속도)
            
        if self.lane_control: #1차선.
            if self.path_data is None:
                self.steering_command = 0
            else:
                current_v = (self.left_speed_command + self.right_speed_command)/2.0
                self.steering_command = self.compute_steering_with_stanley1(current_v, 0.8)

                if self.traffic_light == True:
                    x, y, w, h = self.traffic_light_bbox
                    if h >= 20.0:
                        self.left_speed_command = 80  # 예시 속도 값 (255가 최대 속도)
                        self.right_speed_command = 80  # 예시 속도 값 (255가 최대 속도)

                elif self.traffic_light == False:
                    self.left_speed_command = 255  # 예시 속도 값 (255가 최대 속도)
                    self.right_speed_command = 255  # 예시 속도 값 (255가 최대 속도)

        # if self.path_data is None:
        #     self.steering_command = 0

        # else:
        #     target_slope = DMFL.calculate_slope_between_points(self.path_data[-10], self.path_data[-1])
        #     self.steering_command = int(target_slope * 5 / 80)
        #     if self.steering_command > 9 :
        #         self.steering_command  = 9
        #     elif self.steering_command <-9 :
        #         self.steering_command  = -9
        #     else :
        #         self.steering_command = self.steering_command 

    # 일정한 속도 유지






        self.get_logger().info(f"steering: {self.steering_command}, " 
                               f"left_speed: {self.left_speed_command}, " 
                               f"right_speed: {self.right_speed_command},"
                               f"traffic: {self.traffic_light},"
                               f"traffic_boxxx: {self.traffic_light_bbox},"
                               f"flag {self.traffic_light_flag_toggled}")

        # 모션 명령 메시지 생성 및 퍼블리시
        motion_command_msg = MotionCommand()
        motion_command_msg.steering = self.steering_command
        motion_command_msg.left_speed = self.left_speed_command
        motion_command_msg.right_speed = self.right_speed_command
        self.publisher.publish(motion_command_msg)
        

        ###########################3 수정 코드 시작
        # 수정: flag 퍼블리시 시 고정값 True 대신 self.lane_control 값을 사용
        flag_msg = Bool()
        flag_msg.data = self.lane_control
        self.flag_pub.publish(flag_msg)
        ###########################3 수정 코드 끝

    # timer_callback 마지막 부분, 퍼블리시 직전에 로그 추가
        self.get_logger().info(
            f"[Debug] lane_control={self.lane_control} "
            f"(use_lane={'lane1' if self.lane_control else 'lane2'})"
        )

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
