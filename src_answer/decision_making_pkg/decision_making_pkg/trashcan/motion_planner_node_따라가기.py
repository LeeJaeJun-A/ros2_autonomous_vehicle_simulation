import rclpy
import math         ##math 추가.

from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSReliabilityPolicy

from std_msgs.msg import String, Bool, Int8
from interfaces_pkg.msg import PathPlanningResult, DetectionArray, MotionCommand
from .lib import decision_making_func_lib as DMFL

from rclpy.qos import qos_profile_sensor_data   # ← 이미 rclpy에서 준비돼 있음

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
        # 왼쪽 / 오른쪽 LiDAR 토픽을 **서로 다른 이름**으로 등록
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
        self.lidar_right_data = None
        self.lidar_left_data = None
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
        self.cross_walk_flag_toggled = False
        self.prev_delta = 0.0
        self.traffic_light = False
        self.traffic_light_bbox = None  # 초기화
        self.steering_chop_flag = False
        self.car_rear_boxes = []

        self.waiting_for_car_alignment = False
        # evade state machine
        self.car_evade_mode = False
        self.evade_phase = None        # 'reverse','stop','align',None
        self.phase_start = None
        self.lidar_data = None
        self.stop_flag = False          #정지 코드.
        ###########################3 추가 코드 끝
        self.front = False

        # 서브스크라이버 설정
        self.detection_sub = self.create_subscription(DetectionArray, self.sub_detection_topic, self.detection_callback, self.qos_profile)
        self.path_sub = self.create_subscription(PathPlanningResult, self.sub_path_topic, self.path_callback, self.qos_profile)
        self.traffic_light_sub = self.create_subscription(String, self.sub_traffic_light_topic, self.traffic_light_callback, self.qos_profile)
        self.lidar_sub = self.create_subscription(Int8, self.sub_lidar_obstacle_topic, self.lidar_callback, self.qos_profile)

        # 퍼블리셔 설정
        self.publisher = self.create_publisher(MotionCommand, self.pub_topic, self.qos_profile)

        self.flag_pub = self.create_publisher(Bool, "flag_topic", self.qos_profile)


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
                self.get_logger().info(
                    f"[DEBUG] h={h:.1f}, prev_h={self.prev_h}, toggled={self.traffic_light_flag_toggled}"
                )
            
        car_rear_boxes = []
        for det in msg.detections:
            if det.class_name == "car_rear":
                cx = det.bbox.center.position.x
                cy = det.bbox.center.position.y
                w  = det.bbox.size.x
                h  = det.bbox.size.y
                car_rear_boxes.append({'cx': cx, 'cy': cy, 'w': w, 'h': h})

        self.car_rear_boxes = car_rear_boxes


    def path_callback(self, msg: PathPlanningResult):
        self.path_data = list(zip(msg.x_points, msg.y_points))
                
    def traffic_light_callback(self, msg: String):
        self.traffic_light_data = msg

    def lidar_callback(self, msg: Int8):
        self.lidar_data = msg.data


    def timer_callback(self):
        now = self.get_clock().now()
#--------------------------------------횡단보도 차선 변경 로직.
        # # ── 1) 횡단보도 근접 토글/리셋
        if self.traffic_light and not self.traffic_light_flag_toggled:   #수정 상태 점검 필요.
            x, y, w, h = self.traffic_light_bbox
            if h >= 45 and self.traffic_light_data.data == 'Red':
                self.steering_command = 0 
                self.left_speed_command = 0 
                self.right_speed_command = 0
                self.stop_flag = True
        #         self.lane_control = not self.lane_control   #이게 차선변경
                self.traffic_light_flag_toggled = True      #toggle로 중복 방지.

        #     if not self.lane_control and h >= 20:
        #         self.lane_control = not self.lane_control   #이게 차선변경
        #         self.traffic_light_flag_toggled = True      #toggle로 중복 방지.


        # elif self.traffic_light and self.traffic_light_flag_toggled:
        #     x, y, w, h = self.traffic_light_bbox
        #     if not self.lane_control and h < 30:
        #         self.traffic_light_flag_toggled = False
        #     if self.lane_control and h < 20:
        #         self.traffic_light_flag_toggled = False

        

        # if self.traffic_light_bbox is not None:
        #     x, y, w, h = self.traffic_light_bbox
        #     if h > 40 and not self.traffic_light_flag_toggled:
        #         self.lane_control = not self.lane_control
        #         self.traffic_light_flag_toggled = True

        #     # 신호등이 멀어지면 → flag 초기화
        #     if h < 40 and self.traffic_light_flag_toggled:
        #         self.traffic_light_flag_toggled = False
#--------------------------------------------------------------------------------------

    #    # ── 2) 장애물 감지
    #     if self.lidar_right_data is not None and self.lidar_right_data.data is True:
    #         # 라이다가 장애물을 감지한 경우
    #         self.steering_command = 0 
    #         self.left_speed_command = 0 
    #         self.right_speed_command = 0 

        # ── 3) 신호등 적색 정지
        # elif self.traffic_light_data is not None and self.traffic_light_data.data == 'Red':
            # 빨간색 신호등을 감지한 경우
            ###########################3 추가 코드 시작
            # 신호등이 처음 보이면 토글
            # if not self.traffic_light_flag_toggled:
            #     self.lane_control = not self.lane_control #이게 차선변경
            #     self.traffic_light_flag_toggled = True
            ###########################3 추가 코드 끝

            # for detection in self.detection_data.detections:
            #     if detection.class_name=='traffic_light':
            #         x_min = int(detection.bbox.center.position.x - detection.bbox.size.x / 2) # bbox의 좌측상단 꼭짓점 x좌표
            #         x_max = int(detection.bbox.center.position.x + detection.bbox.size.x / 2) # bbox의 우측하단 꼭짓점 x좌표
            #         y_min = int(detection.bbox.center.position.y - detection.bbox.size.y / 2) # bbox의 좌측상단 꼭짓점 y좌표
            #         y_max = int(detection.bbox.center.position.y + detection.bbox.size.y / 2) # bbox의 우측하단 꼭짓점 y좌표

            #         if y_max < 150:
            #             # 신호등 위치에 따른 정지명령 결정
            #             self.steering_command = 0 
            #             self.left_speed_command = 0 
            #             self.right_speed_command = 0


        # FSM for evade
        if self.evade_phase == 'reverse':
            if (now - self.phase_start).nanoseconds / 1e9 >= 2.4:
                self.evade_phase = 'stop'
                self.phase_start = now
                self.left_speed_command = self.right_speed_command = 100
                self.steering_command = -6

        elif self.evade_phase == 'stop':
            self.left_speed_command = self.right_speed_command = 100
            self.steering_command = -6
            if (now - self.phase_start).nanoseconds / 1e9 >= 4.0:
                self.evade_phase = 'align'
                self.phase_start = now
                self.align_start_time = now          # ★ 추가

        # elif self.evade_phase == 'align':
        #     if len(self.car_rear_boxes) == 2:
        #         h1 = self.car_rear_boxes[0]['h']
        #         h2 = self.car_rear_boxes[1]['h']
        #         ratio = min(h1,h2)/max(h1,h2)
        #         if ratio >= 0.5:
        #             self.evade_phase = 'lidar'
        #             self.car_evade_mode = False
        #             self.steering_command = 0
        #             self.left_speed_command = self.right_speed_command = 100
        #     self.left_speed_command = self.right_speed_command = 0
        #     # self.steering_command = -6

        elif self.evade_phase == 'align':
                # 기준 높이 설정
            HEIGHT_THRESHOLD = 30  
            if len(self.car_rear_boxes) == 2:
                box1, box2 = self.car_rear_boxes
                h1, h2 = box1['h'], box2['h']
                ratio  = min(h1, h2) / max(h1, h2)
                # 왼쪽 박스 선택
                left_box = box1 if box1['cx'] < box2['cx'] else box2
                h_now    = left_box['h']

                self.get_logger().info(f"[ALIGN] left_box height = {h_now:.1f}")

            elif len(self.car_rear_boxes) == 1:
                box1 = self.car_rear_boxes[0]
                h_now = box1['h']


            # 전진/후진 로직
            if len(self.car_rear_boxes) >= 1:
                if h_now < HEIGHT_THRESHOLD:
                    # 전진일 때 카운터 증가

                    target_slope = DMFL.calculate_slope_between_points(self.path_data[-10], self.path_data[-1])
                    self.steering_command = int(target_slope * 12 / 80)
                    if self.steering_command > 9 :
                        self.steering_command  = 9
                    elif self.steering_command < -9 :
                        self.steering_command  = -9
                    else :
                        self.steering_command = self.steering_command

                    if not hasattr(self, 'forward_count'):
                        self.forward_count = 0
                    self.forward_count += 1

                    # front 플래그 설정
                    if self.forward_count >= 3:
                        self.front = True
                        self.get_logger().info("[ALIGN] front condition activated.")
                    else:
                        self.front = False

                    # 전진 명령
                    self.left_speed_command = self.right_speed_command = 50
                else:
                    # 후진일 때 카운터와 front 상태 초기화
                    self.forward_count = 0
                    self.front = False
                    self.steering_command   = 0
                    self.left_speed_command = self.right_speed_command = -50


                if  self.lidar_data == 1 and self.front == True:
                            if self.lane_control:
                                self.phase_start = now

                            self.lane_control = False
                            self.steering_command = 7
                            self.car_evade_mode = True
                            self.left_speed_command = self.right_speed_command = 50
                            self.get_logger().info(f"Evade lane change\n" )

                            if self.car_evade_mode and (now - self.phase_start).nanoseconds / 1e9 >= 1.5:
                                self.car_evade_mode = False
                                self.phase_start = now
                                self.evade_phase = 'nothing'



        # trigger reverse when two rears first seen
        if len(self.car_rear_boxes) == 2 and not self.car_evade_mode and self.evade_phase is None:
            self.lane_control = True
            self.car_evade_mode = True
            self.evade_phase = 'reverse'
            self.phase_start = now
            self.left_speed_command = self.right_speed_command = -80
            self.steering_command = 5          #후진시 바꿔야함!!! 이상한 문제가 있음.

        # after evasion, check rear-right LiDAR to return
        if  self.lidar_data == 1 and self.evade_phase == 'lidar':
            if self.lane_control:
                self.phase_start = now

            self.lane_control = False
            self.steering_command = 7
            self.car_evade_mode = True
            self.left_speed_command = self.right_speed_command = 50
            self.get_logger().info(f"Evade lane change\n" )

            if self.car_evade_mode and (now - self.phase_start).nanoseconds / 1e9 >= 1.5:
                self.car_evade_mode = False
                self.phase_start = now
                self.evade_phase = 'nothing'


        #── 4) 정상 주행
        if not self.car_evade_mode and not self.stop_flag:    
            if self.path_data is None:
                self.steering_command = 0

            else:
                target_slope = DMFL.calculate_slope_between_points(self.path_data[-10], self.path_data[-1])
                self.steering_command = int(target_slope * 12 / 80)
                if self.steering_command > 9 :
                    self.steering_command  = 9
                elif self.steering_command < -9 :
                    self.steering_command  = -9
                else :
                    self.steering_command = self.steering_command
                if self.evade_phase == 'align':
                    self.left_speed_command = 50  # 예시 속도 값 (255가 최대 속도)
                    self.right_speed_command = 50  # 예시 속도 값 (255가 최대 속도)

                self.left_speed_command = 100  # 예시 속도 값 (255가 최대 속도)
                self.right_speed_command = 100  # 예시 속도 값 (255가 최대 속도)

        if self.evade_phase == 'align':
            if self.left_speed_command >=0:
                target_slope = DMFL.calculate_slope_between_points(self.path_data[-10], self.path_data[-1])
                self.steering_command = int(target_slope * 12 / 80)
                if self.steering_command > 9 :
                    self.steering_command  = 9
                elif self.steering_command < -9 :
                    self.steering_command  = -9
                else :
                    self.steering_command = self.steering_command

            if self.left_speed_command < 0:
                target_slope = DMFL.calculate_slope_between_points(self.path_data[-10], self.path_data[-1])
                self.steering_command = -int(target_slope * 12 / 80)
                if self.steering_command > 9 :
                    self.steering_command  = 9
                elif self.steering_command < -9 :
                    self.steering_command  = -9
                else :
                    self.steering_command = self.steering_command


        if self.stop_flag:
            self.left_speed_command = 0  # 예시 속도 값 (255가 최대 속도)
            self.right_speed_command = 0  # 예시 속도 값 (255가 최대 속도)
            self.steering_command = 0

        self.get_logger().info(f"steering: {self.steering_command}, " 
                               f"left_speed: {self.left_speed_command}, " 
                               f"right_speed: {self.right_speed_command}"
                               f"lidar_data: {self.lidar_data}")

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
        # self.get_logger().info(
        #     f"[Debug] lane_control={self.lane_control} "
        #     f"(use_lane={'lane1' if self.lane_control else 'lane2'})"
        # )

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
