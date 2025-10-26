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
            if (h >= 68 and self.traffic_light_data is not None and self.traffic_light_data.data == 'Red'):
                self.steering_command = 0 
                self.left_speed_command = 0 
                self.right_speed_command = 0
                self.stop_flag = True

            if (h >= 45 and self.traffic_light_data is not None and self.traffic_light_data.data == 'Green'):
                self.stop_flag = False




        # if self.traffic_light and self.detection_data is not None:
        #     for detection in self.detection_data.detections:
        #         if detection.class_name == 'traffic_light':
        #             y_max = int(detection.bbox.center.position.y + detection.bbox.size.y / 2)
        #             if y_max < 150:
        #                 self.stop_flag = True
        # else:
        #     self.stop_flag = False


            # x, y, w, h = self.traffic_light_bbox
            # if (h >= 45 and self.traffic_light_data is not None and self.traffic_light_data.data == 'Red'):
            #     self.steering_command = 0 
            #     self.left_speed_command = 0 
            #     self.right_speed_command = 0
            #     self.stop_flag = True

            # if (h >= 45 and self.traffic_light_data is not None and self.traffic_light_data.data == 'green'):
            #     self.stop_flag = False

        if self.evade_phase == 'reverse':
            if (now - self.phase_start).nanoseconds / 1e9 >= 2:
                self.evade_phase = 'stop'
                self.phase_start = now
                self.left_speed_command = self.right_speed_command = 80
                self.steering_command = 5

        elif self.evade_phase == 'stop':
            self.left_speed_command = self.right_speed_command = 100
            self.steering_command = -5
            if (now - self.phase_start).nanoseconds / 1e9 >= 4.0:
                self.evade_phase = 'align'
                self.phase_start = now
                self.align_start_time = now          # ★ 추가


        elif self.evade_phase == 'align':
            HEIGHT_THRESHOLD = 60

            target_slope = DMFL.calculate_slope_between_points(self.path_data[-10], self.path_data[-1])
            steer_val = int(target_slope * 12 / 80)
            self.steering_command = max(-9, min(9, steer_val))
            self.left_speed_command = self.right_speed_command = -50

            if len(self.car_rear_boxes) == 2:
                box1, box2 = self.car_rear_boxes
                # h1, h2 = box1['h'], box2['h']
                # ratio = min(h1, h2) / max(h1, h2)
                left_box = box1 if box1['cx'] < box2['cx'] else box2
                h_now = left_box['h']

            elif len(self.car_rear_boxes) == 1:
                h_now = self.car_rear_boxes[0]['h']

            else:
                self.get_logger().warn("[ALIGN] No car boxes detected.")
                return  # 아무것도 못하면 align 중단

            # 경로 정보 부족 시 align 중단
            if not self.path_data or len(self.path_data) < 10:
                self.get_logger().warn("[ALIGN] Not enough path points.")
                return

            # 전진/후진 판단
            if h_now <= HEIGHT_THRESHOLD:
                target_slope = DMFL.calculate_slope_between_points(self.path_data[-10], self.path_data[-1])
                steer_val = int(target_slope * 12 / 80)
                self.steering_command = max(-9, min(9, steer_val))

                if not hasattr(self, 'forward_count'):
                    self.forward_count = 0
                self.forward_count += 1

                if self.forward_count >= 3:
                    self.front = True
                    self.get_logger().info("[ALIGN] Front condition activated.")
                else:
                    self.front = False

                self.left_speed_command = self.right_speed_command = 50

            elif h_now > HEIGHT_THRESHOLD:
                self.forward_count = 0
                self.front = False
                self.steering_command = 0
                self.left_speed_command = self.right_speed_command = -50


            if  self.lidar_data == 1 and self.front == True:
                if self.lane_control:
                    self.phase_start = now

                self.lane_control = False
                self.steering_command = 9
                self.car_evade_mode = True
                self.left_speed_command = 200
                self.right_speed_command = 100
                self.evade_phase = 'FINAL'

                self.get_logger().info(f"Evade lane change\n" )

        elif self.evade_phase == 'FINAL':
            if self.car_evade_mode and (now - self.phase_start).nanoseconds / 1e9 >= 2.5:
                self.car_evade_mode = False
                self.phase_start = now
                self.evade_phase = 'nothing'





        # trigger reverse when two rears first seen
        if len(self.car_rear_boxes) == 2 and not self.car_evade_mode and self.evade_phase == None:
            self.lane_control = True
            self.car_evade_mode = True
            self.evade_phase = 'align'
            self.phase_start = now
            self.left_speed_command = self.right_speed_command = -50
            self.steering_command = 5          #후진시 바꿔야함!!! 이상한 문제가 있음.

            
            


#── 4) 정상 주행
        elif not self.car_evade_mode and not self.stop_flag:    
            if self.path_data == None:
                self.steering_command = 0

            else:
                target_slope = DMFL.calculate_slope_between_points(self.path_data[-10], self.path_data[-1])
 
                self.steering_command = int(target_slope * 7 / 100)

                if target_slope<-70:
                    self.steering_command = -9
                    self.left_speed_command = 70
                    self.right_speed_command = 120

                else:
                    self.left_speed_command = 100
                    self.left_speed_command = 100

                if self.steering_command > 9 :
                    self.steering_command  = 9
                elif self.steering_command < -9 :
                    self.steering_command  = -9
                else :
                    self.steering_command = self.steering_command
                    
                self.left_speed_command = self.right_speed_command = 100

        if self.evade_phase == 'nothing':
            target_slope = DMFL.calculate_slope_between_points(self.path_data[-10], self.path_data[-1])
            self.steering_command = int(target_slope * 7 / 100)

            if target_slope<-70:
                self.steering_command = -9
                self.left_speed_command = 70
                self.right_speed_command = 120

            else:
                self.left_speed_command = 100
                self.left_speed_command = 100


        if self.stop_flag:
            self.left_speed_command = 0  # 예시 속도 값 (255가 최대 속도)
            self.right_speed_command = 0  # 예시 속도 값 (255가 최대 속도)
            self.steering_command = 0

        self.get_logger().info(f"steering: {self.steering_command}\n, " 
                               f"left_speed: {self.left_speed_command}\n, " 
                               f"right_speed: {self.right_speed_command}\n,"
                               f"lidar_data: {self.lidar_data}\n,"
                               f"current_phase {self.evade_phase}\n,"
                               f"fronting {self.front}\n,"
                               f"stop_flag: {self.stop_flag}\n")

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
