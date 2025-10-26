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

import csv
import datetime

#---------------Variable Setting---------------
SUB_DETECTION_TOPIC_NAME = "detections"
SUB_PATH_TOPIC_NAME = "path_planning_result"
SUB_TRAFFIC_LIGHT_TOPIC_NAME = "yolov8_traffic_light_info"
SUB_LIDAR_OBSTACLE_TOPIC_NAME = "lidar_obstacle_info"
PUB_TOPIC_NAME = "topic_control_signal"

#----------------------------------------------

# 모션 플랜 발행 주기 (초) - 소수점 필요 (int형은 반영되지 않음)
TIMER = 0.1

file_name = ""

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


        ###
        self.cnt = 0
        self.csvcnt = 0

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
        self.target_slope = 0 #for log



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
        self.traffic_light_toggle_time = None  # ← 시간 기록용
        self.traffic_cnt = 0
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
        


    def timer_callback(self):
#--------------------------------------횡단보도 차선 변경 로직.
        if self.traffic_light_bbox is not None:
            self.traffic_cnt += 1
            x, y, w, h = self.traffic_light_bbox
            if h > 25 and not self.traffic_light_flag_toggled and self.traffic_cnt >= 3:
                self.lane_control = not self.lane_control
                self.traffic_light_flag_toggled = True
                self.traffic_light_toggle_time = self.get_clock().now().nanoseconds

        if self.traffic_light_toggle_time is not None:
            if self.get_clock().now().nanoseconds - self.traffic_light_toggle_time >= 20 * 1000000000:
                self.traffic_light_flag_toggled = False
                self.traffic_light_toggle_time = None  # Optional: reset the toggle time after 20 seconds
                self.traffic_cnt = 0

        RSPD = 230
        SPD = max(130, RSPD - 5*self.cnt) 
        #── 4) 정상 주행 (Stanley 적용)


        if self.lane_control is False: #2차선.
            if self.path_data is None:
                self.steering_command = 0
            else:
                target_slope = DMFL.calculate_slope_between_points(self.path_data[-10], self.path_data[-1])
                self.get_logger().info(f"target_slope: {target_slope} ")
                abs_target_slope = abs(target_slope)
                if abs_target_slope > 55: 
                    self.steering_command = int(target_slope/8)
                elif abs_target_slope >40:
                    self.steering_command = int(target_slope/18)
                elif abs_target_slope >30:
                    self.steering_command = int(target_slope/18.5)
                elif abs_target_slope >15:
                    self.steering_command = int(target_slope/19)
                else:
                    self.steering_command =  int(target_slope/21)


                if self.steering_command > 0:
                    self.steering_command =  min(9,self.steering_command)
                else:
                    self.steering_command = max(-9, self.steering_command)
                self.left_speed_command = RSPD  # 예시 속도 값 (255가 최대 속도)
                self.right_speed_command = RSPD  # 예시 속도 값 (255가 최대 속도)       

                self.target_slope = target_slope
                if self.traffic_light:
                    x, y, w, h = self.traffic_light_bbox
                    if h >= 50.0:
                        self.left_speed_command = SPD  # 예시 속도 값 (255가 최대 속도)
                        self.right_speed_command = SPD   # 예시 속도 값 (255가 최대 속도)
                        self.cnt += 1

                else:
                    self.cnt = 0
                    self.left_speed_command = RSPD  # 예시 속도 값 (255가 최대 속도)
                    self.right_speed_command = RSPD  # 예시 속도 값 (255가 최대 속도)
            
        else: #1차선.
            if self.path_data is None:
                self.steering_command = 0
            else:
                target_slope = DMFL.calculate_slope_between_points(self.path_data[-10], self.path_data[-1])
                self.get_logger().info(f"target_slope: {target_slope} ")
                abs_target_slope = abs(target_slope)
                CSPD = 160
                t = True
                if abs_target_slope > 68:
                    if target_slope > 0:
                        self.steering_command = 9
                        self.left_speed_command = CSPD
                        self.right_speed_command = CSPD
                        t = False
                    else:
                        self.steering_command = -9
                        self.left_speed_command = CSPD
                        self.right_speed_command = CSPD    
                        t = False            
                elif abs_target_slope > 48:
                    self.steering_command = int(target_slope/8.8)
                elif abs_target_slope >45:
                    self.steering_command = int(target_slope/10)
                elif abs_target_slope >40:
                    self.steering_command = int(target_slope/17)
                elif abs_target_slope >30:
                    self.steering_command = int(target_slope/17.5)
                elif abs_target_slope >20:
                    self.steering_command = int(target_slope/18)
                else:
                    self.steering_command =  int(target_slope/20)
                if t:
                    if self.steering_command > 0:
                        self.steering_command =  min(9,self.steering_command)
                    else:
                        self.steering_command = max(-9, self.steering_command)
                
                    self.left_speed_command = RSPD  # 예시 속도 값 (255가 최대 속도)
                    self.right_speed_command = RSPD  # 예시 속도 값 (255가 최대 속도)  
                self.target_slope = target_slope
                
                
                if self.traffic_light:
                    x, y, w, h = self.traffic_light_bbox
                    if h >=70.0:
                        self.left_speed_command = SPD  # 예시 속도 값 (255가 최대 속도)
                        self.right_speed_command = SPD   # 예시 속도 값 (255가 최대 속도)
                        self.cnt += 1
                
                elif abs_target_slope > 68:
                    self.left_speed_command = CSPD
                    self.right_speed_command = CSPD

                else:
                    self.cnt = 0
                    self.left_speed_command = RSPD  # 예시 속도 값 (255가 최대 속도)
                    self.right_speed_command = RSPD  # 예시 속도 값 (255가 최대 속도)
                




        self.get_logger().info(f"steering: {self.steering_command}, " 
                               f"left_speed: {self.left_speed_command}, " 
                               f"right_speed: {self.right_speed_command},"
                               f"traffic: {self.traffic_light},"
                               f"traffic_box: {self.traffic_light_bbox}")
        
        self.csvcnt += 1
        if self.csvcnt == 9:      
            self.csvcnt = 0
            my_values = ['lane1' if self.lane_control else 'lane2', int(self.target_slope), self.steering_command, self.left_speed_command, self.right_speed_command ]  

            with open(file_name, mode="a", newline='') as file:
                writer = csv.writer(file)
                writer.writerow(my_values)  # 한 줄에 5개 값 저장
            

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

    now = datetime.datetime.now()
    global file_name 
    file_name = now.strftime(f"log_{now:%H%M%S}.csv")
    # CSV 파일 처음 열 때 헤더와 함께 덮어쓰기
    with open(file_name, mode="w", newline='') as file:
        writer = csv.writer(file)
        writer.writerow(["lane", "target_slope", "steering_command", "left_speed_command","right_speed_command" ])  # 헤더 작성

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n\nshutdown\n\n")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
