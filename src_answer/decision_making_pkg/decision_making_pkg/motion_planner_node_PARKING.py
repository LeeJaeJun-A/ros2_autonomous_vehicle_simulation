import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSReliabilityPolicy
from sensor_msgs.msg import LaserScan
from interfaces_pkg.msg import MotionCommand
from std_msgs.msg import Float32
import numpy as np
from sklearn.cluster import DBSCAN

from std_msgs.msg import Float32, Bool
#--------------- Node Specific Parameters ---------------
SUB_LIDAR_TOPIC_NAME = "scan"              # 구독할 라이다 토픽 이름
PUB_TOPIC_NAME = "topic_control_signal"    # 발행할 제어 신호 토픽 이름
TIMER = 0.1                                # 타이머 주기 (초)
STEERING_FACTOR = 9                        # 조향 명령 스케일 팩터 (조정 필요)

#--------------- Initial Motion Parameters ---------------
FORWARD_SPEED_INIT = 100                  # 초기 직진 속도
REVERSE_SPEED = -80                   # 후진 속도
STOP_SPEED = 0                             # 정지 속도

#--------------- Turning Parameters ---------------
LEFT_TURN_DURATION = 4.4                # 좌회전 지속 시간 (초)
RIGHT_TURN_DURATION = 5.0              # 우회전 지속 시간 (초)
TURN_STEERING = -9                         # 회전 조향 값 (좌회전)
RIGHT_TURN_STEERING = 9                    # 우회전 조향 값
TURN_SPEED = 200                            # 회전 속도

#--------------- Rear Lidar Processing Parameters ---------------
REAL_REAR_ANGLE = 180.0                    # 후방 기준 각도 (degree)
REAR_LIDAR_ANGLE_MIN_DEG = REAL_REAR_ANGLE - 80.0 # 후방 라이다 감지 최소 각도
REAR_LIDAR_ANGLE_MAX_DEG = REAL_REAR_ANGLE + 80.0 # 후방 라이다 감지 최대 각도
REAR_ANGLE_TOLERANCE_DEG = 10.5           # 후방 물체로 간주할 각도 범위 오차

#--------------- Post-Parking Maneuver Parameters ---------------
PARKED_DURATION = 3.0                      # 주차 완료 후 대기 시간 (초)
FORWARD_SPEED_AFTER_PARK = 200             # 정지 후 직진 속도 
FORWARD_DURATION_AFTER_RIGHT = 5.0         # 우회전 후 직진 시간 (초)
#----------------------------------------------

class MotionPlanningNode(Node):
    def __init__(self):
        super().__init__('motion_planner_node')

        # 토픽 이름 설정
        self.sub_lidar_topic = self.declare_parameter('sub_lidar_topic', SUB_LIDAR_TOPIC_NAME).value
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
        self.lidar_data = None
        self.right_obstacle_detected = False
        self.rear_obstacle_detected = False
        self.is_turning_left = False
        self.left_turn_start_time = None
        self.parking_state = 'initial_forward' # 초기 상태: 직진
        self.received_start_angles = []
        self.received_end_angles = []
        self.steering_command = 0.0
        self.left_speed_command = 0.0
        self.right_speed_command = 0.0
        self.rear_obstacle_detected_once = False # 후방 물체가 한 번이라도 감지되었는지 확인
        self.parked_start_time = None
        self.forward_after_park_start_time = None
        self.right_turn_after_forward_start_time = None
        self.initial_forward_start_time = None

        # 서브스크라이버 설정
        
        self.lidar_sub = self.create_subscription(LaserScan, self.sub_lidar_topic, self.lidar_callback, self.qos_profile)
        self.start_angle_sub = self.create_subscription(
            Float32,
            'obstacle_start_angle',
            self.start_angle_callback,
            self.qos_profile)
        self.end_angle_sub = self.create_subscription(
            Float32,
            'obstacle_end_angle',
            self.end_angle_callback,
            self.qos_profile)
        
        self.obstacle_info_sub = self.create_subscription(  # [수정됨] 장애물 유무 구독
            Bool,
            'lidar_obstacle_info',
            self.obstacle_info_callback,
            self.qos_profile
        )
    
        # 퍼블리셔 설정
        self.publisher = self.create_publisher(MotionCommand, self.pub_topic, self.qos_profile)
        self.initial_forward_start_time = self.get_clock().now().nanoseconds / 1e9
        # 타이머 설정
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

    def start_angle_callback(self, msg: Float32):
        angle = msg.data
        if self.parking_state == 'reversing' and REAR_LIDAR_ANGLE_MIN_DEG <= angle <= REAR_LIDAR_ANGLE_MAX_DEG:
            self.received_start_angles.append(angle)
            self.get_logger().info(f"Received start angle (filtered): {angle:.2f} deg")
        else:
            self.get_logger().debug(f"Ignored start angle: {angle:.2f} deg (out of rear range)")

    def end_angle_callback(self, msg: Float32):
        angle = msg.data
        if self.parking_state == 'reversing' and REAR_LIDAR_ANGLE_MIN_DEG <= angle <= REAR_LIDAR_ANGLE_MAX_DEG:
            self.received_end_angles.append(angle)
            self.get_logger().info(f"Received end angle (filtered): {angle:.2f} deg")
        else:
            self.get_logger().debug(f"Ignored end angle: {angle:.2f} deg (out of rear range)")

    def obstacle_info_callback(self, msg: Bool):
        self.right_obstacle_detected = msg.data

    def lidar_callback(self, msg: LaserScan):
        self.lidar_data = msg

    def timer_callback(self):
        now = self.get_clock().now().nanoseconds / 1e9

        #1 초기직진
        if self.parking_state == 'initial_forward':
            if self.initial_forward_start_time is None:
                self.initial_forward_start_time = now  # 시작 시간 기록

            self.steering_command = STOP_SPEED
            self.left_speed_command = FORWARD_SPEED_INIT
            self.right_speed_command = FORWARD_SPEED_INIT

            # 10초 이상 지난 후에만 장애물 감지 반응
            if (now - self.initial_forward_start_time) >= 5.0:
                if self.right_obstacle_detected:
                    self.get_logger().info("Right obstacle detected. Preparing to turn left.")
                    self.parking_state = 'turning_left'
                    self.left_turn_start_time = now

        #2 오른쪽 물체 발견 시 좌회전
    
        elif self.parking_state == 'turning_left':
            if now - self.left_turn_start_time >= LEFT_TURN_DURATION: # 4.5초동안 좌회전하고 끝나면 후진
                self.get_logger().info("Left turn completed. Starting to reverse straight.");
                self.parking_state = 'reversing' 
                # self.parking_state = 'adjusting'
                self.adjusting_start_time = now 
                self.reversing_start_time = now
                self.steering_command = 0.0 # 좌회전 완료 후 직진 후진
                self.received_start_angles = [] # 초기화
                self.received_end_angles = []   # 초기화
            else:
                self.steering_command = TURN_STEERING
                self.left_speed_command = TURN_SPEED/3
                self.right_speed_command = TURN_SPEED*0.85
                

        #3 후진 및 조향 각도 계산 (후방 물체 감지 후)
        elif self.parking_state == 'reversing':
            self.left_speed_command = REVERSE_SPEED
            self.right_speed_command = REVERSE_SPEED
            # if self.rear_obstacle_detected_once and self.received_start_angles and self.received_end_angles:
            # if self.received_start_angles and self.received_end_angles:
            if len(self.received_start_angles) >=2 and len(self.received_end_angles)>=2:
                max_end_angle = np.max(self.received_end_angles)
                min_start_angle = np.min(self.received_start_angles)
                # min_end_angle = np.min(self.received_end_angles)
                # max_start_angle = np.max(self.received_start_angles)

                steering_angle_deg = (max_end_angle + min_start_angle) / 2.0 
                if steering_angle_deg > 183.0:
                    self.steering_command = 1.0 # 우회전 (공간이 오른쪽에 많음)
                    self.get_logger().info(f"Reversing: Steering right (angle > 180.0: {steering_angle_deg:.2f})")
                elif steering_angle_deg < 177.0:
                    self.steering_command = -1.0 # 좌회전 (공간이 왼쪽에 많음)
                    self.get_logger().info(f"Reversing: Steering left (angle < 180.0: {steering_angle_deg:.2f})")
                else:
                    self.steering_command = 0.0 # 180도면 직진
                    self.get_logger().info(f"Reversing: Steering straight (angle == 180.0: {steering_angle_deg:.2f})")
            else:
                self.steering_command = 0.0 # 초기 후진은 직진
                self.get_logger().info("Reversing straight initially.")

            # 3초동안 조향하며 후진하다가 정지
            if now - self.reversing_start_time > 12.0: #왼쪽점 11초
                self.parking_state = 'parked'
                self.left_speed_command = STOP_SPEED
                self.right_speed_command = STOP_SPEED
                self.steering_command = STOP_SPEED
                self.parked_start_time = now
                self.get_logger().info("Parking completed. Waiting...")

        #4 주차 완료 후 3초 대기
        elif self.parking_state == 'parked':
            self.left_speed_command = STOP_SPEED
            self.right_speed_command = STOP_SPEED
            self.steering_command = STOP_SPEED
            if self.parked_start_time is not None and now - self.parked_start_time >= PARKED_DURATION:
                self.get_logger().info("Waiting complete. Moving forward after park.")
                self.parking_state = 'forward_after_park'
                self.forward_after_park_start_time = now

        #5 정지 후 직진
        elif self.parking_state == 'forward_after_park':
            self.steering_command = STOP_SPEED
            self.left_speed_command = FORWARD_SPEED_AFTER_PARK
            self.right_speed_command = FORWARD_SPEED_AFTER_PARK
            if self.forward_after_park_start_time is not None and now - self.forward_after_park_start_time >= 1.0: # 1초였음
                self.get_logger().info("Moving forward complete. Turning right.")
                self.parking_state = 'turn_right_after_forward'
                self.right_turn_after_forward_start_time = now

        #6 직진 후 우회전
        elif self.parking_state == 'turn_right_after_forward':
            self.steering_command = RIGHT_TURN_STEERING
            self.left_speed_command = TURN_SPEED
            self.right_speed_command = TURN_SPEED/3
            if self.right_turn_after_forward_start_time is not None and now - self.right_turn_after_forward_start_time >= RIGHT_TURN_DURATION:
                self.get_logger().info("Right turn complete. Moving forward.")
                self.parking_state = 'forward_after_right_turn'
                self.forward_after_right_turn_start_time = now

        #7 우회전 후 직진
        elif self.parking_state == 'forward_after_right_turn':
            self.steering_command = STOP_SPEED
            self.left_speed_command = FORWARD_SPEED_AFTER_PARK
            self.right_speed_command = FORWARD_SPEED_AFTER_PARK
            # 이후 필요한 동작 추가 가능, 현재는 계속 직진

        motion_command_msg = MotionCommand()
        motion_command_msg.steering = int(self.steering_command * STEERING_FACTOR) # 조향 값 스케일링 (조정 필요)
        motion_command_msg.left_speed = int(self.left_speed_command)
        motion_command_msg.right_speed = int(self.right_speed_command)
        self.publisher.publish(motion_command_msg)

def main(args=None):
    rclpy.init(args=args)
    node = MotionPlanningNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n\nshutdown\n\n")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    main()