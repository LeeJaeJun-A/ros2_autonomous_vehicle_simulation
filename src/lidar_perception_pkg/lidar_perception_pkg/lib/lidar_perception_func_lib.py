"""
LiDAR Perception Function Library
주차 및 장애물 감지를 위한 헬퍼 함수들
"""

import numpy as np
from sensor_msgs.msg import LaserScan


class StabilityDetector:
    """
    연속적인 감지를 통해 안정적인 장애물 감지를 수행하는 클래스
    일시적인 노이즈를 필터링하고 안정적인 감지만 True로 반환
    """
    def __init__(self, consec_count=3):
        """
        Args:
            consec_count (int): True로 판단하기 위해 필요한 연속 감지 횟수
        """
        self.consec_count = consec_count
        self.counter = 0

    def check_consecutive_detections(self, detected):
        """
        현재 감지 상태를 업데이트하고 안정적인 감지 여부를 반환

        Args:
            detected (bool): 현재 프레임에서 장애물이 감지되었는지 여부

        Returns:
            bool: 연속으로 consec_count번 감지되었는지 여부
        """
        if detected:
            self.counter += 1
        else:
            self.counter = 0

        return self.counter >= self.consec_count


def rotate_lidar_data(msg: LaserScan, offset: int = 0):
    """
    LiDAR 데이터를 회전시킵니다.

    Args:
        msg (LaserScan): 입력 LaserScan 메시지
        offset (int): 회전할 각도 (0~359)

    Returns:
        LaserScan: 회전된 LaserScan 메시지
    """
    if offset == 0:
        return msg

    ranges = np.array(msg.ranges)
    intensities = np.array(msg.intensities)

    # offset만큼 데이터 회전
    rotated_ranges = np.roll(ranges, offset)
    rotated_intensities = np.roll(intensities, offset)

    msg.ranges = rotated_ranges.tolist()
    msg.intensities = rotated_intensities.tolist()

    return msg


def flip_lidar_data(msg: LaserScan, pivot_angle: int = 0):
    """
    LiDAR 데이터를 특정 각도를 기준으로 좌우 반전시킵니다.

    Args:
        msg (LaserScan): 입력 LaserScan 메시지
        pivot_angle (int): 반전 기준 각도 (0~359)

    Returns:
        LaserScan: 반전된 LaserScan 메시지
    """
    ranges = np.array(msg.ranges)
    intensities = np.array(msg.intensities)

    # pivot_angle을 기준으로 좌우 반전
    flipped_ranges = np.concatenate([
        ranges[pivot_angle:],
        ranges[:pivot_angle]
    ])[::-1]

    flipped_intensities = np.concatenate([
        intensities[pivot_angle:],
        intensities[:pivot_angle]
    ])[::-1]

    msg.ranges = flipped_ranges.tolist()
    msg.intensities = flipped_intensities.tolist()

    return msg


def calculate_obstacle_center_angle(start_angle: float, end_angle: float):
    """
    장애물의 시작 각도와 끝 각도로부터 중심 각도를 계산합니다.

    Args:
        start_angle (float): 장애물 시작 각도 (도)
        end_angle (float): 장애물 끝 각도 (도)

    Returns:
        float: 중심 각도 (도)
    """
    return (start_angle + end_angle) / 2.0


def normalize_angle(angle: float):
    """
    각도를 0~360도 범위로 정규화합니다.

    Args:
        angle (float): 입력 각도

    Returns:
        float: 정규화된 각도 (0~360)
    """
    while angle < 0:
        angle += 360
    while angle >= 360:
        angle -= 360
    return angle

