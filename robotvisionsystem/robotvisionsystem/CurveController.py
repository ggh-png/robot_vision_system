#! /usr/bin/env python
# -*- coding:utf-8 -*-

import cv2
from rclpy.node import Node
from robotvisionsystem.Logger import Logger
from robotvisionsystem.Sensor import Sensor
from robotvisionsystem.PIDSpeedController import PIDSpeedController
import numpy as np


# colors
class CurveController():
    '''
    Detects rectangle shaped contour of given specification range
    '''

    def __init__(self):
        # if not isinstance(node, Node):
        #     raise TypeError("Logger expects an rclpy.node.Node instance.")
        # self.node = node
        # self.logger = Logger(self.node)
        # self.sensor = Sensor(self.node)
        self.pid_speed_controller = PIDSpeedController()

        self.min_speed = 4.0
        self.max_speed = 2.0

        # pid controller
        self.Kp_z = 0.75
        self.Ki_z = 0.0
        self.Kd_z = 0.0

        self.Kp_x = 0.2
        self.Ki_x = 0.0
        self.Kd_x = 0.0

        self.previous_error_z = 0.0
        self.previous_error_x = 0.0
    # lane controller

    def map_value(self, value, from_min, from_max, to_min, to_max):
        return (value - from_min) * (to_max - to_min) / (from_max - from_min) + to_min

    def __call__(self, target_min_z, target_max_z, target_min_x, target_max_x, pos_z, pos_x, current_speed):
        # Let's say `target_lane` is the current angle reading
        # 현재 위치에서 y축으로 20만큼
        max_error_z = target_max_z - pos_z
        min_error_z = target_min_z - pos_z
        # 현재 위치에서 x축으로 20만큼
        max_error_x = target_max_x - pos_x
        min_error_x = target_min_x - pos_x

        error_z = min(max_error_z, min_error_z)
        error_x = min(max_error_x, min_error_x)

        delta = self.Kp_z * error_z + self.Ki_z * \
            (error_z + self.previous_error_z) + \
            self.Kd_z * (error_z - self.previous_error_z)

        self.previous_error_z = error_z
        # 최대 조향각을 30도로 제한하고
        # 최소 조향각을 -30도로 제한합니다.
        # 조향각에 반비례하여 속도를 조절합니다.
        if delta > 30.0:
            delta = 30.0
        elif delta < -30.0:
            delta = -30.0

        speed = self.Kp_x * error_x + self.Ki_x * \
            (error_x + self.previous_error_x) + \
            self.Kd_x * (error_x - self.previous_error_x)
        self.previous_error_x = error_x
        if speed > 5.0:
            speed = 5.0
        elif speed < 2.0:
            speed = 2.0

        speed = self.pid_speed_controller(abs(speed), current_speed)
        # print("speed: ", speed)

        return -delta, speed, error_z, error_x
