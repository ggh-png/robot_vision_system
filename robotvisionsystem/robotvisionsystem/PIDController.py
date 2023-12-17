#! /usr/bin/env python
# -*- coding:utf-8 -*-

import cv2
from rclpy.node import Node
from robotvisionsystem.Logger import Logger
from robotvisionsystem.Sensor import Sensor
from robotvisionsystem.PIDSpeedController import PIDSpeedController
import numpy as np


# colors
class PIDController():
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
        # pid angle controller
        self.Kp = 0.5
        self.Ki = 0.0
        self.Kd = 0.2

        self.max_speed = 5.0
        self.min_speed = 3.0

        self.previous_error = 0.0

    # lane controller
    def map_value(self, value, from_min, from_max, to_min, to_max):
        return (value - from_min) * (to_max - to_min) / (from_max - from_min) + to_min

    def __call__(self, cte, current_speed):
        target_angle = 250
        # Let's say `target_lane` is the current angle reading
        current_angle = cte
        error = target_angle - current_angle

        # pid angle controller
        delta = self.Kp * error + self.Ki * \
            (error + self.previous_error) + \
            self.Kd * (error - self.previous_error)
        self.previous_error = error

        # 최대 조향각을 30도로 제한하고
        # 최소 조향각을 -30도로 제한합니다.
        # 조향각에 반비례하여 속도를 조절합니다.
        if delta > 30.0:
            delta = 30.0
        elif delta < -30.0:
            delta = -30.0

        # pid speed controller
        delta = self.map_value(delta, -30.0, 30.0, -5.0, 5.0)

        target_speed = 10.0
        speed = self.pid_speed_controller(target_speed, current_speed)

        return -delta, speed
