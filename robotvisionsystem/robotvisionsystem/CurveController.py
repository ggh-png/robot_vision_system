#! /usr/bin/env python
# -*- coding:utf-8 -*-

import cv2
from rclpy.node import Node
from robotvisionsystem.Logger import Logger
import numpy as np


# colors
class CurveController():
    '''
    Detects rectangle shaped contour of given specification range
    '''

    def __init__(self, node: Node):
        if not isinstance(node, Node):
            raise TypeError("Logger expects an rclpy.node.Node instance.")
        self.node = node
        self.logger = Logger(self.node)
        # pid controller
        self.Kp = 0.5
        self.Ki = 0.0
        self.Kd = 0.0

        self.previous_error = 0.0
    # lane controller

    def map_value(self, value, from_min, from_max, to_min, to_max):
        return (value - from_min) * (to_max - to_min) / (from_max - from_min) + to_min

    def __call__(self, ray):
        for i in range(12):
            if ray[i] > 20.0:
                ray[i] = 20.0

        error = ray[1] + ray[2] + ray[3] - \
            ray[9] - ray[10] - ray[11]
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
        delta = self.map_value(delta, -30.0, 30.0, -5.0, 5.0)

        speed = min((0.12 - (abs(delta) / 90.0)), 0.1)
        return -delta, speed
        # self.set_mode('stopline_mode')
