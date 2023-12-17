#! /usr/bin/env python
# -*- coding:utf-8 -*-

import cv2
from rclpy.node import Node
from robotvisionsystem.Logger import Logger
from robotvisionsystem.Sensor import Sensor

import numpy as np


# colors
class PIDSpeedController():
    '''
    Detects rectangle shaped contour of given specification range
    '''

    def __init__(self):
        # if not isinstance(node, Node):
        #     raise TypeError("Logger expects an rclpy.node.Node instance.")
        # self.node = node
        # self.logger = Logger(self.node)
        # self.sensor = Sensor(self.node)

        # pid speed controller
        # self.Kp = 0.075  # 3.2
        # self.Ki = 0.0
        # self.Kd = 0.0

        self.Kp = 0.055  # 3.2
        self.Ki = 0.03
        self.Kd = 0.04

        self.previous_error = 0.0

    def __call__(self, target_speed, current_speed):

        error = target_speed - current_speed
        pwm = self.Kp * error + self.Ki * \
            (error + self.previous_error) + \
            self.Kd * (error - self.previous_error)
        self.previous_error = error
        if pwm < 0.05:
            pwm = 0.05
        elif pwm > 0.5:
            pwm = 0.5
        # self.logger.info("speed: {}, rpm: {}".format(
        #     pwm, self.sensor.current_velocity))

        return pwm
