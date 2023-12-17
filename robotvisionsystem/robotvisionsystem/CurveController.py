#! /usr/bin/env python
# -*- coding:utf-8 -*-

from robotvisionsystem.PIDSpeedController import PIDSpeedController


# colors
class CurveController():
    '''
    Detects rectangle shaped contour of given specification range
    '''

    def __init__(self):

        self.pid_speed_controller = PIDSpeedController()

    def map_value(self, value, from_min, from_max, to_min, to_max):
        return (value - from_min) * (to_max - to_min) / (from_max - from_min) + to_min

    def __call__(self, target_min_yaw, target_max_yaw, current_yaw, current_speed, delta):
        # Let's say `target_lane` is the current angle reading
        # 현재 위치에서 y축으로 20만큼
        max_error = target_max_yaw - current_yaw
        min_error = target_min_yaw - current_yaw

        error = min(abs(max_error), abs(min_error))

        speed = self.pid_speed_controller(3, current_speed)
        # print("speed: ", speed)

        return -delta, speed, error
