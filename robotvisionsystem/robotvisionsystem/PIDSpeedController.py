#! /usr/bin/env python
# -*- coding:utf-8 -*-

# colors
class PIDSpeedController():
    '''
    Detects rectangle shaped contour of given specification range
    '''

    def __init__(self):

        self.Kp = 0.055  # 3.2
        self.Ki = 0.0
        self.Kd = 0.02

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

        # # print("speed: {}, rpm: {}".format(
        #     pwm, current_speed))

        return pwm
