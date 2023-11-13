#! /usr/bin/env python
# -*- coding:utf-8 -*-


from typing import Any
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from robotvisionsystem_msgs.msg import Motor
from robotvisionsystem_msgs.msg import State
from robotvisionsystem_msgs.msg import Ray
from robotvisionsystem_msgs.msg import Stanley

from std_msgs.msg import Float32
from std_msgs.msg import Bool
import numpy as np
import cv2
from cv_bridge import CvBridge

import math


class RobotVisionSystem(Node):
    '''
    Main class for AutoDriving
    '''

    def __init__(self, hz=10):
        super().__init__('robot_vistion_system_node')
        print('robot_vistion_system_node start')
        # controller
        # -----------------------

        # 제어 주기 설정
        self.pub = self.create_publisher(Motor, '/car/motor', 1)

        # pure pursuit config
        self.WB = 2.102  # wheel base 0.102m
        self.Lf = 8.  # look-ahead distance 0.4m
        self.diff_angle = 0
        self.control_msg = Motor()

        # -----------------------
        # pid controller
        self.Kp = 0.4
        self.Ki = 0.001
        self.Kd = 0.05
        self.previous_error = 0

        # curve controller
        self.Kp_curve = 0.05
        self.Ki_curve = 0.00
        self.Kd_curve = 0.0
        self.previous_error_curve = 0
        # detector
        # 차선 중심점
        self.create_subscription(
            Stanley, '/centor_lane', self.callback_centor_lane, 10)
        self.target_lane = 0.0
        # 신호등 인식 0: 빨강, 노랑, 1: 초록
        self.create_subscription(
            Bool, '/traffic_light', self.callback_traffic_light, 10)
        self.traffic_light = 0
        # 정지선 인식 0: 인식안됨, 1: 인식됨
        self.create_subscription(
            Bool, '/stopline', self.callback_stopline, 10)
        self.stopline = 0

        # sensor
        self.create_subscription(
            Ray, '/car/sensor/ray', self.callback_ray, 10)
        self.ray = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]

        # 타이머 콜백 함수
        # 주기 10ms
        self.timer = self.create_timer(0.01, self.control)
        self.cte = 0.0
        self.angle = 0.0

        self.curve_cnt = 0
        # -----------------------

        # flow
        # 정지선 인식 모드
        # - 인식 안되면 차선 중심점으로 주행
        # - 인식 되면 정지선까지 정지 후 신호등 인식 모드로 전환
        # 신호등 인식 모드
        # - 빨강, 노랑 : 정지
        # - 초록 : 커브 모드로 전환
        # 커브 모드
        # - 일정 시간동안 커브 주행 후 정지선 인식 모드로 전환

        # mode controller
        self.mode = 'stopline_mode'  # 'stopline_mode'

        self.control_dict = {
            'traffic_light_mode': self.traffic_light_mode,  # 신호등 인식 모드
            'stopline_mode': self.stopline_mode,  # 정지선 인식 모드
            'curve_mode': self.curve_mode,  # 커브 모드
            'lane_mode': self.pursuit  # 차선 중심점으로 주행
        }
    # self.mode_controller.set_mode('zgzg')
    # mode controller

    def set_mode(self, mode):
        self.mode = mode

    # callback function
    # -----------------------
    # 차선 중심점
    def callback_centor_lane(self, data):
        # print('callback_centor_lane')
        self.cte = data.cte
        self.angle = data.angle
        # self.target_lane = data.data

    # 신호등 인식 0: 빨강, 노랑, 1: 초록
    def callback_traffic_light(self, data):
        self.traffic_light = data.data

    # 정지선 인식 0: 인식안됨, 1: 인식됨
    def callback_stopline(self, data):
        self.stopline = data.data
    # -----------------------

    # sensor
    def callback_ray(self, data):
        self.ray = data.ray_array
        # print(self.ray)

    # lane controller

    def pursuit(self):
        self.Lf = 3.5
        # 차량의 전방 2m 지점을 look ahead point로 설정합니다.
        self.WB = 2.0
        current_angle = self.cte

        # 현재 조향각인 current_angle과 목표값(0으로 가정)을 이용하여 알고리즘을 적용합니다.
        # 다음 줄을 알고리즘에 맞게 수정해주세요.

        target_angle = 250  # -5  # 여기에 Pure Pursuit 알고리즘을 적용하여 목표 조향각을 계산합니다.

        # 현재 조향각과 목표 조향각과의 차이를 계산합니다.
        self.diff_angle = target_angle - current_angle
        # print('target_angle : ', target_angle, 'current_angle : ', current_angle, 'diff_angle : ', self.diff_angle)
        # degree to radian
        # self.diff_angle = self.diff_angle * math.pi / 180

        delta = math.atan2(2.0 * self.WB * math.sin(self.diff_angle), self.Lf)
        print('delta : ', delta)
        # radian to degree

        delta = delta * 180 / math.pi
        if delta > 2.5:
            delta = 2.50
        elif delta < -2.5:
            delta = -2.50
        self.control_msg.steer = -1 * delta
        self.control_msg.motorspeed = 0.09
        self.pub.publish(self.control_msg)
        print('steer : ', self.control_msg.steer)

    def map_value(self, value, from_min, from_max, to_min, to_max):
        return (value - from_min) * (to_max - to_min) / (from_max - from_min) + to_min

    def pid(self):
        self.Kp = 0.5
        self.Ki = 0.01
        self.Kd = 0.2

        target_angle = 250
        # Let's say `target_lane` is the current angle reading
        current_angle = self.cte
        error = target_angle - current_angle

        delta = self.Kp * error + self.Ki * \
            (error + self.previous_error) + \
            self.Kd * (error - self.previous_error)

        # 최대 조향각을 30도로 제한하고
        # 최소 조향각을 -30도로 제한합니다.
        # 조향각에 반비례하여 속도를 조절합니다.
        if delta > 30.0:
            delta = 30.0
        elif delta < -30.0:
            delta = -30.0
        delta = self.map_value(delta, -30, 30, -5.0, 5.0)

        speed = min((0.12 - (abs(delta) / 50.0)), 0.088)

        self.previous_error = error
        self.control_msg.steer = -1 * delta
        self.control_msg.motorspeed = speed
        # self.control_msg.breakbool = False
        print('steer : ', self.control_msg.steer,
              'speed : ', self.control_msg.motorspeed)
        self.pub.publish(self.control_msg)
        # self.control_msg.motorspeed = 0.08
        # self.pub.publish(self.control_msg)

    def stanley(self):
        yaw_term = self.angle
        target = self.cte
        cte = (target) * 1.9/500.0
        stanley_k = 0.5
        cte_term = np.arctan2(stanley_k*cte, 30.0)
        self.control_msg.steer = np.degrees(
            0.4 * yaw_term + cte_term) * 5.0 / 3.0 + 2
        self.control_msg.motorspeed = 0.1
        self.control_msg.breakbool = False
        self.pub.publish(self.control_msg)
        # print('steer : ', self.control_msg.steer, 'cte : ', target)

    def poweroff(self):
        self.control_msg.motorspeed, self.control_msg.steer = 0.0, 0.0
        self.control_msg.breakbool = True
        self.pub.publish(self.control_msg)

    # 스탑라인 인식 모드 0: 인식안됨, 1: 인식됨
    def stopline_mode(self):
        if self.stopline == False:
            # self.pursuit()
            self.pid()
            # self.stanley()
        else:
            self.poweroff()
            self.set_mode('traffic_light_mode')

    # 신호등 인식 모드 0: 빨강, 노랑, 1: 초록
    def traffic_light_mode(self):
        if self.traffic_light == 0:
            self.poweroff()
        else:
            self.set_mode('curve_mode')

# [56.528008   7.110108   3.8785684  3.2687771  3.6757927  6.0503397
#  24.24845    5.716587   3.1184006  2.6281233  2.9553661  4.8645186]

    # ray의 0번쨰 값은 전방 1번째 값은 30 도 2번째 값은 60도 3번째 값은 90도
    # 1 ~ 6 좌측 7 ~ 12 우측
    # 오차값 중첩
    # 커브 모드 1초간 커브 주행 후 스탑라인 인식 모드로 전환
    def curve_mode(self):
        self.curve_cnt += 1
        if self.curve_cnt == 1000:
            self.curve_cnt = 0
            self.set_mode('stopline_mode')
        else:
            error = self.ray[1] + self.ray[2] + self.ray[3] + self.ray[4] + \
                self.ray[5] - self.ray[7] - self.ray[8] - \
                self.ray[9] - self.ray[10] - self.ray[11]
            delta = self.Kp_curve * error + self.Ki_curve * \
                (error + self.previous_error_curve) + \
                self.Kd_curve * (error - self.previous_error_curve)
            self.previous_error_curve = error
            self.control_msg.steer = 0.0
            self.control_msg.motorspeed = 0.1
            self.pub.publish(self.control_msg)
            print('steer : ', self.control_msg.steer)
            # self.set_mode('stopline_mode')

    def control(self):
        '''
        main controller
        uses method based on current mode
        '''
        # print("start control")
        self.control_dict[self.mode]()  # 수정된 부분
        # print('mode : ', self.mode)
        # cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    robotvisionsystem = RobotVisionSystem()

    while rclpy.ok():
        # robotvisionsystem.control()  # 수정된 부분
        rclpy.spin_once(robotvisionsystem)  # 수정된 부분
    rclpy.shutdown()


if __name__ == '__main__':
    main()
