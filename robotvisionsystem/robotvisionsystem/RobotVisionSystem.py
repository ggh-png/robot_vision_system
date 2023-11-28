#! /usr/bin/env python
# -*- coding:utf-8 -*-


from typing import Any
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from robotvisionsystem_msgs.msg import Motor

from robotvisionsystem.Logger import Logger
from robotvisionsystem.PIDController import PIDController
from robotvisionsystem.CurveController import CurveController
from robotvisionsystem.Sensor import Sensor

import numpy as np

import math


class RobotVisionSystem():
    '''
    Main class for AutoDriving
    '''

    def __init__(self, node: Node):
        # Ensure that the node argument is indeed an instance of rclpy.node.Node
        if not isinstance(node, Node):
            raise TypeError("Logger expects an rclpy.node.Node instance.")
        self.node = node
        self.logger = Logger(self.node)
        self.sensor = Sensor(self.node)

        # controller
        # -----------------------
        self.pid_controller = PIDController(self.node)
        self.curve_controller = CurveController(self.node)

        self.pub = self.node.create_publisher(Motor, '/car/motor', 10)
        self.control_msg = Motor()

        # detector
        # 차선 중심점

        self.ray = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]

        # 타이머 콜백 함수
        # 주기 10ms
        self.curve_cnt = 0
        # -----------------------

        # flow
        # 신호등 인식
        # 감속 및 정지선 인식
        #

        # mode controller
        self.mode = 'stopline_mode'  # 'stopline_mode'
        self.logger.info("stopline mode start")

        self.control_dict = {
            'traffic_light_mode': self.traffic_light_mode,  # 신호등 인식 모드
            'stopline_mode': self.stopline_mode,  # 정지선 인식 모드
            'curve_mode': self.curve_mode,  # 커브 모드
        }

    def poweroff(self):
        self.control_msg.motorspeed, self.control_msg.steer = 0.0, 0.0
        self.control_msg.breakbool = True
        self.pub.publish(self.control_msg)

    # pid cocntol mode
    def pid(self, slow):
        self.control_msg.steer, self.control_msg.motorspeed = self.pid_controller(
            self.sensor.centor_lane)
        self.control_msg.motorspeed *= slow
        self.control_msg.breakbool = False
        self.pub.publish(self.control_msg)

    # 신호등 인식 모드 0: 빨강, 노랑, 1: 초록

    def stopline_mode(self):
        # 신호등이 인식되지 않으면 pid 모드로 전환
        if self.sensor.traffic_light == "Unknown":
            self.logger.error("detect unknown traffic light")
            self.pid(0.5)
            return
        else:
            # 횡단 보도를 인식하면 정지 모드로 전환
            if self.sensor.stopline == True:
                self.poweroff()
                self.mode = 'traffic_light_mode'
                self.logger.warn("traffic mode start")
                return
            self.pid(1.0)
            return
    # 스탑라인 인식 모드 0: 인식안됨, 1: 인식됨

    def traffic_light_mode(self):
        if self.sensor.traffic_light == "Red" or self.sensor.traffic_light == "Yellow":
            self.poweroff()
        else:
            # 커브 모드로 전환
            self.mode = 'curve_mode'
            self.logger.error("curve mode start")


# [56.528008   7.110108   3.8785684  3.2687771  3.6757927  6.0503397
#  24.24845    5.716587   3.1184006  2.6281233  2.9553661  4.8645186]

    # ray의 0번쨰 값은 전방 1번째 값은 30 도 2번째 값은 60도 3번째 값은 90도
    # 1 ~ 6 좌측 7 ~ 12 우측
    # 오차값 중첩
    # 커브 모드 1초간 커브 주행 후 스탑라인 인식 모드로 전환


    def curve_mode(self):
        self.curve_cnt += 1
        if self.curve_cnt == 100:
            self.curve_cnt = 0
            self.mode = 'stopline_mode'
            self.logger.info("stopline mode start")
        else:
            self.control_msg.steer, self.control_msg.motorspeed = self.curve_controller(
                self.sensor.ray_msg)
            self.control_msg.breakbool = False
            self.pub.publish(self.control_msg)

    def control(self):
        '''
        main controller
        uses method based on current mode
        '''
        # print("start control")
        self.control_dict[self.mode]()  # 수정된 부분
        self.logger.info("mode: " + self.mode)
        # cv2.waitKey(1)
