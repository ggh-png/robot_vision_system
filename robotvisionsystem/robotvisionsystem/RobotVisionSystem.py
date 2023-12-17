#! /usr/bin/env python
# -*- coding:utf-8 -*-


from typing import Any
from rclpy.node import Node

from robotvisionsystem_msgs.msg import Motor

from robotvisionsystem.Logger import Logger
from robotvisionsystem.PIDController import PIDController
from robotvisionsystem.PIDSpeedController import PIDSpeedController
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
        self.pid_controller = PIDController()
        self.curve_controller = CurveController()
        self.pid_speed_controller = PIDSpeedController()

        self.pub = self.node.create_publisher(Motor, '/car/motor', 10)
        self.control_msg = Motor()

        # 타이머 콜백 함수
        # 주기 10ms
        self.curve_cnt = 0
        self.curve_mode_cnt = 0
        self.start_cnt = 0

        # -----------------------

        # flow
        # 신호등 인식
        # 감속 및 정지선 인식
        #

        # curve control
        self.target_min_yaw = 0.0
        self.target_max_yaw = 0.0
        self.target_delta = 0.0
        self.curve_flag = False

        # mode controller
        self.mode = 'start_mode'  # 'stopline_mode'
        # self.mode = 'stopline_mode'
        self.logger.info("stopline mode start")

        self.control_dict = {
            'start_mode': self.start_mode,
            'traffic_light_mode': self.traffic_light_mode,  # 신호등 인식 모드
            'stopline_mode': self.stopline_mode,  # 정지선 인식 모드
            'curve_mode': self.curve_mode,  # 커브 모드
            'time_mode': self.time_mode,  # 시간 모드
        }

    def poweroff(self):
        self.control_msg.motorspeed, self.control_msg.steer = 0.0, 0.0
        self.control_msg.breakbool = True
        self.pub.publish(self.control_msg)

    # pid cocntol mode
    def pid(self, slow):
        self.control_msg.steer, self.control_msg.motorspeed = self.pid_controller(
            self.sensor.centor_lane, self.sensor.current_velocity)
        self.control_msg.motorspeed *= slow
        self.control_msg.breakbool = False
        self.pub.publish(self.control_msg)

    # 신호등 인식 모드 0: 빨강, 노랑, 1: 초록

    def start_mode(self):
        self.control_msg.motorspeed = self.pid_speed_controller(
            3, self.sensor.current_velocity)
        self.control_msg.steer = 0.0
        self.control_msg.breakbool = False
        self.pub.publish(self.control_msg)

        self.start_cnt += 1
        if self.start_cnt > 70:
            self.mode = 'stopline_mode'
            self.start_cnt = 0
            self.logger.info("stopline mode start")

    def stopline_mode(self):
        # 신호등이 인식되지 않으면 pid 모드로 전환
        if self.sensor.traffic_light != "Detected":
            if self.sensor.stopline == True:
                self.poweroff()
                self.mode = 'traffic_light_mode'
                self.logger.warn("traffic mode start")
                return
            if self.sensor.traffic_light == "Red":
                self.pid(0.5)
            elif self.sensor.traffic_light == "Yellow":
                self.pid(0.5)
            elif self.sensor.traffic_light == "Green":
                self.pid(0.5)
            if self.curve_mode_cnt == 3:
                self.pid(0.25)
            elif self.curve_mode_cnt == 2:
                self.pid(0.15)

            return
        else:
            if self.curve_mode_cnt == 2:
                self.pid(0.25)
            elif self.curve_mode_cnt == 3:
                self.pid(0.15)
            else:
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

    def curve_mode(self):

        if self.curve_flag == False:
            self.target_min_yaw = self.sensor.yaw - 80
            self.target_max_yaw = self.sensor.yaw + 80
            self.curve_mode_cnt += 1
            self.delta = -10.0
            if self.curve_mode_cnt == 2:
                self.target_max_yaw = self.sensor.yaw + 70
                self.target_min_yaw = self.sensor.yaw - 70
                self.delta = -10.5
            elif self.curve_mode_cnt == 3:
                self.target_max_yaw = self.sensor.yaw + 70
                self.target_min_yaw = self.sensor.yaw - 70
                self.delta = -9.5
            if self.curve_mode_cnt == 4:
                self.target_max_yaw = self.sensor.yaw + 95
                self.target_min_yaw = self.sensor.yaw - 95
                self.delta = -9.5

                self.curve_mode_cnt = 0

            self.curve_flag = True
        else:
            self.control_msg.steer, self.control_msg.motorspeed, error = self.curve_controller(
                self.target_min_yaw, self.target_max_yaw, self.sensor.yaw, self.sensor.current_velocity, self.delta)
            # self.control_msg.motorspeed = self.pid_speed_controller(3)
            self.control_msg.steer *= -1.0
            self.control_msg.breakbool = False
            self.pub.publish(self.control_msg)
            # self.logger.warn(str(error))

            if abs(error) < 8.5:
                self.mode = 'time_mode'
                self.curve_flag = False
                self.logger.info("time mode start")

    def time_mode(self):
        self.curve_cnt += 1
        if self.curve_cnt > 10:
            self.mode = 'stopline_mode'
            self.curve_cnt = 0
            self.logger.info("stopline mode start")
        # self.poweroff()

    def control(self):
        '''
        main controller
        uses method based on current mode
        '''
        # print("start control")
        self.control_dict[self.mode]()  # 수정된 부분
        # self.logger.warn("yaw: " + str(self.sensor.yaw))

        # self.logger.info("mode: " + self.mode)
        # cv2.waitKey(1)
        pass
