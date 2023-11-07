#! /usr/bin/env python
# -*- coding:utf-8 -*-

import cv2

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from std_msgs.msg import String

from cv_bridge import CvBridge, CvBridgeError

import numpy as np


# colors
class TrafficLightDetector(object):
    '''
    Detects rectangle shaped contour of given specification range
    '''
    def __init__(self):
        self.traffic_light = 'Red'
        self.detected = False
        self.traffic_light_threshold = 85

    def detect_traffic_light(self, image):
        # 640x480의 이미지를 320의 480으로 자릅니다
        image = image[:250, 200:440]
        cv2.imshow('image', image)
        # 자른 이미지를 가우시안 블러를 이용하여 노이즈를 제거합니다.
        # 248 164, 382 206
        # 5x5 커널을 사용하여 가우시안 블러를 적용합니다.
        # 가우시안 블러를 이용하여 노이즈를 제거합니다.
        blur = cv2.GaussianBlur(image, (5, 5), 0)
        # cv2.imshow('blur', blur)
        # HLS 색공간으로 변환하여 채널을 분리합니다.
        _, L, _ = cv2.split(cv2.cvtColor(blur, cv2.COLOR_BGR2HLS))
        # cv2.imshow('L', L)  
        # 임계값을 적용하여 이진화 이미지를 얻습니다.

        _, lane = cv2.threshold(
            L, self.traffic_light_threshold, 255, cv2.THRESH_BINARY)
        cv2.imshow('lane', lane)
        # Canny 엣지 검출
        edges = cv2.Canny(lane, 70, 200)
        cv2.imshow('edges', edges)


        contours, _ = cv2.findContours(
            edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        # 컨투어를 둘러싸는 가장 작은 직사각형을 찾습니다.


        for cont in contours:
            # 컨투어를 둘러싸는 가장 작은 직사각형을 찾습니다.
            x, y, w, h = cv2.boundingRect(cont)

            # 조건을 확인하여 너비가 30, 높이가 60에 가까운 직사각형만을 검출합니다.
            # 허용 범위는 +/- 10으로 설정하였습니다.
            # 23, 57 640ㅍ480 
            # 33, 79
            if (10 <= w <= 100) and (0 <= h <= 25):
                self.detected = True
                # 해당 조건을 만족하는 직사각형 영역에 초록색 사각형을 그립니다.
                cv2.rectangle(image, (x, y), (x + w, y + h), (0, 255, 0), 2)
                

        
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        
        # 색상 범위 정의 
        # rgb 255, 53, 62값을 인식하기 위해 색상 범위를 조정하였습니다.
        # RGB 색상 정의
        red_color = np.uint8([[[255, 53, 62]]])
        green_color = np.uint8([[[23, 255, 83]]])
        yellow_color = np.uint8([[[255, 255, 53]]])

        # RGB 색상을 HSV 색상으로 변환
        hsv_red_color = cv2.cvtColor(red_color, cv2.COLOR_RGB2HSV)
        hsv_green_color = cv2.cvtColor(green_color, cv2.COLOR_RGB2HSV)
        hsv_yellow_color = cv2.cvtColor(yellow_color, cv2.COLOR_RGB2HSV)

        red_lower = np.array([hsv_red_color[0][0][0] - 1, 120, 200])
        red_upper = np.array([hsv_red_color[0][0][0] + 1, 255, 255])
        green_lower = np.array([hsv_green_color[0][0][0] - 1, 120, 70])
        green_upper = np.array([hsv_green_color[0][0][0] + 1, 255, 255])
        yellow_lower = np.array([hsv_yellow_color[0][0][0] - 1, 120, 70])
        yellow_upper = np.array([hsv_yellow_color[0][0][0] + 1, 255, 255])


        # 색상 범위에 따른 마스크 생성
        red_mask = cv2.inRange(hsv, red_lower, red_upper)
        cv2.imshow('red_mask', red_mask)

        green_mask = cv2.inRange(hsv, green_lower, green_upper)
        cv2.imshow('green_mask', green_mask)
        yellow_mask = cv2.inRange(hsv, yellow_lower, yellow_upper)
        cv2.imshow('yellow_mask', yellow_mask)

        # 마스크에 따라 신호등 색상 감지
        red_detected = cv2.countNonZero(red_mask)
        green_detected = cv2.countNonZero(green_mask)
        yellow_detected = cv2.countNonZero(yellow_mask)
        
        # 신호등을 인식한 경우 
        if self.detected:
            if red_detected > green_detected and red_detected > yellow_detected:
                return 'Red'
            elif green_detected > red_detected and green_detected > yellow_detected:
                return 'Green'
            elif yellow_detected > red_detected and yellow_detected > green_detected:
                return 'Yellow'
            else:
                return 'Unknown'
        else:
            return 'Unknown'
        
    def __call__(self, image):
        self.traffic_light = self.detect_traffic_light(image)
        cv2.imshow('Traffic Light Detection', image)
        cv2.waitKey(1)
        return self.traffic_light


class TrafficLightDetectionNode(Node):

    def __init__(self):
        super().__init__('stop_line_detection_node')

        self.bridge = CvBridge()
        self.detector = TrafficLightDetector()

        self.sub_image = self.create_subscription(
            Image, '/car/sensor/camera/front', self.image_callback, 10)

        # You may want to publish the detected lane or some other information
        # For simplicity, we'll republish the image with the lane markings
        self.pub_centor_lane = self.create_publisher(
            String, '/traffic_light', 10)
        

    def image_callback(self, img_msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(img_msg, "bgr8")
        except CvBridgeError as e:
            self.get_logger().warn('Failed to convert image: %s' % e)
            return

        # Use the LaneDetector logic here
        detected = self.detector(cv_image)  # As an example
        print(detected)

        # Publish the detected lane
        self.pub_centor_lane.publish(String(data=detected))


def main(args=None):
    rclpy.init(args=args)

    node = TrafficLightDetectionNode()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
