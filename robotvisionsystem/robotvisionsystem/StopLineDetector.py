#! /usr/bin/env python
# -*- coding:utf-8 -*-

import cv2
from .BEV import BEV

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

# colors
red, green, blue, yellow = (0, 0, 255), (0, 255, 0), (255, 0, 0), (0, 255, 255)


class StopLineDetector(object):
    '''
    Detects rectangle shaped contour of given specification range
    '''

    def __init__(self):

        # BEV
        self.bev = BEV()

        # stopline detection param
        self.stopline_threshold = 125
        self.area_threshold = 2000  # 2000
        self.lengh_threshold = 300

    def __call__(self, img):
        '''
        return True if stopline is detected else False
        '''
        bev = self.bev(img)
        # 가우시안 블러
        # 5x5 커널을 사용하여 가우시안 블러를 적용합니다.
        # 이유는 노이즈를 제거하기 위해서입니다.
        blur = cv2.GaussianBlur(bev, (5, 5), 0)

        # cv2.waitKey(1)
        # HLS 색공간으로 변환
        # HLS 색공간은 색상(Hue), 채도(Saturation), 명도(Value)로 구성되어 있습니다.
        _, L, _ = cv2.split(cv2.cvtColor(blur, cv2.COLOR_BGR2HLS))
        # 임계값을 적용하여 이진화 이미지를 얻습니다.
        _, lane = cv2.threshold(
            L, self.stopline_threshold, 255, cv2.THRESH_BINARY)
        # cane = cv2.Canny(L, 50, 150)

        contours, _ = cv2.findContours(
            lane, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

        detected = False

        rectangle_count = 0  # 사각형의 개수를 저장하는 변수

        for cont in contours:
            # 컨투어를 둘러싸는 가장 작은 직사각형을 찾습니다.
            x, y, w, h = cv2.boundingRect(cont)

            # 조건을 확인하여 너비가 30, 높이가 60에 가까운 직사각형만을 검출합니다.
            # 허용 범위는 +/- 10으로 설정하였습니다.
            # 23, 57
            if (20 <= w <= 40) and (20 <= h <= 70):
                rectangle_count += 1  # 사각형 조건에 부합하면 카운트 증가
                # 해당 조건을 만족하는 직사각형 영역에 초록색 사각형을 그립니다.
                cv2.rectangle(bev, (x, y), (x + w, y + h), green, 2)

        # 4개의 사각형을 검출했을 때만 detected를 True로 설정합니다.
        detected = True if rectangle_count > 4 else False
        # print("rectangle_count : ", rectangle_count)
        cv2.imshow('stopline', bev)
        cv2.waitKey(1)
        # if detected:
        #     print("detected ", detected)
        return detected


class StopLineDetectionNode(Node):

    def __init__(self):
        super().__init__('stop_line_detection_node')

        self.bridge = CvBridge()
        self.detector = StopLineDetector()

        self.sub_image = self.create_subscription(
            Image, '/car/sensor/camera/front', self.image_callback, 10)

        # You may want to publish the detected lane or some other information
        # For simplicity, we'll republish the image with the lane markings
        self.pub_stop_line = self.create_publisher(
            Bool, '/stopline', 10)


    def image_callback(self, img_msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(img_msg, "bgr8")
        except CvBridgeError as e:
            self.get_logger().warn('Failed to convert image: %s' % e)
            return

        # Use the LaneDetector logic here
        detected = self.detector(cv_image)  # As an example

        # Publish the detected lane
        self.pub_stop_line.publish(Bool(data=detected))
    


def main(args=None):
    rclpy.init(args=args)
    node = StopLineDetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
