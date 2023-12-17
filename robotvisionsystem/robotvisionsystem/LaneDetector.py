#! /usr/bin/env python3
# -*- coding:utf-8 -*-

from rclpy.node import Node

import cv2
import numpy as np
from collections import deque
from robotvisionsystem.BEV import BEV
from robotvisionsystem.Logger import Logger


# colors
red, green, blue, yellow = (0, 0, 255), (0, 255, 0), (255, 0, 0), (0, 255, 255)


class LaneDetector:
    '''
    Detects left, middle, right lane from an image and calculate angle of the lane.
    Uses canny, houghlinesP for detecting possible lane candidates.
    Calculates best fitted lane position and predicted lane position from previous result.
    '''

    def __init__(self):
        # Ensure that the node argument is indeed an instance of rclpy.node.Node
        # if not isinstance(node, Node):
        #     raise TypeError("Logger expects an rclpy.node.Node instance.")
        # self.node = node
        # self.logger = Logger(self.node)

        self.bev = BEV()

        # canny params
        self.canny_low, self.canny_high = 100, 120

        # HoughLineP params
        self.hough_threshold, self.min_length, self.min_gap = 10, 50, 10

        # initial state
        self.angle = 0.0
        self.prev_angle = deque([0.0], maxlen=5)
        self.lane = np.array([160.0, 250., 340.])

        # filtering params:
        self.angle_tolerance = np.radians(30)
        self.cluster_threshold = 25

        self.target_lane = 0.0

    def to_canny(self, img, show=False):
        img = cv2.GaussianBlur(img, (7, 7), 0)
        img = cv2.Canny(img, self.canny_low, self.canny_high)
        if show:
            cv2.imshow('canny', img)
            cv2.waitKey(1)
        return img

    def hough(self, img, show=False):
        lines = cv2.HoughLinesP(
            img, 1, np.pi/183, self.hough_threshold, self.min_gap, self.min_length)
        if show:
            hough_img = np.zeros((img.shape[0], img.shape[1], 3))
            if lines is not None:
                for x1, y1, x2, y2 in lines[:, 0]:
                    cv2.line(hough_img, (x1, y1), (x2, y2), red, 2)
            cv2.imshow('hough', hough_img)
            cv2.waitKey(1)
        return lines

    def filter(self, lines, show=True):
        '''
        filter lines that are close to previous angle and calculate its positions
        '''

        thetas, positions = [], []  # 각도와 위치를 저장하기 위한 리스트 초기화
        if show:
            # 필터링 결과를 표시할 빈 이미지 초기화
            filter_img = np.zeros(
                (self.bev.warp_img_h, self.bev.warp_img_w, 3))

        if lines is not None:  # 입력된 직선들이 존재할 경우
            for x1, y1, x2, y2 in lines[:, 0]:  # 각 직선에 대해
                if y1 == y2:  # 직선이 수평일 경우 무시
                    continue

                # 직선의 방향에 따라 플래그 설정
                flag = 1 if y1-y2 > 0 else -1

                # 직선의 각도 계산
                theta = np.arctan2(flag * (x2-x1), flag * 0.9 * (y1-y2))

                # 계산된 각도가 저장된 이전 각도와 충분히 가까운 경우
                if abs(theta - self.angle) < self.angle_tolerance:
                    # 해당 직선의 위치 계산
                    position = float(
                        (x2-x1)*(self.bev.warp_img_mid-y1))/(y2-y1) + x1
                    thetas.append(theta)  # 계산된 각도 저장
                    positions.append(position)  # 계산된 위치 저장
                    if show:  # 필터링된 직선 표시
                        cv2.line(filter_img, (x1, y1), (x2, y2), red, 2)

        # 이전 각도 리스트에 현재 각도 추가
        self.prev_angle.append(self.angle)

        # 계산된 각도들의 평균값으로 현재 각도 업데이트
        if thetas:
            self.angle = np.mean(thetas)

        if show:  # 필터링된 직선들을 표시
            cv2.imshow('filtered lines', filter_img)
            cv2.waitKey(1)
        return positions  # 계산된 위치들 반환

    def get_cluster(self, positions):
        '''
        group positions that are close to each other
        '''

        clusters = []  # 클러스터를 저장하기 위한 리스트 초기화

        # 각 위치에 대해
        for position in positions:
            # 해당 위치가 유효한 범위 내에 있을 경우
            if 0 <= position < self.bev.warp_img_w:
                for cluster in clusters:
                    # 해당 위치와 가장 가까운 클러스터를 찾기 위한 조건
                    if abs(cluster[0] - position) < self.cluster_threshold:
                        # 클러스터에 위치 추가
                        cluster.append(position)
                        break
                else:
                    # 어떤 클러스터에도 속하지 않을 경우 새로운 클러스터 생성
                    clusters.append([position])

        # 각 클러스터의 평균 위치 계산
        lane_candidates = [np.mean(cluster) for cluster in clusters]

        return lane_candidates  # 계산된 평균 위치들 반환

    def predict_lane(self):
        '''
        predicts lane positions from previous lane positions and angles
        '''
        # 이전 차선의 위치와 각도를 기반으로 중심 외의 차선 위치 예측
        # predicted_lane = self.lane[1] + [-220/max(
        #     np.cos(self.angle), 0.75), 0, 240/max(np.cos(self.angle), 0.75)]
        predicted_lane = self.lane[1] + [-183/max(
            np.cos(self.angle), 0.75), 0, 183/max(np.cos(self.angle), 0.75)]

        # 예측된 차선 위치에 대한 조정: 이전 각도의 평균과 현재 각도의 차이를 반영
        # predicted_lane = predicted_lane + \
        #     (self.angle - np.mean(self.prev_angle))*70
        predicted_lane = self.lane[1] + [-183/max(np.cos(self.angle), 0.75), 0]

        return predicted_lane  # 예측된 차선 위치 반환

    def update_lane(self, lane_candidates, predicted_lane):
        '''
        calculate lane position using best fitted lane and predicted lane
        '''

        # lane_candidates가 비어 있으면 예측된 차선 위치를 사용하여 차선을 업데이트
        if not lane_candidates:
            self.lane = predicted_lane
            return

        # 가능한 차선 조합들을 저장하기 위한 리스트 초기화
        possibles = []

        # 각 lane_candidate에 대해
        for lc in lane_candidates:

            # 현재 lc와 self.lane의 각 원소와의 차이를 계산하고, 그 중 최소값의 인덱스를 얻음
            idx = np.argmin(abs(self.lane-lc))

            # 첫 번째 차선에 대한 처리
            if idx == 0:
                # lc를 기준으로 예상되는 다른 차선의 위치를 계산
                estimated_lane = [
                    lc,
                    lc + 183/max(np.cos(self.angle), 0.75),
                ]

                # 가능한 모든 조합을 possibles 리스트에 추가
                for lc2 in lane_candidates:
                    if abs(lc2-estimated_lane[1]) < 50:
                        possibles.append([lc, lc2])

            # 두 번째 차선에 대한 처리
            elif idx == 1:
                # lc를 기준으로 예상되는 다른 차선의 위치를 계산
                estimated_lane = [
                    lc - 183/max(np.cos(self.angle), 0.75),
                    lc,
                ]

                # 가능한 모든 조합을 possibles 리스트에 추가
                for lc1 in lane_candidates:
                    if abs(lc1-estimated_lane[0]) < 50:
                        possibles.append([lc1, lc])

        # possibles 리스트가 비어 있는 경우에 대한 처리
        if not possibles:
            self.lane = predicted_lane
            return

        # 각 가능한 조합과 예측된 차선 위치 간의 오차를 계산
        possibles = np.array(possibles)
        error = np.sum((possibles-predicted_lane)**2, axis=1)

        # 오차가 가장 작은 차선 조합을 선택
        best = possibles[np.argmin(error)]

        # 최종 차선 위치를 예측된 차선 위치와 최적의 조합의 가중 평균으로 업데이트
        self.lane = 0.4 * best + 0.6 * predicted_lane

    def mark_lane(self, img, lane=None):
        '''
        mark calculated lane position to an image 
        '''
        img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
        if lane is None:
            lane = self.lane
        l1, l3 = self.lane
        cv2.circle(img, (int(l1), self.bev.warp_img_mid),
                   3, red, 5, cv2.FILLED)
        # cv2.circle(img, (int(l2), self.bev.warp_img_mid),
        #            3, green, 5, cv2.FILLED)
        cv2.circle(img, (int(l3), self.bev.warp_img_mid),
                   3, blue, 5, cv2.FILLED)
        cv2.imshow('marked', img)
        cv2.waitKey(1)
        if l1 < 0 or l3 > self.bev.warp_img_w:
            self.target_lane = 250
        else:
            self.target_lane = (l1 + l3) / 2

    def __call__(self, img):
        '''
        returns angle and cte of a target lane from an image
        angle : radians
        cte : pixels
        '''

        canny = self.to_canny(img, show=False)
        bev = self.bev(canny, show=False)
        lines = self.hough(bev, show=False)
        positions = self.filter(lines, show=False)
        lane_candidates = self.get_cluster(positions)
        predicted_lane = self.predict_lane()
        self.update_lane(lane_candidates, predicted_lane)
        self.mark_lane(bev)

        return self.target_lane
