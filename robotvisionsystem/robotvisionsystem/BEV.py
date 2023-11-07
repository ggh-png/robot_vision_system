#! /usr/bin/env python
# -*- coding:utf-8 -*-

import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError


class BEV:
    '''
    Calibrates camera images to remove distortion and transforms to bird-eye-view image
    '''

    def __init__(self):
        # calibration config
        self.img_size = (640, 480)
        self.warp_img_w, self.warp_img_h, self.warp_img_mid = 500, 120, 60

        # perspective config
        warpx_mid, warpx_margin_hi, warpx_margin_lo, warpy_hi, warpy_lo, tilt = 320, 200, 319, 325, 375, -5
        self.warp_src = np.array([[warpx_mid+tilt-warpx_margin_hi, warpy_hi], [warpx_mid+tilt+warpx_margin_hi, warpy_hi],
                                  [warpx_mid-warpx_margin_lo,  warpy_lo], [warpx_mid+warpx_margin_lo, warpy_lo]], dtype=np.float32)

        # Adjusted to new image width
        self.warp_dist = np.array([[60, 0], [500-60, 0],
                                   [60, 119], [500-60, 119]], dtype=np.float32)

        self.M = cv2.getPerspectiveTransform(self.warp_src, self.warp_dist)

    def to_perspective(self, img, show=False):
        img = cv2.warpPerspective(
            img, self.M, (self.warp_img_w, self.warp_img_h), flags=cv2.INTER_LINEAR)
        if show:
            cv2.imshow('bird-eye-view', img)
            cv2.waitKey(1)
        return img

    def __call__(self, img, show=False):
        '''
        return bird-eye-view image of an input image
        '''
        img = self.to_perspective(img, show=show)
        return img
