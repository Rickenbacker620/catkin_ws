import socket
from collections import deque

import cv2 as cv
import numpy as np

from core.parameters import Parameters
from utils.common import Stabilizer, calc_slope, fit_line, get_bias_points
from utils.common import filter_abnormal_lines
from utils.draw import draw_lines, draw_points
from utils.warp import get_trapezoid_pts

WIDTH = 1920
HEIGHT = 1080

s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)


class LaneDetector:
    def __init__(self, parameters: Parameters) -> None:
        self.parameters = parameters
        self.debug = False
        self.left_color = (255, 0, 0)
        self.right_color = (0, 255, 0)
        self.center_color = (0, 0, 255)
        self.stabilizer = Stabilizer()
        self.zebra_lines_queue = deque(maxlen=30)
        self.has_zebra_lines = False

    def __call__(self, image, debug=False):
        if debug:
            self.debug = True
            self.image_marked = image.copy()
        preprocessed_image = self.preprocess(image)
        return self.find_bias(preprocessed_image)

    @staticmethod
    def merge_lines(lines, abnormal_thresh):
        filter_abnormal_lines(lines, abnormal_thresh)
        pts = np.array(lines).reshape(-1, 2)
        k, b = fit_line(pts)
        return k, b

    def preprocess(self, image):

        # 灰度处理
        gray = cv.cvtColor(image, cv.COLOR_BGR2GRAY)

        # 高斯模糊
        blur = cv.GaussianBlur(gray, (5, 5), 3)

        # Canny边缘检测
        edge = cv.Canny(blur, threshold1=30, threshold2=70)

        # 提取ROI
        roi_pts = get_trapezoid_pts(
            edge, self.parameters.roi.top, self.parameters.roi.bot)
        mask = np.zeros_like(edge)
        mask = cv.fillPoly(mask,
                           np.array(
                               [[roi_pts[0], roi_pts[1], roi_pts[3], roi_pts[2]]]),
                           color=255)
        roi = cv.bitwise_and(edge, edge, mask=mask)
        cv.imshow("roi", roi)

        return roi

    def find_bias(self, image):
        height, width = image.shape[:2]
        lines = cv.HoughLinesP(image, 1, np.pi / 180, self.parameters.lane.hough_thresh,
                               minLineLength=self.parameters.lane.hough_min_line_length,
                               maxLineGap=self.parameters.lane.hough_max_line_gap)

        if lines is not None:
            lines = lines.reshape(-1, 4).tolist()

            right_lines = [line for line in lines if 0 < calc_slope((line[0], line[1]), (line[2], line[3])) < 10]
            left_lines = [line for line in lines if -10 < calc_slope((line[0], line[1]), (line[2], line[3])) < 0]
            vertical_lines = [line for line in lines if abs(calc_slope(
                (line[0], line[1]), (line[2], line[3]))) < self.parameters.lane.zebra_slope_thresh]

            self.zebra_lines_queue.append(len(vertical_lines))

            self.has_zebra_lines = True if np.sum(
                self.zebra_lines_queue) > self.parameters.lane.zebra_line_count_thresh else False

            if len(left_lines) != 0 and len(right_lines) != 0:

                kl, bl = self.merge_lines(left_lines, self.parameters.lane.abnormal_left_thresh)
                kr, br = self.merge_lines(right_lines, self.parameters.lane.abnormal_right_thresh)

                ptr, ptl = get_bias_points(kr, br, kl, bl)

                ptc = int((ptl[0] + ptr[0]) // 2), 240

                bias = ptc[0] - width // 2

                self.stabilizer.push(bias, threshold=self.parameters.lane.bias_stabilizer_thresh)

                bias_filtered = int(self.stabilizer.data)

                if self.debug is True:
                    draw_lines(self.image_marked, *left_lines, color=self.left_color)
                    draw_lines(self.image_marked, *right_lines, color=self.right_color)
                    draw_lines(self.image_marked, *vertical_lines, color=(0, 255, 255))
                    draw_points(self.image_marked, ptr, color=self.right_color)
                    draw_points(self.image_marked, ptl, color=self.left_color)
                    bias_direction = "left" if bias_filtered > 0 else "right"
                    draw_points(self.image_marked, ptc,
                                color=self.center_color)
                    draw_points(self.image_marked, (width // 2 +
                                                    int(bias_filtered), ptc[1]), color=(255, 255, 0))
                    cv.putText(self.image_marked, "bias: " + bias_direction + str(abs(bias_filtered)),
                               (20, 20), cv.FONT_HERSHEY_COMPLEX, 0.5, (255, 0, 255))
                    cv.putText(self.image_marked, "zebra_line: " + str(self.has_zebra_lines),
                               (150, 20), cv.FONT_HERSHEY_COMPLEX, 0.5, (255, 0, 255))
                    cv.imshow("img_with_lines", self.image_marked)

                return bias_filtered
        else:
            return 0
