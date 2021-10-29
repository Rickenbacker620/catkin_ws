import os
import cv2 as cv
import numpy as np
from utils.common import Stabilizer, calc_slope, fit_line, get_bias, get_bias_points
from utils.camera_calib import load_coefficients, undistort_image
from utils.common import filter_abnormal_lines
from utils.draw import draw_lines, draw_points, draw_slope, plot_one_box
from utils.warp import get_trapezoid_pts, get_warp
import core.params as params
import socket

WIDTH = 1920
HEIGHT = 1080


s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)


class LaneDetector:
    def __init__(self, params) -> None:
        self.params = params
        self.debug = False
        self.left_color = (255, 0, 0)
        self.right_color = (0, 255, 0)
        self.center_color = (0, 0, 255)
        self.stabilizer = Stabilizer()

    def __call__(self, image, debug=False):
        if debug:
            self.debug = True
            self.image_marked = image.copy()
        preprocessed_image = self.preprocess(image)
        return self.find_bias(preprocessed_image)

    def preprocess(self, image):

        # 灰度处理
        gray = cv.cvtColor(image, cv.COLOR_BGR2GRAY)

        # 高斯模糊
        blur = cv.GaussianBlur(gray, (5, 5), 3)

        # Canny边缘检测
        edge = cv.Canny(blur, threshold1=30, threshold2=70)

        # 提取ROI
        roi_pts = get_trapezoid_pts(
            edge, self.params.roi.top, (self.params.roi.bot))
        mask = np.zeros_like(edge)
        mask = cv.fillPoly(mask,
                           np.array(
                               [[roi_pts[0], roi_pts[1], roi_pts[3], roi_pts[2]]]),
                           color=255)
        roi = cv.bitwise_and(edge, edge, mask=mask)

        return roi

    def find_bias(self, image):
        height, width = image.shape[:2]
        lines = cv.HoughLinesP(image, 1, np.pi/180, 20,
                               minLineLength=20, maxLineGap=100)

        if lines is not None:
            lines = lines.reshape(-1, 4).tolist()

            left_lines = [line for line in lines if calc_slope(
                (line[0], line[1]), (line[2], line[3])) > 0]
            right_lines = [line for line in lines if calc_slope(
                (line[0], line[1]), (line[2], line[3])) < 0]
            if len(left_lines) != 0 and len(right_lines) != 0:
                filter_abnormal_lines(right_lines, 0.2)
                filter_abnormal_lines(left_lines, 0.2)

                right_pts = np.array(right_lines).reshape(-1, 2)

                left_pts = np.array(left_lines).reshape(-1, 2)

                kr, br = fit_line(right_pts)
                kl, bl = fit_line(left_pts)

                ptr, ptl = get_bias_points(kr, br, kl, bl)

                ptc = int((ptl[0]+ptr[0])//2), 240

                bias = ptc[0] - width//2

                self.stabilizer.push(bias, threshold=30)

                bias_filtered = int(self.stabilizer.data)

                if self.debug is True:
                    draw_lines(self.image_marked, *left_lines,
                               color=self.left_color)
                    draw_lines(self.image_marked, *right_lines,
                               color=self.right_color)
                    draw_points(self.image_marked, ptr, color=self.right_color)
                    draw_points(self.image_marked, ptl, color=self.left_color)
                    bias_direction = "left" if bias_filtered > 0 else "right"
                    draw_points(self.image_marked, ptc,
                                color=self.center_color)
                    draw_points(self.image_marked, (width//2 +
                                int(bias_filtered), ptc[1]), color=(255, 255, 0))
                    cv.putText(self.image_marked, "bias: " + bias_direction + str(abs(bias_filtered)),
                               (20, 20), cv.FONT_HERSHEY_COMPLEX, 0.5, (255, 255, 255))
                    # cv.imshow("img_with_lines", self.image_marked)

                    # 测试这个尺寸图像比较稳定，再大发送不稳定
                    img = cv.resize(self.image_marked, (341, 256))
                    img = cv.imencode('.jpeg', img)[1]      # 使用jpeg格式压缩图像
                    data_encode = np.array(img)
                    data = data_encode.tostring()   # 转换为byte
                    s.sendto(data, ('192.168.2.109', 8989))

                return bias_filtered
        else:
            return 0


class ImageDetector:
    image_raw = None
    image_marked = None
    params = params.ImageParams()

    def __init__(self, image=None) -> None:
        self.mtx, self.dist = load_coefficients(
            "./config/calibration_chessboard.yml")
        self.filter = Stabilizer()
        if image is not None:
            self.set_image(image)
        self.lane_detetcor = LaneDetector(self.params)

    @property
    def width(self):
        return self.image_raw.shape[1]

    @property
    def height(self):
        return self.image_raw.shape[0]

    def set_image(self, image):
        if image.shape != (720, 1280):
            image = cv.resize(image, (1280, 720))
        undistort = cv.resize(undistort_image(
            image, self.mtx, self.dist), (640, 360))
        self.image_raw = undistort
        self.image_marked = undistort.copy()

    def draw_line(self,  pt1, pt2, image=None):
        if image is None:
            image = self.image_marked
        cv.line(image, pt1, pt2, (255, 255, 0), 2)

    def draw_box(self, pt1, pt2, color=None, label=None, line_thickness=3):
        plot_one_box((*pt1, *pt2), self.image_marked,
                     color, label, line_thickness)
