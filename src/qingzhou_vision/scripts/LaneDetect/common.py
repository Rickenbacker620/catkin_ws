import math
from types import SimpleNamespace
import cv2 as cv
import numpy as np


class AverageMeter():
    '''
    更新平均值
    '''

    def __init__(self):
        self.reset()

    def reset(self):
        self.valk = 0
        self.valb = 0

        self.avgk = 0
        self.avgb = 0

        self.sumk = 0
        self.sumb = 0

        self.count = 0

    def update(self, k, b):
        self.valk = k
        self.valb = b

        self.sumk += k
        self.sumb += b

        self.count += 1

        self.avgk = self.sumk/self.count
        self.avgb = self.sumb/self.count


r_avg = AverageMeter()
l_avg = AverageMeter()


def calc_slope(pt1, pt2):
    '''
    计算斜率

    参数: 两点
    返回: 斜率
    '''
    return (pt2[1]-pt1[1])/(pt2[0]-pt1[0])


def fit_line(points):
    '''
    对输入的点集拟合直线

    参数: 点集
    返回: 拟合直线的k,b
    '''
    line = cv.fitLine(points, distType=cv.DIST_L2,
                      param=0, reps=1e-2, aeps=1e-2).squeeze()

    k = line[1]/line[0]
    b = line[3] - k*line[2]
    return k, b


def pt2kb(pt1, pt2):
    '''
    两点算k,b

    参数: pt1,pt2
    返回: k,b
    '''
    k = calc_slope(pt1, pt2)
    x, y = pt2
    b = y-k*x
    return k, b


def kb2pt(img, k, b):
    '''
    k,b算两点

    参数: k,b
    返回: pt1,pt2
    '''
    _, w = img.shape[0:2]
    x1 = 0
    x2 = w
    y1 = int(k*x1+b)
    y2 = int(k*x2+b)
    return (x1, y1), (x2, y2)


def filter_points(lines):
    '''
    获取k和b符合要求的线段并返回左右两道的点集
    '''

    # 去除多余的1维,此时shape[线段数量,4]
    lines = lines.squeeze()

    # 根据两点坐标得出k,b
    k, b = pt2kb((lines[:, 0], lines[:, 1]), (lines[:, 2], lines[:, 3]))

    # 根据斜率选出左右车道线
    right_lane = np.intersect1d(
        np.where((1/3 < k) & (k < 1)),
        np.where((-600 < b) & (b < -450)))
    right_points = lines[right_lane].reshape(-1, 2)

    left_lane = np.intersect1d(
        np.where((-1 < k) & (k < -(1/3))),
        np.where((150 < b) & (b < 450)))
    left_points = lines[left_lane].reshape(-1, 2)

    return right_points, left_points


def find_lane(img):
    blur = cv.GaussianBlur(img, (3, 3), 1)
    # 首先对图像进行canny边缘检测
    edge = cv.Canny(blur, 50, 100)

    # 概率霍夫变换找出所有直线线段
    lines = cv.HoughLinesP(edge, 1, np.pi/180, 50,
                           minLineLength=10, maxLineGap=50)

    # 确保确保至少有1条线段
    if lines is None or len(lines) <= 1:
        return (r_avg.avgk, r_avg.avgb), (l_avg.avgk, l_avg.avgb)

    right_points, left_points = filter_points(lines)

    # 如果没有点符合要求，则利用平均值
    if len(right_points) == 0 or len(left_points) == 0:
        return (r_avg.avgk, r_avg.avgb), (l_avg.avgk, l_avg.avgb)

    r_result = fit_line(right_points)
    l_result = fit_line(left_points)

    r_avg.update(*r_result)
    l_avg.update(*l_result)

    return r_result, l_result


def get_test_points(k1, b1, k2, b2):
    # y1, y2, y3, y4 = 810, 810, 1080, 1080
    y1, y2, y3, y4 = 500, 500, 720, 720
    x1, x2, x3, x4 = int((y1-b2)//k2), int((y2-b1) //
                                           k1), int((y3-b2)//k2), int((y4-b1)//k1)
    return (x1, y1), (x2, y2), (x3, y3), (x4, y4)
