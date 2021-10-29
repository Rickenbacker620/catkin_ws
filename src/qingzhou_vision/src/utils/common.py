from collections import deque
import queue
import cv2 as cv
import numpy as np
from queue import Queue


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
    dx = (pt2[0]-pt1[0])
    dy = (pt2[1]-pt1[1])
    if (dx == 0):
        dx = 1
    return dy/dx


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
    y1, y2, y3, y4 = 810, 810, 1080, 1080
    x1, x2, x3, x4 = int((y1-b2)//k2), int((y2-b1) //
                                           k1), int((y3-b2)//k2), int((y4-b1)//k1)
    return (x1, y1), (x2, y2), (x3, y3), (x4, y4)


def gen_window(windows, width=1280, height=720):
    for window in windows:
        cv.namedWindow(window, 0)
        cv.resizeWindow(window, width, height)
        cv.moveWindow(window, 100, 100)


class TestImg():
    def __init__(self, source, video=False, preload=False):
        self.source = source

        self.video = False
        if video is True:
            self.video = True
            self.cap = cv.VideoCapture(source)
            ret, self.image = self.cap.read()
        elif preload is True:
            self.image = source
        else:
            self.image = cv.imread(source)

        gen_window(['result'])

    def get_image(self, num=None):
        if self.video == True and num != None:
            cap2 = cv.VideoCapture(self.source)
            cap2.set(cv.CAP_PROP_POS_FRAMES, num)
            _, frame = cap2.read()
            return frame
        return self.image

    def get_shape(self):
        return self.image.shape

    def run_test(self, pipeline):
        while True:
            result = pipeline(self.image)
            cv.imshow('result', result)
            if self.video is True:
                ret, self.image = self.cap.read()
                if ret == False:
                    print('video ends')
            if cv.waitKey(27) == 27:
                break
        if self.video is True:
            self.cap.release()
        cv.destroyAllWindows()


def filter_abnormal_lines(lines, threshold):
    '''
    斜率以纵向为x轴,横向为y轴(方便后续计算k值)
    '''
    slopes = [calc_slope((line[1], line[0]),
                         (line[3], line[2])) for line in lines]
    while len(lines) > 0:
        mean = np.mean(slopes)
        diff = [abs(slope - mean) for slope in slopes]
        idx = np.argmax(diff)
        if diff[idx] > threshold:
            slopes.pop(idx)
            lines.pop(idx)
        else:
            break
    return lines, mean


class Stabilizer:

    def __init__(self) -> None:
        # def half_norm(x):
        #     return np.exp(-np.square(x)/2)*(np.sqrt(2/np.pi))

        self.weighs = np.arange(30, 1, -3)

        self.weighs = self.weighs / np.sum(self.weighs)

        # self.weighs = [half_norm(sample)
        #                for sample in samples if half_norm(sample) > 0]

        self.queue = deque(maxlen=len(self.weighs))

    @property
    def data(self):
        sum = 0
        for idx, data in enumerate(self.queue):
            sum += self.weighs[idx]*data
        return sum
        # return self.queue[-1]

    def push(self, data, threshold):
        if len(self.queue) == self.queue.maxlen:
            copy = self.queue.copy()
            copy.remove(np.max(copy))
            copy.remove(np.min(copy))
            mean = np.mean(copy)
            diff = abs(data - mean)
            if np.max(copy) == np.min(copy):
                self.queue.pop()
                self.queue.pop()
                self.queue.append(data)
            elif diff < threshold:
                self.queue.append(data)
            else:
                self.queue.append(mean)

        else:
            self.queue.append(data)


def get_bias(image, slope):
    copy = image.copy()
    h, w = image.shape[:2]
    y_center = h//2
    x_center = w//2
    dx = y_center // slope
    return (x_center-dx, 0)


def get_bias_points(kr, br, kl, bl, y=240):
    xr, xl = int((y-br)//kr), int((y-bl) // kl)
    return (xr, y), (xl, y)
