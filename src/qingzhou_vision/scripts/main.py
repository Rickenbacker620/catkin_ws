#!/usr/bin/env python3

# fmt: off
import os
import random
import sys

import numpy as np

from asd.util import say_hello

sys.path.append('./YOLOv5')
from YOLOv5.detect import detect as detect_box
from YOLOv5.utils.plots import plot_one_box
from YOLOv5.models.experimental import attempt_load
from LaneDetect.LandDetect import detect_line
from LaneDetect.common import get_test_points

from types import SimpleNamespace
import cv2 as cv
# fmt: on


opt = SimpleNamespace(weights='/home/yzu/catkin_ws/src/qingzhou_vision/scripts/YOLOv5/weights/last.pt',
                      source='YOLOv5/tsttt.png',
                      img_size=640,
                      conf_thres=0.25,
                      iou_thres=0.45,
                      device='cuda',
                      classes=None,
                      agnostic_nms=False,
                      augment=False,
                      update=False,
                      project='runs/detect',
                      name='exp',
                      exist_ok=False,
                      line_thickness=3,
                      hide_labels=False,
                      hide_conf=False)


# 得出行驶方向
'''
def judge_direction(boxes):
    if boxes is None or len(boxes) == 0:
        return None

    center_box = None
    min_center_x = None

    for box in boxes:
        label, xywh, conf, xyxy = box
        print(f"label:{label}")
        center_x = abs(xywh[0]-0.5)

        if min_center_x is None or center_x < min_center_x:
            center_box = box
            min_center_x = center_x

    dir = names[int(center_box[0])]
    print(dir)
    return dir
'''


# def judge_direction(boxes):
#     if boxes is None or len(boxes) == 0:
#         return None

#     center_box = None
#     min_center_x = None

#     for box in boxes:
#         _, xywh, _, _ = box
#         center_x = abs(xywh[0]-0.5)

#         if min_center_x is None or center_x < min_center_x:
#             center_box = box
#             min_center_x = center_x

#     dir = names[int(center_box[0])]
#     print(dir)
#     return dir

# 保存txt


def save_txt(name, dir, pt1, pt2, pt3, pt4):
    with open(f'results/{name[:-4]}.txt', 'w') as result:
        result.write(dir + '\n')
        result.write(f'{pt1[0]} {pt1[1]}\n')
        result.write(f'{pt2[0]} {pt2[1]}\n')
        result.write(f'{pt3[0]} {pt3[1]}\n')
        result.write(f'{pt4[0]} {pt4[1]}\n')

# 检测所有并保存结果


# def detect_all(filename):

#     # 读取图片
#     image = cv.imread(f'./images/{filename}')

#     # 首先检测地标
#     boxs = detect_box(model, image, opt)

#     # 再检测车道线
#     (k1, b1), (k2, b2) = detect_line(image)

#     # 画出bounding box
#     if boxs is not None and len(boxs) >= 1:
#         for box in boxs:
#             label, xywh, conf, xyxy = box
#             c = int(label)
#             plot_one_box(
#                 xyxy, image, label=names[c], color=colors[c], line_thickness=opt.line_thickness)

    # # 得到行驶方向
    # direction = judge_direction(boxs)

    # # 计算得到点
    # pt1, pt2, pt3, pt4 = get_test_points(k1, b1, k2, b2)

    # # 画出车道线
    # cv.line(image, pt1, pt3, (255, 255, 0), 3)
    # cv.line(image, pt2, pt4, (0, 255, 0), 3)

    # save_txt(filename, direction, pt1, pt2, pt3, pt4)

    # cv.imwrite(f'results/{filename}', image)


class DetectService():
    weights = 'YOLOv5/weights/last.pt'
    device = 'cuda',

    def __init__(self, opt) -> None:
        self.opt = opt
        self.model = attempt_load(
            self.opt.weights, map_location=self.opt.device)
        self.names = self.model.module.names if hasattr(
            self.model, 'module') else self.model.names
        self.colors = [[random.randint(0, 255)
                        for _ in range(3)] for _ in self.names]

    def detect_line(self, image):
        return detect_line(image)

    def judge_direction(self, boxes):
        if boxes is None or len(boxes) == 0:
            return 'None'

        center_box = None
        min_center_x = None

        for box in boxes:
            _, xywh, _, _ = box
            center_x = abs(xywh[0]-0.5)

            if min_center_x is None or center_x < min_center_x:
                center_box = box
                min_center_x = center_x

        dir = self.names[int(center_box[0])]
        return dir

    def detect_direction(self, image):
        boxes = detect_box(self.model, image, self.opt)
        image_copy = image[...]

        if boxes is not None and len(boxes) >= 1:
            for box in boxes:
                label, xywh, conf, xyxy = box
                c = int(label)
                plot_one_box(
                    xyxy, image_copy, label=self.names[c], color=self.colors[c], line_thickness=self.opt.line_thickness)

        # 得到行驶方向
        return self.judge_direction(boxes)

    def detect_all(self, image):
        # direction = self.detect_direction(image)
        (k1, b1), (k2, b2) = self.detect_line(image)
        print(image.shape)
        # 计算得到点
        try:
            pt1, pt2, pt3, pt4 = get_test_points(k1, b1, k2, b2)
        except:
            pt1, pt2, pt3, pt4 = (100, 100), (400, 100), (50, 200), (250, 200)

        # 画出车道线
        cv.line(image, pt1, pt3, (255, 255, 0), 3)
        cv.line(image, pt2, pt4, (0, 255, 0), 3)
        # cv.line(image, (100, 100), (50, 200), (255, 255, 0), 3)
        # cv.line(image, (400, 100), (250, 200), (0, 255, 0), 3)
        import socket
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        # cap = cv2.VideoCapture(0,cv2.CAP_GSTREAMER)
        img = cv.resize(image, (341, 256))  # 测试这个尺寸图像比较稳定，再大发送不稳定
        img = cv.imencode('.jpeg', img)[1]      # 使用jpeg格式压缩图像
        data_encode = np.array(img)
        data = data_encode.tostring()   # 转换为byte
        # cv2.waitKey(0)
        s.sendto(data, ('192.168.2.109', 8989))
        # print(direction)
        print(k1, b1, k2, b2)


def gstreamer_pipeline(
    capture_width=1280,
    capture_height=720,
    display_width=1280,
    display_height=720,
    framerate=10,
    flip_method=0,
):
    return (
        "nvarguscamerasrc ! "
        "video/x-raw(memory:NVMM), "
        "width=(int)%d, height=(int)%d, "
        "format=(string)NV12, framerate=(fraction)%d/1 ! "
        "nvvidconv flip-method=%d ! "
        "video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! "
        "videoconvert ! "
        "video/x-raw, format=(string)BGR ! appsink"
        % (
            capture_width,
            capture_height,
            framerate,
            flip_method,
            display_width,
            display_height,
        )
    )


def show_camera():
    # To flip the image, modify the flip_method parameter (0 and 2 are the most common)
    print(gstreamer_pipeline(flip_method=0))
    cap = cv.VideoCapture(gstreamer_pipeline(
        flip_method=0), cv.CAP_GSTREAMER)
    if cap.isOpened():
        window_handle = cv.namedWindow("CSI Camera", cv.WINDOW_AUTOSIZE)
        # Window
        while cv.getWindowProperty("CSI Camera", 0) >= 0:
            ret_val, img = cap.read()
            cv.imshow("CSI Camera", img)
            # This also acts as
            keyCode = cv.waitKey(30) & 0xFF
            # Stop the program on the ESC key
            if keyCode == 27:
                break
        cap.release()
        cv.destroyAllWindows()
    else:
        print("Unable to open camera")


if __name__ == '__main__':

    svc = DetectService(opt=opt)

    cap = cv.VideoCapture(gstreamer_pipeline(
        flip_method=0), cv.CAP_GSTREAMER)
    idx = 0
    if cap.isOpened():
        while True:
            idx = idx + 1
            if(idx < 10000):
                continue
            ret_val, image = cap.read()
            svc.detect_all(image)
