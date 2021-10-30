#!/usr/bin/env python3

# fmt: off
import os
import random
import sys

import numpy as np

from YOLOv5.detect import detect as detect_box
from YOLOv5.utils.plots import plot_one_box
from YOLOv5.models.experimental import attempt_load

from types import SimpleNamespace
import cv2 as cv
# fmt: on



class DirectionDetector():
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

    def __call__(self, image, debug=False):
        if debug:
            self.debug = True
            self.image_marked = image.copy()
        direction = self.detect_direction(image)
        return direction

    def hello(self):
        return "hello"

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

        if boxes is not None and len(boxes) >= 1:
            for box in boxes:
                label, xywh, conf, xyxy = box
                c = int(label)
                plot_one_box(xyxy, self.image_marked, label=self.names[c], color=self.colors[c], line_thickness=self.opt.line_thickness)

        # 得到行驶方向
        return self.judge_direction(boxes)