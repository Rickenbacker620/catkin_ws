# fmt: off
import sys
import argparse
import time
from pathlib import Path
from types import SimpleNamespace
import numpy as np

import cv2
import torch
import torch.backends.cudnn as cudnn
from numpy import random
import os

sys.path.append('/home/yzu/catkin_ws/src/qingzhou_vision/scripts/YOLOv5')

from models.experimental import attempt_load
from utils.datasets import LoadStreams, LoadImages
from utils.general import check_img_size, check_requirements, check_imshow, non_max_suppression, apply_classifier, \
    scale_coords, xyxy2xywh, strip_optimizer, set_logging, increment_path, save_one_box
from utils.plots import plot_one_box
from utils.torch_utils import select_device, load_classifier, time_synchronized
from utils.datasets import letterbox
# fmt: on


def detect(model, image, opt):

    result = []
    source, weights, imgsz = opt.source, opt.weights, opt.img_size

    # Initialize
    set_logging()

    device = select_device(opt.device)
    half = device.type != 'cpu'  # half precision only supported on CUDA

    # Load model
    stride = int(model.stride.max())  # model stride
    imgsz = check_img_size(imgsz, s=stride)  # check img_size
    if half:
        model.half()  # to FP16

    # Set Dataloader
    # dataset = LoadImages(source, img_size=imgsz, stride=stride)

    img0 = image

    # Padded resize
    img = letterbox(img0, imgsz, stride=stride)[0]

    # Convert
    img = img[:, :, ::-1].transpose(2, 0, 1)  # BGR to RGB, to 3x416x416
    img = np.ascontiguousarray(img)
    dataset = [('', img, img0, None)]

    # Get names and colors
    names = model.module.names if hasattr(model, 'module') else model.names

    # Run inference
    if device.type != 'cpu':
        model(torch.zeros(1, 3, imgsz, imgsz).to(device).type_as(
            next(model.parameters())))  # run once
    t0 = time.time()
    for path, img, im0s, _ in dataset:
        img = torch.from_numpy(img).to(device)
        img = img.half() if half else img.float()  # uint8 to fp16/32
        img /= 255.0  # 0 - 255 to 0.0 - 1.0
        if img.ndimension() == 3:
            img = img.unsqueeze(0)

        # Inference
        t1 = time_synchronized()
        pred = model(img, augment=opt.augment)[0]

        # Apply NMS
        pred = non_max_suppression(
            pred, opt.conf_thres, opt.iou_thres, classes=opt.classes, agnostic=opt.agnostic_nms)
        t2 = time_synchronized()

        # Process detections
        for i, det in enumerate(pred):  # detections per image
            _, s, im0, frame = path, '', im0s.copy(), getattr(dataset, 'frame', 0)

            # p = Path(p)  # to Path
            s += '%gx%g ' % img.shape[2:]  # print string
            # normalization gain whwh
            gn = torch.tensor(im0.shape)[[1, 0, 1, 0]]
            if len(det):
                # Rescale boxes from img_size to im0 size
                det[:, :4] = scale_coords(
                    img.shape[2:], det[:, :4], im0.shape).round()

                # Print results
                for c in det[:, -1].unique():
                    n = (det[:, -1] == c).sum()  # detections per class
                    # add to string
                    s += f"{n} {names[int(c)]}{'s' * (n > 1)}, "

                # Write results
                for *xyxy, conf, cls in reversed(det):
                    xywh = (xyxy2xywh(torch.tensor(xyxy).view(1, 4)
                                      ) / gn).view(-1).tolist()  # normalized xywh
                    # label format
                    line = (cls, xywh, conf, xyxy)
                    result.append(line)

            # Print time (inference + NMS)
            # print(f'{s}Done. ({t2 - t1:.3f}s)')

    # print(f'Done. ({time.time() - t0:.3f}s)')
    return result
