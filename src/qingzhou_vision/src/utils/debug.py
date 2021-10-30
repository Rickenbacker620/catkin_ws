import socket
from types import SimpleNamespace
import cv2 as cv
import numpy as np
from utils.common import gen_window


def nothing(x):
    pass


class ParamPanel():
    def __init__(self, window_name="Parameters"):
        self.window_name = window_name
        self._params = []
        gen_window([window_name])

    def add_param(self, name, bound, default=0):
        cv.createTrackbar(name, self.window_name, default, bound, nothing)
        self._params.append(name)

    def add_params(self, *param_list):
        for param in param_list:
            self.add_param(param[0], param[1], param[2])

    def get_param(self, name):
        return cv.getTrackbarPos(name, self.window_name)

    @property
    def params(self):
        param_dict = dict()
        for param in self._params:
            param_dict[param] = self.get_param(param)
        return SimpleNamespace(**param_dict)

class ImageSender:
    def __init__(self, url, port) -> None:
        self.s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.url = url
        self.port = port

    def send(self,image):
        img = cv.resize(image, (341, 256))
        img = cv.imencode('.jpeg', img)[1]
        data_encode = np.array(img)
        data = data_encode.tostring()
        self.s.sendto(data, (self.url, self.port))

image_sender = ImageSender('192.168.2.143',8989)