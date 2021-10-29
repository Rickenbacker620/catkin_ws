from types import SimpleNamespace
import cv2 as cv
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
