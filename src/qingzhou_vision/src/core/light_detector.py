import cv2
import numpy as np
from os import listdir
from os.path import join
from collections import deque


class LightDetector:
    def __init__(self) -> None:
        self.light = None
        self.light_queue = deque(maxlen=10)

    def __call__(self, image):
        light = self.find_light(image)
        self.light_queue.append(light)
        lights = {'Red': 0, 'Green': 0, 'Yellow': 0}
        for light in self.light_queue:
            lights['Green'] += light['Green']
            lights['Red'] += light['Red']
            lights['Yellow'] += light['Yellow']
        cur_light = max(lights, key=lights.get)
        if cur_light == None:
            return 'Red'
        return cur_light

    def find_light(self, image):
        color_dict = {"Red": 0, "Yellow": 0, "Green": 0}

        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # color range
        lower_red1 = np.array([0, 100, 100])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([160, 100, 100])
        upper_red2 = np.array([180, 255, 255])
        lower_green = np.array([40, 50, 50])
        upper_green = np.array([90, 255, 255])
        lower_yellow = np.array([15, 150, 150])
        upper_yellow = np.array([35, 255, 255])
        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        maskg = cv2.inRange(hsv, lower_green, upper_green)
        masky = cv2.inRange(hsv, lower_yellow, upper_yellow)
        maskr = cv2.add(mask1, mask2)

        size = image.shape
        # print size

        # hough circle detect
        r_circles = cv2.HoughCircles(maskr, cv2.HOUGH_GRADIENT, 1, 80,
                                     param1=50, param2=10, minRadius=0, maxRadius=30)

        g_circles = cv2.HoughCircles(maskg, cv2.HOUGH_GRADIENT, 1, 60,
                                     param1=50, param2=10, minRadius=0, maxRadius=30)

        y_circles = cv2.HoughCircles(masky, cv2.HOUGH_GRADIENT, 1, 30,
                                     param1=50, param2=5, minRadius=0, maxRadius=30)

        # traffic light detect
        r = 5
        bound = 4.0 / 10
        if r_circles is not None:
            r_circles = np.uint16(np.around(r_circles))

            for i in r_circles[0, :]:
                if i[0] > size[1] or i[1] > size[0] or i[1] > size[0]*bound:
                    continue

                h, s = 0.0, 0.0
                for m in range(-r, r):
                    for n in range(-r, r):

                        if (i[1]+m) >= size[0] or (i[0]+n) >= size[1]:
                            continue
                        h += maskr[i[1]+m, i[0]+n]
                        s += 1
                if h / s > 50:
                    color_dict['Red'] += 1

        if g_circles is not None:
            g_circles = np.uint16(np.around(g_circles))

            for i in g_circles[0, :]:
                if i[0] > size[1] or i[1] > size[0] or i[1] > size[0]*bound:
                    continue

                h, s = 0.0, 0.0
                for m in range(-r, r):
                    for n in range(-r, r):

                        if (i[1]+m) >= size[0] or (i[0]+n) >= size[1]:
                            continue
                        h += maskg[i[1]+m, i[0]+n]
                        s += 1
                if h / s > 100:
                    color_dict['Green'] += 1

        if y_circles is not None:
            y_circles = np.uint16(np.around(y_circles))

            for i in y_circles[0, :]:
                if i[0] > size[1] or i[1] > size[0] or i[1] > size[0]*bound:
                    continue

                h, s = 0.0, 0.0
                for m in range(-r, r):
                    for n in range(-r, r):

                        if (i[1]+m) >= size[0] or (i[0]+n) >= size[1]:
                            continue
                        h += masky[i[1]+m, i[0]+n]
                        s += 1
                if h / s > 50:
                    color_dict['Yellow'] += 1
        return color_dict
