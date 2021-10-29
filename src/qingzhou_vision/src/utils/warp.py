import numpy as np
import cv2 as cv


def get_trapezoid_pts(image, pt_top, pt_bot):
    h, w = image.shape[:2]
    pt_top = (w*pt_top[0]//1000, h*pt_top[1]//1000)
    pt_bot = (w*pt_bot[0]//1000, h*pt_bot[1]//1000)
    pt1 = pt_top
    pt2 = (w-pt_top[0], pt_top[1])
    pt3 = pt_bot
    pt4 = (w-pt_bot[0], pt_bot[1])
    return pt1, pt2, pt3, pt4


def get_warp(image, pt_top, pt_bot):
    h, w, _ = image.shape

    pts = get_trapezoid_pts(image, pt_top, pt_bot)

    pts1 = np.float32(pts)
    pts2 = np.float32([[0, 0], [w, 0], [0, h], [w, h]])
    matrix = cv.getPerspectiveTransform(pts1, pts2)
    warp = cv.warpPerspective(image, matrix, (w, h))
    return warp
