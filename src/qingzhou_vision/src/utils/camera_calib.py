
import os
import glob
import cv2 as cv
import numpy as np


def save_coefficients(mtx, dist, path):
    cv_file = cv.FileStorage(path, cv.FILE_STORAGE_WRITE)
    cv_file.write('K', mtx)
    cv_file.write('D', dist)
    cv_file.release()


def load_coefficients(path="calibration_chessboard.yml"):
    cv_file = cv.FileStorage(path, cv.FILE_STORAGE_READ)
    camera_matrix = cv_file.getNode('K').mat()
    dist_matrix = cv_file.getNode('D').mat()

    cv_file.release()
    return [camera_matrix, dist_matrix]


def calibrate_camera(chessboard_path, params_save_path="calibration_chessboard.yml"):
    criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    objp = np.zeros((7*10, 3), np.float32)
    objp[:, :2] = np.mgrid[0:7, 0:10].T.reshape(-1, 2)

    objpoints = []  # 3d point in real world space
    imgpoints = []  # 2d points in image plane.
    images = glob.glob(chessboard_path + "*.jpg")
    for fname in images:
        img = cv.imread(fname)
        gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)

        ret, corners = cv.findChessboardCorners(gray, (7, 10), None)
        if ret == True:
            objpoints.append(objp)
            corners2 = cv.cornerSubPix(
                gray, corners, (11, 11), (-1, -1), criteria)
            imgpoints.append(corners)
            cv.drawChessboardCorners(img, (7, 10), corners2, ret)

    ret, mtx, dist, rvecs, tvecs = cv.calibrateCamera(
        objpoints, imgpoints, gray.shape[::-1], None, None)

    save_coefficients(mtx, dist, params_save_path)


def undistort_image(image, mtx, dist):
    h,  w = image.shape[:2]
    newcameramtx, roi = cv.getOptimalNewCameraMatrix(
        mtx, dist, (w, h), 1, (w, h))

    # undistort
    dst = cv.undistort(image, mtx, dist, None, newcameramtx)
    x, y, w, h = roi
    dst = dst[y:y+h, x:x+w]
    return dst


if __name__ == "__main__":
    mtx, dist = load_coefficients("./config/calibration_chessboar.yml")

    cap = cv.VideoCapture("./SampleVid.mp4")
    while True:
        _, img = cap.read()
        img = cv.resize(img, (1280, 720))
        dst = undistort_image(img, mtx, dist)
        cv.imshow("winwin", dst)

        cv.waitKey(0)
