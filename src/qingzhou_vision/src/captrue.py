import cv2 as cv
from utils.camera import gstreamer_pipeline, load_coefficients, undistort_image
from utils.debug import ImageSender

if __name__ == '__main__':

    # cap = cv.VideoCapture("/home/yzu/catkin_ws/SampleVid.mp4")
    print("1")
    cap = cv.VideoCapture(gstreamer_pipeline())

    mtx, dist = load_coefficients(
        "/home/yzu/catkin_ws/src/qingzhou_vision/config/calibration_chessboard.yml")

    sender = ImageSender("192.168.2.109", 8989)

    counter = 0

    while True:

        _, image = cap.read()
        if image.shape != (720, 1280):
            image = cv.resize(image, (1280, 720))

        # 去除畸变
        undistort = cv.resize(undistort_image(image, mtx, dist), (640, 360))

        sender.send(undistort)

        capture = input("cap")

        if capture == 'y':
            cv.imwrite("/home/yzu/catkin_ws/images/cap" + str(counter) + ".png", undistort)

        counter += 1
