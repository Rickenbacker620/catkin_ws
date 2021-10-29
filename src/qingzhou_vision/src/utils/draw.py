import cv2
import random


def plot_one_box(x, im, color=None, label=None, line_thickness=3):
    # Plots one bounding box on image 'im' using OpenCV
    assert im.data.contiguous, 'Image not contiguous. Apply np.ascontiguousarray(im) to plot_on_box() input image.'
    tl = line_thickness or round(
        0.002 * (im.shape[0] + im.shape[1]) / 2) + 1  # line/font thickness
    color = color or [random.randint(0, 255) for _ in range(3)]
    c1, c2 = (int(x[0]), int(x[1])), (int(x[2]), int(x[3]))
    cv2.rectangle(im, c1, c2, color, thickness=tl, lineType=cv2.LINE_AA)
    if label:
        tf = max(tl - 1, 1)  # font thickness
        t_size = cv2.getTextSize(label, 0, fontScale=tl / 3, thickness=tf)[0]
        c2 = c1[0] + t_size[0], c1[1] - t_size[1] - 3
        cv2.rectangle(im, c1, c2, color, -1, cv2.LINE_AA)  # filled
        cv2.putText(im, label, (c1[0], c1[1] - 2), 0, tl / 3,
                    [225, 255, 255], thickness=tf, lineType=cv2.LINE_AA)


def draw_points(image, *pts, color=(0, 0, 255)):
    for pt in pts:
        cv2.circle(image, pt, 4, color, 10)


def draw_lines(image, *lines, color=(0, 0, 255), thickness=2):
    if lines is None or len(lines) == 0:
        return image
    else:
        for line in lines:
            cv2.line(image, (line[0], line[1]),
                     (line[2], line[3]), color, thickness)
        return image


def draw_slope(image, slope):
    copy = image.copy()
    h, w = image.shape[:2]
    y_center = h//2
    x_center = w//2
    dx = y_center // slope
    pt1 = (int(x_center-dx), 0)
    pt2 = (int(x_center+dx), h)
    return draw_lines(copy, (*pt1, *pt2))
