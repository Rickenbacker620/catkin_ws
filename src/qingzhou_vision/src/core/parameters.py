class Parameters:
    class Blur:
        kernel = (3, 3)
        sigma = 1

    class Canny:
        threshold_low = 20
        threshold_high = 60

    class Warp:
        top_point = (293, 697)
        bottom_point = (11, 849)

    class ROI:
        top = (125, 682)
        bot = (0, 923)

    class Lane:
        # 霍夫变换参数
        hough_thresh = 20
        hough_min_line_length = 20
        hough_max_line_gap = 100

        # 斑马线参数
        zebra_slope_thresh = 0.1
        zebra_line_count_thresh = 500

        # 左右车道线筛选参数
        abnormal_left_thresh = 0.2
        abnormal_right_thresh = 0.2

        # 偏移滤波参数
        bias_stabilizer_thresh = 30

    blur = Blur()
    canny = Canny()
    warp = Warp()
    roi = ROI()
    lane = Lane()
