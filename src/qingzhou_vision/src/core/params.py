from utils.debug import ParamPanel


class ImageParams:
    class Blur:
        kernal = (3, 3)
        signma = 1

    class Canny:
        threshold_low = 20
        threshold_high = 60

    class Warp:
        top_point = (293, 697)
        bottom_point = (11, 849)

    class ROI:
        top = (125, 682)
        bot = (0, 923)

    blur = Blur()
    canny = Canny()
    warp = Warp()
    roi = ROI()

    def __init__(self, debug_mode=False) -> None:
        if debug_mode is True:
            self.panel = ParamPanel()
