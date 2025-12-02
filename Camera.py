
OPENCV = 0
PICAMERA = 1


class Camera:
    """
    This class was added in order to support both USB webcams and the raspberry pi camera system
    You should not need to modify it.
    """

    def __init__(self, resolution, capture_method=OPENCV):
        self.capture_method = capture_method

        if capture_method == OPENCV:
            from cv2 import VideoCapture, CAP_PROP_FRAME_WIDTH, CAP_PROP_FRAME_HEIGHT
            self.camera = VideoCapture(0)
            self.camera.set(CAP_PROP_FRAME_WIDTH, resolution[0])
            self.camera.set(CAP_PROP_FRAME_HEIGHT, resolution[1])

        elif capture_method == PICAMERA:
            from picamera2 import Picamera2  # remove if you aren't using picamera
            self.camera = Picamera2()
            config = self.camera.create_preview_configuration(main={"format": 'XRGB8888', "size": resolution})
            self.camera.configure(config)
            self.camera.start()

    def read(self):
        if self.capture_method == OPENCV:
            ret, image = self.camera.read()
            if ret:
                return image
            else:
                raise UserWarning("OpenCV could not read from a USB camera. Did you mean to use pycamera?")

        elif self.capture_method == PICAMERA:
            image = self.camera.capture_array()
            return image
