import cv2
import pyapriltags as apt
from typing import List

import calibrate
import Camera


class MiniTags:
    """
    The MiniTags class was designed to be fairly portable and simple to use.
    It handles camera calibration, tag detection, and ESP32 communication

    get_tag() returns a list of tags currently in frame. You will care most about the following properties:
    tag.tag_id: int, The ID of the tag. FRC tags are numbered 1 thru 16
    center: numpy.ndarray, The center of the tag measured in pixel coordinates
    pose_R: Optional[numpy.ndarray], The orientation matrix of the tag
    pose_t: Optional[numpy.ndarray], Vector from the camera to the tag, in meters
    """

    def __init__(self,
                 camera_type: int = Camera.PICAMERA,
                 camera_resolution: tuple = (480, 240),
                 camera_matrix: tuple = None,
                 checker_size: float = 0.02261,
                 tag_size: float = 0.0405,
                 tag_type: str = "tag36h11",
                 ) -> None:
        """
        :param camera_type: camera.PICAMERA or camera.OPENCV
        :param camera_resolution: tuple of desired image resolution. Your camera may not support all/arbitrary resolutions. Recalibrate camera after changing resolutions
        :param camera_matrix: tuple of camera matrix if you do not want to use auto-calibration. (focal pixels x, focal pixels y, image center x, image center y)
        :param checker_size: the edge length of your calibration checkers in meters.
        :param tag_size: the edge length of your tag size. 0.0405m For 1/4 scale 36h11 tags and 0.0807m For 1/2 scale 36h11 tags
        :param tag_type: the apriltags string that identifies the tags to be used. FRC uses "tag36h11"
        """

        self.camera_resolution = camera_resolution
        self.camera_matrix = camera_matrix

        # This is exclusively set by calibration code
        self.raw_matrix = None
        self.camera_distortion = None
        self.camera_optimizer = None

        self.checker_size = checker_size
        self.tag_size = tag_size
        self.tag_standard = tag_type
        self.camera = Camera.Camera(self.camera_resolution, camera_type)

        # See https://github.com/WillB97/pyapriltags
        self.detector = apt.Detector(families=self.tag_standard, nthreads=1, quad_decimate=1.0)
        txt_camera = "USB CAMERA" if self.camera.capture_method == Camera.OPENCV else "RASPBERRY PI CAMERA"
        print("MiniFRC tags starting up using %s..." % txt_camera)

    def calibrate(self) -> bool:
        """
        calibrate() attempts to load 'camera.npy' or generate it if it doesn't exist.
        There are a couple levels of calibration available depending on what you need:

        1. Manual
        - Simply make a manual entry to self.camera_matrix instead of None
        - Good solution if you don't have a wide-angle camera lense (little distortion)
        - See comment where self.camera_matrix is defined on details

        2. Automatic
        - Takes some more setup: see the comment in calibrate_camera within calibrate.py
        - Great for correcting distortion in fisheye/wide-angle lenses
        - Only needs to be done once, a camera.npy file is created to store the settings
        """
        if self.camera_matrix is None:
            ret, matrix, distortion = calibrate.load_camera_properties()
            if not ret:
                print("Running Camera Calibration to generate camera.npy... Please Read calibrate.py for more info...")
                ret = calibrate.calibrate_camera(self.camera, self.checker_size)
                if ret:
                    self.calibrate()
            else:
                self.raw_matrix = matrix
                self.camera_matrix = (matrix[0][0], matrix[1][1], matrix[0][2], matrix[1][2])
                self.camera_distortion = distortion
                self.camera_optimizer, roi = cv2.getOptimalNewCameraMatrix(matrix,
                                                                           distortion,
                                                                           self.camera_resolution,
                                                                           1,
                                                                           self.camera_resolution)
            return True

    def get_tags(self) -> List[apt.Detection]:
        color_image = self.camera.read()
        if self.camera_optimizer is not None:
            color_image = cv2.undistort(color_image,
                                        self.raw_matrix,
                                        self.camera_distortion,
                                        None,
                                        self.camera_optimizer)
        gray_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)
        if self.camera_matrix is not None:
            detections = self.detector.detect(gray_image,
                                              estimate_tag_pose=True,
                                              camera_params=self.camera_matrix,
                                              tag_size=self.tag_size)
        else:
            detections = []
            print("No camera matrix! Set manually or call calibrate() before detecting tags!")
        return detections
