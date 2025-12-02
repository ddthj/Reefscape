import numpy as np
import cv2

from Camera import Camera


def load_camera_properties():
    """
    load_camera_properties attempts to read camera.npy
    """
    try:
        with open("camera.npy", "rb") as f:
            matrix = np.load(f)
            distortion = np.load(f)
            return True, matrix, distortion
    except FileNotFoundError:
        print("Could not find camera.npy!")
    except Exception as e:
        print(e)
    return False, None, None


def calibrate_camera(cam: Camera, checker_size) -> bool:
    """
    calibrate_camera creates and saves camera distortion properties to 'camera.npy'
    It will run automatically if camera.npy can't be found, but requires some setup:

    1. Print the pattern from https://github.com/opencv/opencv/blob/4.x/doc/pattern.png
    2. Measure the size of the checker squares. They should be 22.587mm if printed without a margin
        -if your checker size is different, you can change CHECKER_SIZE in MiniTags.py
    3. Place checkerboard on a well-lit flat surface and run this program. Calibration should begin automatically
    4. Move your camera around the checkerboard to capture it at various angles
    5. If the captured data looks good, press 'Enter' to save. Otherwise, press any other key to skip
    5. Repeat until you have 10 good captures
    6. Wait (potentially several minutes) to see the undistorted video feed. If all looks good, press 'ESC' to finish
    7. If undistorted feed looks too crazy, press 'ESC' and then delete the camera.npy file to try again
    """

    # calibration accuracy criteria
    criteria = (cv2.TermCriteria_EPS + cv2.TermCriteria_MAX_ITER, 30, 0.001)

    # this results in a 8x5 matrix below, because we're detecting
    # the black corners of the checkerboard
    checkerboard = (9, 6)

    real3d = np.zeros((checkerboard[0] * checkerboard[1], 3), np.float32)
    real3d[:, :2] = np.mgrid[0:checkerboard[0], 0:checkerboard[1]].T.reshape(-1, 2)
    real3d *= checker_size  # size of square edge. This is what sets your measurement units

    objpoints = []
    imgpoints = []

    while len(imgpoints) < 10:
        frame = cam.read()
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        ret, corners = cv2.findChessboardCorners(gray, checkerboard)
        if ret:
            corners2d = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
            frame = cv2.drawChessboardCorners(frame, checkerboard, corners2d, ret)
            cv2.imshow("img", frame)
            key = cv2.waitKey(100)
            if key == 13:
                objpoints.append(real3d)
                imgpoints.append(corners2d)
                print(len(imgpoints))
            elif key == 27:
                break
        else:
            cv2.imshow("img", frame)
            key = cv2.waitKey(20)
            if key == 27:
                break
    ret, matrix, distortion, r_vecs, t_vecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

    print("Matrix")
    print(matrix)

    print("Distortion")
    print(distortion)

    h, w = frame.shape[:2]
    cameramtx, roi = cv2.getOptimalNewCameraMatrix(matrix, distortion, (w, h), 1, (w, h))

    while True:
        frame = cam.read()
        frame = cv2.undistort(frame, matrix, distortion, None, cameramtx)
        cv2.imshow("img", frame)
        key = cv2.waitKey(20)
        if key == 27:
            break

    cv2.destroyAllWindows()

    with open("camera.npy", "wb") as f:
        np.save(f, matrix)
        np.save(f, distortion)
    print("saved camera.npy!")
    return True
