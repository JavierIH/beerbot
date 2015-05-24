import numpy as np
import cv2


class CalibratedVideoCapture:
    def __init__(self, id, camera_parameters_file):
        # Start device
        self.videoCapture = cv2.VideoCapture
        self.videoCapture.open(id)

        # Load camera parameters
        npz_file = np.load(camera_parameters_file)
        self.cameraMatrix = npz_file['cameraMatrix']
        self.distCoeffs = npz_file['distCoeffs']
        self.newCameraMatrix = npz_file['newCameraMatrix']

    def read(self):
        # Read image from camera
        ret, image = self.videoCapture.read()

        # Undistort image
        dst = cv2.undistort(image, self.cameraMatrix, self.distCoeffs, None, self.newCameraMatrix)

        return ret, dst