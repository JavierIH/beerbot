import numpy as np
import cv2


class CalibratedVideoCapture:
    def __init__(self, id, camera_parameters_file):
        # Start device
        self.videoCapture = cv2.VideoCapture()
        if not self.videoCapture.open(id):
            print "Could not open device"

        # Load camera parameters
        npz_file = np.load(camera_parameters_file)
        self.cameraMatrix = npz_file['cameraMatrix']
        self.distCoeffs = npz_file['distCoeffs']
        self.newCameraMatrix = npz_file['newCameraMatrix']
        self.roi = npz_file['roi']

    def read(self):
        # Read image from camera
        success, image = self.videoCapture.read()

        if not success:
            print "Could not read image"
            return False, None

        # Undistort image
        dst = cv2.undistort(image, self.cameraMatrix, self.distCoeffs, None, self.newCameraMatrix)

        # crop the image
        x,y,w,h = self.roi
        dst = dst[y:y+h, x:x+w]

        return success, dst


if __name__ == '__main__':

    cap = CalibratedVideoCapture(1, './logitech/calibration_image.npz')

    while True:
        dummy, frame = cap.read()
        cv2.imshow("Corrected", frame)
        k = cv2.waitKey(30) & 0xFF
        if k == ord('q'):
            break

    cv2.destroyAllWindows()



