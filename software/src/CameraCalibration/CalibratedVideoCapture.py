import numpy as np
import cv2


class CalibratedVideoCapture:
    def __init__(self, id=None, camera_parameters_file=None):
        self.videoCapture = cv2.VideoCapture()
        self.cameraMatrix = None
        self.distCoeffs = None
        self.newCameraMatrix = None
        self.roi = None
        self.opened = False

        if id and camera_parameters_file:
            self.open(id, camera_parameters_file)

    def open(self, id, camera_parameters_file):
        # Start device
            
            
        self.videoCapture = cv2.VideoCapture()

        
        
        
        if not self.videoCapture.open(id):
            print "Could not open device"
            
            
        self.videoCapture.set(cv2.cv.CV_CAP_PROP_FRAME_WIDTH, 1280)
        self.videoCapture.set(cv2.cv.CV_CAP_PROP_FRAME_HEIGHT, 720)        
        
        

        # Load camera parameters
        npz_file = np.load(camera_parameters_file)
        self.cameraMatrix = npz_file['cameraMatrix']
        self.distCoeffs = npz_file['distCoeffs']
        self.newCameraMatrix = npz_file['newCameraMatrix']
        self.roi = npz_file['roi']

        self.opened = True

    def read(self):
        if not self.opened:
            raise Exception("Camera needs to be opened before read")

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
    cap = CalibratedVideoCapture()
    cap.open(1, './logitech/calibration_image.npz')

    while True:
        dummy, frame = cap.read()
        cv2.imshow("Corrected", frame)
        k = cv2.waitKey(30) & 0xFF
        if k == ord('q'):
            break

    cv2.destroyAllWindows()



