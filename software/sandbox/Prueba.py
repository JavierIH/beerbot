import cv2
from Camera import Camera

__author__ = 'kike'


def main():
    camera = Camera(0, './CameraCalibration/logitech/calibration_image.npz')
    while 1:
        x, y, orientation, contours= camera.get_robot_pose()
        k = cv2.waitKey(1) & 0xFF
        if k == ord('q'):
            break

        print('posicion: ', x, y)
        print('orientacion: ', orientation)


if __name__ == '__main__':
    main()