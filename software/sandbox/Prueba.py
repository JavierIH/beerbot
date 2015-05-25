import cv2
from Camera import Camera

__author__ = 'kike'


def main():
    camera = Camera(1, './CameraCalibration/logitech/calibration_image.npz')
    while 1:
        image = camera.get_binary_map()
        cv2.imshow('mapa', image)
        pose = camera.get_robot_pose()
        k = cv2.waitKey(1) & 0xFF
        if k == ord('q'):
            break

        print('posicion: ', pose[0], pose[1])
        print('orientacion: ', pose[2])


if __name__ == '__main__':
    main()