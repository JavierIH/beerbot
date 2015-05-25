import cv2
from Camera import Camera

__author__ = 'kike'


def main():
    camera = Camera()
    while 1:
        image = camera.get_binary_map()
        cv2.imshow('mapa', image)
        pose = camera.get_robot_pose()
        print('posicion: ', pose[0], pose[1])
        print('orientacion: ', pose[2])


if __name__ == '__main__':
    main()