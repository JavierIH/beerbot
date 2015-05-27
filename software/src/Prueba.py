from Camera import Camera

__author__ = 'kike'


def main():
    camera = Camera()
    while 1:
        image = camera.get_binary_map()
        imshow('mapa binarizado', image)
        x, y, orientation, contours = camera.get_robot_pose()
        print('orientacion: ', orientation)