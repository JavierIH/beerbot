import numpy as np
import cv2

__author__ = 'kfrodicio'


class Camera:
    def __init__(self):
        pass

    def get_binary_map(self):
        image = cv2.imread('prueba4.jpg', cv2.CV_LOAD_IMAGE_COLOR)
        green = cv2.inRange(cv2.cvtColor(image, cv2.COLOR_BGR2HSV), np.array([43, 40, 66], np.uint8), np.array([74, 255, 255], np.uint8))
        brown = cv2.inRange(cv2.cvtColor(image, cv2.COLOR_BGR2HSV), np.array([5, 5, 65], np.uint8), np.array([55, 89, 89], np.uint8))
        blue = cv2.inRange(cv2.cvtColor(image, cv2.COLOR_BGR2HSV), np.array([111, 75, 127], np.uint8), np.array([125, 160, 243], np.uint8))
        black = cv2.inRange(cv2.cvtColor(image, cv2.COLOR_BGR2HSV), np.array([110, 0, 74], np.uint8), np.array([177, 60, 116], np.uint8))
        bin_env = cv2.bitwise_or(green, brown)
        bin_env = cv2.bitwise_or(bin_env, blue)
        bin_env = cv2.bitwise_or(bin_env, black)
        bin_env = cv2.dilate(bin_env, np.ones((3, 3), np.uint8), iterations=2)
        bin_env = cv2.erode(bin_env, np.ones((3, 3), np.uint8), iterations=2)
        image_bin = 255 - bin_env
        return image_bin

    def get_robot_pose(self):
        image = cv2.imread('prueba4.jpg', cv2.CV_LOAD_IMAGE_COLOR)
        blue = cv2.inRange(cv2.cvtColor(image, cv2.COLOR_BGR2HSV), np.array([111, 75, 127], np.uint8), np.array([125, 160, 243], np.uint8))
        black = cv2.inRange(cv2.cvtColor(image, cv2.COLOR_BGR2HSV), np.array([110, 0, 74], np.uint8), np.array([177, 60, 116], np.uint8))
        robot = cv2.bitwise_or(blue, black)
        robot = cv2.erode(robot, np.ones((3, 3), np.uint8), iterations=2)
        robot = cv2.dilate(robot, np.ones((3, 3), np.uint8), iterations=4)
        contours, hierarchy = cv2.findContours(robot, 1, 2)
        cnt = contours[0]
        (x, y), radius = cv2.minEnclosingCircle(cnt)
        orientation = self.get_orientation(blue, black)
        orientation = 360 - orientation
        return int(x), int(y), orientation

    def get_goal_pos(self):
        image = cv2.imread('prueba2.jpg', cv2.CV_LOAD_IMAGE_COLOR)
        red = cv2.inRange(cv2.cvtColor(image, cv2.COLOR_BGR2HSV), np.array([0, 49, 83], np.uint8), np.array([21, 255, 190], np.uint8))
        goal = cv2.erode(red, np.ones((3, 3), np.uint8), iterations=2)
        goal = cv2.dilate(goal, np.ones((3, 3), np.uint8), iterations=2)
        contours, hierarchy = cv2.findContours(goal, 1, 2)
        cnt = contours[0]
        (x, y), radius = cv2.minEnclosingCircle(cnt)
        return int(x), int(y)

    def get_orientation(self, blue, black):
        blue = cv2.erode(blue, np.ones((3, 3), np.uint8), iterations=2)
        blue = cv2.dilate(blue, np.ones((3, 3), np.uint8), iterations=5)
        black = cv2.erode(black, np.ones((3, 3), np.uint8), iterations=2)
        black = cv2.dilate(black, np.ones((3, 3), np.uint8), iterations=5)
        contours_a, hierarchy_a = cv2.findContours(blue, 1, 2)
        contours_n, hierarchy_n = cv2.findContours(black, 1, 2)
        cnt_a = contours_a[0]
        cnt_n = contours_n[0]
        rect = cv2.minAreaRect(cnt_a)
        (x_a,y_a), radius_a = cv2.minEnclosingCircle(cnt_a)
        (x_n,y_n), radius_n = cv2.minEnclosingCircle(cnt_n)
        ang = int(rect[2]) + 90
        if 0 <= ang <= 45:
            if x_a > x_n:
                return ang
            else:
                ang += 180
                return ang
        elif 45 < ang <= 135:
            if y_a > y_n:
                return ang
            else:
                ang += 180
                return ang
        elif 135 < ang <= 180:
            if x_a < x_n:
                return ang
            else:
                ang += 180
                return ang