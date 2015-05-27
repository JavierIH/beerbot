import numpy as np
import cv2
from CameraCalibration.CalibratedVideoCapture import CalibratedVideoCapture


__author__ = 'kfrodicio'


class Camera:
    def __init__(self, id, camera_parameters_file):
        self.camera = CalibratedVideoCapture()
        self.camera.open(id, camera_parameters_file)

        self.black_ub = (180, 60, 70)
        self.black_lb = (0, 0, 0)
        self.blue_ub = (125, 255, 243)
        self.blue_lb = (95, 110, 55)
        self.brown_ub = (55, 89, 89)
        self.brown_lb = (5, 5, 65)
        self.green_ub = (84, 255, 200)
        self.green_lb = (35, 47, 70)

    def black_segmentation(self, hsv_image):
        """
            Input: hsv image
            Output: black contours (filtered by size)
        """
        dummy, thresholded_mask = cv2.threshold(hsv_image[:,:,2], 50, 255, cv2.THRESH_BINARY_INV) #+cv2.THRESH_OTSU)
        adaptive_threshold_mask = cv2.adaptiveThreshold(hsv_image[:,:,2],255,cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY_INV,11,2)
        black_mask = cv2.bitwise_and(thresholded_mask, adaptive_threshold_mask)
        #cv2.imshow("black", black_mask)
        return cv2.morphologyEx(black_mask, cv2.MORPH_CLOSE, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3)) )

    def get_binary_map(self):
        dummy, image = self.camera.read()
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        green = cv2.inRange(hsv, self.green_lb, self.green_ub)
        brown = cv2.inRange(hsv, self.brown_lb, self.brown_ub)
        blue = cv2.inRange(hsv, self.blue_lb, self.blue_ub)
        black = cv2.inRange(hsv, self.black_lb, self.black_ub)
        bin_env = cv2.bitwise_or(green, brown)
        bin_env = cv2.bitwise_or(bin_env, blue)
        bin_env = cv2.bitwise_or(bin_env, black)
        bin_env = cv2.dilate(bin_env, np.ones((3, 3), np.uint8), iterations=2)
        bin_env = cv2.erode(bin_env, np.ones((3, 3), np.uint8), iterations=2)
        image_bin = 255 - bin_env
        return image_bin

    def get_robot_pose(self):
        dummy, image = self.camera.read()
        blue = cv2.inRange(cv2.cvtColor(image, cv2.COLOR_BGR2HSV), self.blue_lb, self.blue_ub)
        black = self.black_segmentation(cv2.cvtColor(image, cv2.COLOR_BGR2HSV))
        robot = cv2.bitwise_or(blue, black)
        robot = cv2.erode(robot, np.ones((3, 3), np.uint8), iterations=2)
        robot = cv2.dilate(robot, np.ones((3, 3), np.uint8), iterations=4)
        cv2.imshow('blue', blue)
        cv2.imshow('black', black)
        green = cv2.inRange(cv2.cvtColor(image, cv2.COLOR_BGR2HSV), self.green_lb, self.green_ub)
        cv2.imshow('green', green)
        contours, hierarchy = cv2.findContours(robot, 1, 2)
        cnt = contours[0]
        (x, y), radius = cv2.minEnclosingCircle(cnt)
        orientation = self.get_orientation(blue, black)
        return int(x), int(y), orientation, contours[0]

    def get_goal_pos(self):
        dummy, image = self.camera.read()
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
        if not contours_a or not contours_n:
            return None
        cnt_a = contours_a[0]
        cnt_n = contours_n[0]
        rect = cv2.minAreaRect(cnt_a)
        (x_a,y_a), radius_a = cv2.minEnclosingCircle(cnt_a)
        (x_n,y_n), radius_n = cv2.minEnclosingCircle(cnt_n)
        ang = int(rect[2]) + 90
        if 0 <= ang <= 45:
            if x_a > x_n:
                return 360-ang
            else:
                ang += 180
                return 360-ang
        elif 45 < ang <= 135:
            if y_a > y_n:
                return 360-ang
            else:
                ang += 180
                return 360-ang
        elif 135 < ang <= 180:
            if x_a < x_n:
                return 360-ang
            else:
                ang += 180
                return 360-ang
