import numpy as np
import cv2
from CameraCalibration.CalibratedVideoCapture import CalibratedVideoCapture


__author__ = 'kfrodicio'

class Camera2:
    def __init__(self, id, camera_parameters_file):
        self.camera = CalibratedVideoCapture()
        self.camera.open(id, camera_parameters_file)

        # Thresholds:
        self.color_threshold = { 'blue' : [(95, 110, 55), (125, 255, 243)],
                                 'green': [(35, 47, 70),  (84, 255, 200)],
                                 'orange':[(0, 170, 55),  (32, 255, 200)],
                                 'brown': [(5, 5, 65),  (55, 89, 89)] }
        self.black_threshold = 20

        self.obstacles = None
        self.obstacles_without_targets = None
        self.targets = None
        self.robot = None

    def process_image(self):
        ret, image = self.camera.read()
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        green_mask = self.color_segmentation(hsv, 'green')
        blue_mask = self.color_segmentation(hsv, 'blue')

        green_contours = self.process_mask(cv2.bitwise_not(green_mask))
        blue_contours = self.process_mask(blue_mask)

        # Segment robot
        if len(blue_contours) > 0:
            blue_center = self.contour_center(blue_contours[0])
            robot_contour = None
            robot_center = None
            self.obstacles = []

            for contour in green_contours:
                if cv2.pointPolygonTest(contour, blue_center, False) >= 0:
                    robot_contour = self.simplify_contour(contour)
                    if robot_contour != None:
                        robot_center = self.contour_center(robot_contour)
                        orient_vector = np.array(blue_center) - np.array(robot_center)
                        orientation = np.arctan2(orient_vector[1], orient_vector[0])
                        self.robot = (robot_center, orientation)
                else:
                    self.obstacles.append(contour)

            # Targets (circles):
            self.targets = []
            self.obstacles_without_targets = []
            for obstacle in self.obstacles:
                target = self.get_circle(obstacle)
                if target is not None:
                    self.targets.append(target)
                else:
                    self.obstacles_without_targets.append(obstacle)


            # Debug image
            show = image.copy()
            if robot_center != None and robot_contour != None:

                cv2.circle(show, robot_center,3, (255, 0, 255), 2)
                cv2.drawContours(show, [robot_contour], -1, (255, 0, 255), 2)

                cv2.line(show, robot_center, tuple(robot_center+orient_vector*3), (255, 255, 0), 2)

            if self.obstacles_without_targets:
                cv2.drawContours(show,self.obstacles_without_targets, -1, (0, 255, 255), 2)

            if self.targets:
                for (x, y), r in self.targets:
                    cv2.circle(show, (int(x), int(y)), int(r), (255, 0, 0), 2)

            cv2.imshow("robot", show)

    def color_segmentation(self, hsv_image, color):
        """
            Input: hsv image, name of the color
            Output: blue contours (filtered by size)
        """
        thresholds = self.color_threshold[color]
        mask = cv2.inRange(hsv_image, thresholds[0], thresholds[1])
        # cv2.imshow(color, mask)
        return mask

    def black_segmentation(self, hsv_image):
        """
            Input: hsv image
            Output: black contours (filtered by size)
        """
        dummy, thresholded_mask = cv2.threshold(hsv_image[:,:,2], 50, 255, cv2.THRESH_BINARY_INV) #+cv2.THRESH_OTSU)
        adaptive_threshold_mask = cv2.adaptiveThreshold(hsv_image[:,:,2],255,cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY_INV,11,2)
        black_mask = cv2.bitwise_and(thresholded_mask, adaptive_threshold_mask)
        # cv2.imshow("black", black_mask)
        return black_mask

    def process_mask(self, mask, kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))):
        """
            Input: binary mask, kernel size for closing
            Output: filtered contours contained in image
        """
        filtered_mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        contours, dummy = cv2.findContours(filtered_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        contours_filtered = []
        for contour in contours:
            if len(contour) > 20 and cv2.contourArea(contour) > 150:
                contours_filtered.append(contour)

        contours_filtered.sort(key=lambda x: cv2.contourArea(x))
        contours_filtered.reverse()

        return contours_filtered

    def simplify_contour(self, contour):
        perimeter = cv2.arcLength(contour,True)
        approx = cv2.approxPolyDP(contour,0.12*perimeter,True)

        if len(approx) == 4 and cv2.isContourConvex(approx):
            return approx
        else:
            return None

    def contour_center(self, contour):
        M = cv2.moments(contour)
        cx = int(M['m10']/M['m00'])
        cy = int(M['m01']/M['m00'])
        return (cx, cy)

    def dist(self, a, b):
        return np.sqrt((a[0]-b[0])**2 + (a[1]-b[1])**2)

    def get_circle(self, contour):
        # Compare max and min enclosing circles to check that the contour is circular
        ( center, r) = cv2.minEnclosingCircle(contour)
        (x,y),(h, w),angle = cv2.fitEllipse(contour)

        if abs(h - w) < 30:
            # Ellipse is a circle
            if self.dist(center, (x + h/2, y + h/2) ) < 30:
                # Centers are close
                if abs(r - (h + w) / 4) < 20:
                    # Similar radius
                    return center, r

        return None



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
        cv2.imshow("black_threshold", thresholded_mask)
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


def main():
    camera = Camera2(1, './CameraCalibration/logitech/calibration_image.npz')
    while 1:
        #x, y, orientation, contours= camera.get_robot_pose()
        camera.process_image()
        k = cv2.waitKey(1) & 0xFF
        if k == ord('q'):
            break

        #print('posicion: ', x, y)
        #print('orientacion: ', orientation)


if __name__ == '__main__':
    main()