from Camera import Camera
from CircleCollisionChecker import CircleCollisionChecker
from Control import Control

__author__ = 'kike'


class Tracker:

    def __init__(self, path, points, goal):
        self.path = path
        self.points = points
        self.target = goal
        self.control = Control()
        return

    def track_robot_pose(self, thresh):
        camera = Camera()
        collision = CircleCollisionChecker((1240, 800), 10)
        last_checkpoint = 0
        while self.current != self.target:
            self.current_pos = camera.get_robot_pose()
            if collision.line_collides(self.current_pos[3], self.points[self.path[last_checkpoint]], self.points[self.path[last_checkpoint+1]]) == False:
                self.control.goToTarget(self.points[self.path[last_checkpoint+1]][0], self.points[self.path[last_checkpoint+1]][1])

            if self.current_pos[0] - self.points[self.path[last_checkpoint+1]][0] < thresh and self.current_pos[1] - self.points[self.path[last_checkpoint+1]][1] < thresh:
                last_checkpoint += 1
