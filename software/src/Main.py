# -*- coding: utf-8 -*-
"""
Created on --- May  ------- 2015

@author: javierih
"""

import time
import cv2
from Planner import Planner
from Camera import Camera
from Control import Control

camera = Camera()

address='127.0.0.1'
port=19999

controller = Control(address, port)

camera.process_image()

image_bin = camera.getImage()

start = [0,0]

start[0] = camera.robot[0][0]
start[1] = camera.robot[0][1]
   
cv2.circle(image_bin, tuple(start), 2, 0, 40)    


#cv2.imshow("map", image_bin)

#################################################     PLANNER SETTINGS     ###################
#
#            Planner(self, environment_image,
#                 node_gen_method='random', nodes=200, threshold_neighbors=50,
#                 collision_method='simple', collision_radius=None):
#                     
#        :param environment_image: matrix containing the segmented b/w map
#        :param node_gen_method: method for generating the nodes
#            'random' -> random
#            'Hammersley' -> Hammersley point distribution
#            'Halton' -> Halton point distribution
#        :param nodes: number of nodes generated (before collision checking)
#        :param threshold_neighbors: maximum distance to consider two nodes connected
#        :param collision_method: method to check collision
#            'simple' -> point to obstacle checking
#            'circle' -> circle (robot) to obstacle checking
#            'dilate' -> point to dilated obstacle checking
#        :param collision_radius: in methods using this parameter, this is the radius of the
#            robot
#
##############################################################################################

planner = Planner(image_bin, 'Hammersley', 400, 50, 'dilate', 16)


show = cv2.cvtColor(planner.environment.image, cv2.COLOR_GRAY2BGR)
for point in planner.nodes:
    cv2.circle(show, point, 2, (255, 0, 0), 2)
cv2.drawContours(show, planner.environment.obstacles, -1, (0, 0, 255))

for i, origin in enumerate(planner.nodes):
    for j, end in enumerate(planner.nodes):
        if planner.distance_matrix[i, j] > 0:
            cv2.line(show, origin, end, (255, 255, 0))
cv2.imshow("Connections", show)
#cv2.waitKey(500)
#cv2.destroyAllWindows()

# Ask for input
# Note: second check does not work

# Ask for the initial point and the goal point
print("Los limites del mapa son: ", planner.environment.x_limit, planner.environment.y_limit)

print("Los limites del mapa son: ", planner.environment.x_limit, planner.environment.y_limit)

print("El punto de inicio del robot en pixeles es: ", start[0],", ", start[1])
valid_start = planner.environment.is_valid(tuple(start))

goal = [1,1]
#goal[0] = int(input("Introduzca la coordenada x del punto final:"))
#goal[1] = int(input("Introduzca la coordenada y del punto final:"))

goal = camera.targets()[0][0] # primera lata, xy

valid_goal = planner.environment.is_valid(tuple(goal))

#while goal[0] < 0 or goal[0] > planner.environment.x_limit or goal[1] < 0 or goal[1] > planner.environment.y_limit or valid_goal == False:
#    print("el punto seleccionado no es valido")
#    goal[0] = int(input("Introduzca la coordenada x del punto final (pixeles):"))
#    goal[1] = int(input("Introduzca la coordenada y del punto final (pixeles):"))
#    valid_goal = planner.environment.is_valid(tuple(goal))

# Calculate path
#path, points = planner.find_path_and_simplify(tuple(start), tuple(goal))
path, points = planner.find_path(tuple(start), tuple(goal))

# Draw paths
show = cv2.cvtColor(planner.environment.image, cv2.COLOR_GRAY2BGR)
for i in range(len(path)-1):
    origin = points[path[i]]
    end = points[path[i+1]]
    cv2.line(show, origin, end, (0, 0, 255))
for point in points:
    cv2.circle(show, point, 2, (255, 0, 0), 2)
cv2.circle(show, points[0], 2, (0, 255, 0), 2)
cv2.circle(show, points[len(points)-1], 2, (255, 255, 255), 2)
cv2.imshow("Path", show)

cv2.waitKey(0)


target = [0,0]

path=path[::-1]

for i in path:
    print points[i]        
    
    camera.process_image()

    while(controller.getDistanceToTarget(points[i][0], points[i][1])>0.10):
        camera.process_image()
        controller.turnToTargetAngle (controller.getTargetAngle(points[i][0],points[i][1]))
        controller.robot.move(1,1)
        time.sleep(0.1)
        print controller.getDistanceToTarget(points[i][0], points[i][1])

    print "punto!"
    controller.robot.move(0,0)
    

