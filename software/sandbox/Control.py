# -*- coding: utf-8 -*-
"""

"""
import time
import vrep
import numpy as np
from Camera import Camera
from Beerbot import Beerbot


class Control:
    def __init__(self, port, baudrate):
        self.camera = Camera()
        self.robot = Beerbot()
        print ('Program started')
        self.robot.connect(port,baudrate)
        #if self.clientID!=-1:
        #    print "Conexion establecida"
        #else:
        #    print "Conexion con el simulador fallida, reinicia el simulador"
        #error,self.motorFL=vrep.simxGetObjectHandle(self.clientID,'Pioneer_p3dx_rightMotor',vrep.simx_opmode_oneshot_wait)
        #error,self.motorFR=vrep.simxGetObjectHandle(self.clientID,'Pioneer_p3dx_leftMotor',vrep.simx_opmode_oneshot_wait)
        #error,self.camera=vrep.simxGetObjectHandle(self.clientID,'Vision_sensor',vrep.simx_opmode_oneshot_wait)
        #error,self.robot=vrep.simxGetObjectHandle(self.clientID,'Pioneer_p3dx',vrep.simx_opmode_oneshot_wait)

    def getRobotPosition(self):
        "get the position of the robot"
        x, y, orientation = self.camera.get_robot_pose()
        #time.sleep(0.1)
        return x, y

    def getRobotOrientation(self):
        "get the orientation of the robot"
        x, y, orientation = self.camera.get_robot_pose()
        return orientation

    def printRobotLocation(self):
        "print the position and orientation of the robot (x, y, yaw)"
        pos_x, pos_y, orientation = self.camera.get_robot_pose()
        yaw='%.2f'%(orientation)
        x='%.2f'%(pos_x)
        y='%.2f'%(pos_y)
        print "Position: (", x,",", y, ")", "\nYaw:", yaw

    def getDistanceToTarget (self, x_target, y_target):
        "gets the distance from the robot to a given target"
        x_robot, y_robot, angle_robot = self.getRobotPosition()
        return np.sqrt((x_target-x_robot)**2+(y_target-y_robot)**2)

    def getDifferenceToTargetAngle (self, angle):
        "Gets the difference between the actual orientation and a given orientation"
        difference = angle-self.getRobotOrientation()
        if difference < -180:
            difference+=360
        elif difference > 180:
            difference-=360
        return difference

    def getTargetAngle (self, x_target, y_target):
        "Gets the angle to a point target"
        x_robot, y_robot = self.getRobotPosition()

        x_gap=x_target-x_robot
        y_gap=y_target-y_robot
        l_gap=np.sqrt(x_gap**2+y_gap**2)

        angle=np.arccos(x_gap/l_gap)*180/3.14159265
        if y_gap < 0:
            angle=-angle
        return angle

    def turnToTargetAngle (self, target_angle):
        "Orientates robot to a given angle in the range from -180 to 180"
        while np.abs(self.getDifferenceToTargetAngle(target_angle)) > 1:
            amp = self.getDifferenceToTargetAngle(target_angle)
            left_motor=amp/20
            right_motor=-amp/20
            self.robot.move(left_motor,right_motor)
            #print "Distancia al angulo objetivo: ", getRobotOrientation()
        self.robot.move(0,0)
        return 0

    def goToTarget (self, target_x,target_y):
        while(self.getDistanceToTarget(target_x, target_y)>0.10):
            self.turnToTargetAngle (self.getTargetAngle(target_x,target_y))
            self.robot.move(1,1)
            time.sleep(0.2)
            print self.getDistanceToTarget(target_x, target_y)

        print "punto!"
        self.robot.move(0,0)

    def getImage (self):
        #errorCode, resolution, image=vrep.simxGetVisionSensorImage(self.clientID, self.camera, 0, vrep.simx_opmode_streaming)
        #print "pillando imagen"
        #time.sleep(1) #necesario
        errorCode, resolution, image=vrep.simxGetVisionSensorImage(self.clientID, self.camera, 0, vrep.simx_opmode_oneshot_wait)
        im=np.array(image, dtype=np.uint8)
        im.resize([resolution[1], resolution[0],3])
        im=np.flipud(im)
        return im


if __name__ == '__main__':
    #moveRobot(-0.5,0.5);4

    import cv2

    address='127.0.0.1'
    port=19999
    simulator = Simulator(address,port)

    vision = simulator.getImage()

    cv2.imshow("image", vision)
    cv2.waitKey(500);
