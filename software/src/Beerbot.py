# -*- coding: utf-8 -*-
"""
Created on Sat May 23 17:47:20 2015

@author: javierih
"""

import serial
from Bluetooth import Bluetooth

class Beerbot:

    def __init__(self):
        self.interface = Bluetooth()

    def connect(self, port, baudrate):
        self.interface.connect(port, baudrate)
        
    def disconnect(self):
        self.interface.disconnect()
        
    def move(self, left_speed, right_speed):
        
        if left_speed >= 0:
            left_data = left_speed * 127 
        else: 
            left_data = 255 + left_speed * 127
                
        if right_speed >= 0:
            right_data = right_speed * 127 
        else: 
            right_data = 255 + right_speed * 127
                
        self.interface.send('B'+chr(int(left_data)))
        self.interface.send('C'+chr(int(right_data)))
        
    def openGripper(self):
        self.interface.send('Ab')
                        
    def closeGripper(self):
        self.interface.send('Aa')
        
        



# If the script is run directly, this example is executed:
if __name__ == "__main__":
    import time as t
 
    beerbot = Beerbot()
    beerbot.connect("/dev/rfcomm0", 19200)
    beerbot.move(0,0)
    beerbot.closeGripper()
       
    #beerbot.disconnect()
    print "done!"
