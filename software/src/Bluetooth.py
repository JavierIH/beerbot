# -*- coding: utf-8 -*-
"""
Created on Sat May 23 17:32:40 2015

@author: javierih
"""

import serial

class Bluetooth:

    def __init__(self):
        self.serialPort = None

    def connect(self, port, baudrate):
        """
        Connect with the serial port at a certain baudrate
        """
        # Setup serial port
        self.serialPort = serial.Serial()
        self.serialPort.port = port
        self.serialPort.baudrate = baudrate

        # Open port
        self.serialPort.open()

        if not self.serialPort.isOpen():
            raise IOError("Port could not be opened!")
            
    def disconnect(self):
        self.serialPort.close()
        if self.serialPort.isOpen():
            raise IOError("Port could not be closed!")

    def send(self, msg):
        try:
            self.serialPort.write(msg)
        except AttributeError, e:
            print 'Not connected: [' + str(e) + ']'

# If the script is run directly, this example is executed:
if __name__ == "__main__":
	import time as t

	interface = Bluetooth()
	interface.connect("/dev/rfcomm1", 19200)
	interface.send('B' + chr(253))
	