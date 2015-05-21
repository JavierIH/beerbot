import serial

class RobotRemoteInterface:

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


    def sendChannelA(self, char):
        try:
            self.serialPort.write( 'A' + char)
        except AttributeError, e:
            print 'Not connected: [' + str(e) + ']'

    def sendChannelB(self, char):
        try:
            self.serialPort.write( 'B' + char)
        except AttributeError, e:
            print 'Not connected: [' + str(e) + ']'

    def sendChannelC(self, char):
        try:
            self.serialPort.write( 'C' + char)
        except AttributeError, e:
            print 'Not connected: [' + str(e) + ']'



# If the script is run directly, this example is executed:
if __name__ == "__main__":
	import time as t

	interface = RobotRemoteInterface()
	interface.connect("/dev/ttyACM0", 57600)
	interface.sendChannelA('r')
	interface.sendChannelB( 't')
	t.sleep(1)
	interface.sendChannelC('y')
	t.sleep(1)
	interface.sendChannelB( 't')
	t.sleep(1)
	interface.sendChannelC('y')