import serial
import serial.tools.list_ports

ARDUINO_BAUDRATE = 115200

# ---List of Arduino Commands---
# Start     - Start streaming data to file
# Stop      - Stop streaming data to file
# Set:fdc:w - Send a command to the controller to set the FDC to fdc. Arduino will wait w seconds for the fan to adjust.
# Exit      - Done data collection, exit program
# Time:ts   - Send the sample time in microseconds


# Arduino Port class
class ArduinoPort:
    # Constructor
    def __init__(self):
        self.arduino_serial = serial.Serial()
        self.arduino_serial.port = find_arduino_port()
        self.arduino_serial.baudrate = ARDUINO_BAUDRATE
        self.arduino_serial.timeout = 1

        if self.arduino_serial.port is not None:
            self.arduino_serial.open()
        else:
            self.arduino_serial = None

    # Method to read a line from the Arduino
    def read_line(self):
        return self.arduino_serial.readline().rstrip()

    def write_line(self, hdc):
        self.arduino_serial.write(chr(hdc))

    # Checks if the serial port is still open
    def is_open(self):
        return self.arduino_serial.isOpen()


# Method to find the serial port the Arduino is connected to
def find_arduino_port():
    # Get a list of ports that have 'Arduino' in their description
    arduino_ports = [p.device for p in serial.tools.list_ports.comports() if 'Arduino' in p.description]

    # Found multiple Arduino devices
    if len(arduino_ports) > 1:
        print('Multiple Arduino devices were found:')
        for number, name in enumerate(arduino_ports):
            print('\t' + name + '\t(' + str(number) + ')')
        arduino_port = arduino_ports[int(input('Enter the number of the port you want to use: '))]

    # Found one Arduino device
    elif len(arduino_ports) == 1:
        print('Arduino device found at ' + arduino_ports[0] + '.')
        arduino_port = arduino_ports[0]

    # Found no Arduino devices
    else:
        print('No Arduino devices were found!')
        arduino_port = None

    return arduino_port
