# Imports for reading from the serial port
import serial
import serial.tools.list_ports
from scipy import signal
import numpy
import matplotlib.pyplot

# Imports for sending/receiving controller commands
import requests
from requests.auth import HTTPBasicAuth
import json
import ast

# Import for waiting for the fan to adjust
from time import sleep

# ---IP Address of the Controller---
controllerAddress = '192.168.137.42'

# ---Arduino Serial Port Settings---
ARDUINO_BAUDRATE = 115200
FAN_START_FDC = 1500
BUFFER_SIZE = 1000

# ---Exit Codes---
# 0 - Exited normally due to 'Exit' command from Arduino
# 1 - Exited because no Arduino boards could be found
# 2 - Exited because the serial port closed unexpectedly

# ---List of Arduino Commands---
# Start     - Start streaming data to file
# Stop      - Stop streaming data to file
# Set:fdc:w - Send a command to the controller to set the FDC to fdc. Arduino will wait w seconds for the fan to adjust.
# Exit      - Done data collection, exit program
# Time:dt   - Send the sample time in microseconds dt


# Main data acquisition method
def flow_meter():
    # Create the serial object and set the port, baud rate, and timeout
    ser = serial.Serial()
    ser.port = find_arduino_port()
    ser.baudrate = ARDUINO_BAUDRATE
    ser.timeout = 1

    # Exit if no Arduino devices were found
    if ser.port is None:
        set_speed_adjust(0)
        return 1
    ser.open()

    # Initialize the streaming flag to false
    streaming = False

    # Initialize the data buffers to zeros
    buffer1 = RingBuffer(BUFFER_SIZE)
    buffer1.extend(numpy.zeros(BUFFER_SIZE, dtype='f'))
    buffer2 = RingBuffer(BUFFER_SIZE)
    buffer2.extend(numpy.zeros(BUFFER_SIZE, dtype='f'))

    # Loop index for testing
    iteration = 0

    # Set the fan speed adjust to On and start the fan
    set_speed_adjust(1)
    set_fdc(FAN_START_FDC)

    # Continuously read from the serial port
    while ser.isOpen():
        # Read a line (terminated by '\n') from the serial port, and strip of the trailing '\n'
        line = ser.readline().rstrip()

        # Received a 'Start' command
        if line == 'Start':
            if not streaming:
                print('Received \'Start\' command.')
                streaming = True
            else:
                print('Received \'Start\' command while streaming. Ignoring.')

        # Received a 'Stop' command
        elif line == 'Stop':
            if streaming:
                print('Received \'Stop\' command.')
                streaming = False
            else:
                print('Received \'Stop\' command while not streaming. Ignoring.')
        
        # Received a 'Set' command
        elif 'Set' in line:
            if not streaming:
                fdc = int(line.split(':')[1])
                w = int(line.split(':')[2])
                print('Received \'Set\' command for ' + str(fdc) + '.')
                set_fdc(fdc)
                sleep(w)
            else:
                print('Received \'Set\' command while streaming. Ignoring.')

        # Received a 'Time' command
        elif 'Time' in line:
            if not streaming:
                ts = int(line.split(':')[1])
                print('Received \'Time\' command. Sample time is ' + str(ts) + ' microseconds.')
            else:
                print('Received \'Time\' command while streaming. Ignoring.')

        # Received an 'Exit' command
        elif line == 'Exit':
            if not streaming:
                print('Received \'Exit\' command.')
                set_speed_adjust(0)
                return 0
            else:
                print('Received \'Exit\' command while streaming. Ignoring.')
        
        # Not a command, and currently streaming data
        elif streaming:
            buffer1.extend(int(line.split(',')[0])*numpy.ones(1, dtype='f'))
            buffer2.extend(int(line.split(',')[1])*numpy.ones(1, dtype='f'))
            iteration += 1
            if (iteration % 200) == 0:
                matplotlib.pyplot.clf()
                matplotlib.pyplot.plot(buffer1.get())
                matplotlib.pyplot.plot(buffer2.get())
                matplotlib.pyplot.pause(0.0001)

    print('Serial port unexpectedly closed.')
    set_speed_adjust(0)
    return 2


class RingBuffer():
    # A 1D ring buffer using numpy arrays
    def __init__(self, length):
        self.data = numpy.zeros(length, dtype='f')
        self.index = 0

    def extend(self, x):
        # Adds array x to ring buffer
        x_index = (self.index + numpy.arange(x.size)) % self.data.size
        self.data[x_index] = x
        self.index = x_index[-1] + 1

    def get(self):
        # Returns the first-in-first-out data in the ring buffer
        idx = (self.index + numpy.arange(self.data.size)) %self.data.size
        return self.data[idx]


def get_flow_rate(buffer1, buffer2):
    numpy.argmax(signal.correlate(normalize(buffer1.get()), normalize(buffer2.get()))) - (BUFFER_SIZE - 1)


# Method to normalize arrays
def normalize(array):
    normalized_array = array - numpy.mean(array)
    normalized_array /= max(abs(normalized_array))
    return normalized_array


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

# Method to set the fan duty cycle on the controller
def set_fdc(fdc):
    if 1200 <= fdc <= 12000:
        command = 'http://' + controllerAddress + '/cgi-bin/bc2-cgi?json=' + '{ \'object_no\' : 21 , \'boiler_no\' : ' \
                                                                             '0 , \'FanDuty\' : ' + str(
            fdc) + ', \'Update\' : ' + str(get_update_number('FanDuty')) + '}'
        requests.get(command, auth=HTTPBasicAuth('admin', 'IBC-c3h8'), timeout=1)
    else:
        print(str(fdc) + ' is outside the fan operating range of 1200 to 12000. set_fdc failed!')

# Method to set the fan speed adjust (allows duty cycle override) on the controller
def set_speed_adjust(mode):
    command = 'http://' + controllerAddress + '/cgi-bin/bc2-cgi?json=' + '{ \'object_no\' : 21 , \'boiler_no\' : 0 , ' \
                                                                         '\'SpeedAdjust\' : ' + str(
        mode) + ', \'Update\' : ' + str(get_update_number('SpeedAdjust')) + '}'
    requests.get(command, auth=HTTPBasicAuth('admin', 'IBC-c3h8'), timeout=1)

# Method to get the rpm from the controller
def get_rpm():
    command = 'http://' + controllerAddress + '/cgi-bin/bc2-cgi?json=' + '{ \'object_no\' : 100 , \'object_request\' ' \
                                                                         ': 43 ,\'boiler_no\' : 0 }'
    request = requests.get(command, auth=HTTPBasicAuth('admin', 'IBC-c3h8'), timeout=1)
    # Parse the json sent back by the controller
    try:
        d = ast.literal_eval(json.dumps(request.json()).replace('false', 'False').replace('true', 'True'))
        rpm = d['FanSpeed']
    except ValueError:
        print('get_rpm() failed!')
        rpm = None
    return rpm

# Method to get the update number for a given parameter. 'param' could be a list of parameters if needed in the future.
def get_update_number(param):
    update_number = 0

    if 'HeatOut' in param:
        update_number |= (2 ** 0)

    if 'FanTarget' in param:
        update_number |= (2 ** 1)

    if 'FanDuty' in param:
        update_number |= (2 ** 2)

    if 'SpeedAdjust' in param:
        update_number |= (2 ** 7)

    if 'FanTest' in param:
        update_number |= (2 ** 8)

    if 'VentFactor' in param:
        update_number |= (2 ** 4)

    if 'UlPurgeDisable' in param:
        update_number |= (2 ** 11)

    if 'VentFactorDisable' in param:
        update_number |= (2 ** 12)

    if 'BlockedVentDisable' in param:
        update_number |= (2 ** 13)

    if 'AltitudeDisable' in param:
        update_number |= (2 ** 14)

    if 'AirAdjust' in param:
        update_number |= (2 ** 9)

    if 'ZeroAdjust' in param:
        update_number |= (2 ** 10)

    if 'HeatCalls' in param:
        update_number |= (2 ** 18)

    return update_number

# This gets called when this is run as a script
if __name__ == '__main__':
    flow_meter()
