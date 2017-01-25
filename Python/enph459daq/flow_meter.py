# Imports for reading from the Arduino
import arduino

# Imports for signal processing
from scipy import signal
import numpy as np
import matplotlib.pyplot as plt

# Imports for sending/receiving controller commands
import controller as ctrl

# Import for waiting for the fan to adjust
from time import sleep

# ---Settings---
FAN_START_FDC = 1500
BUFFER_SIZE = 2000

# ---Exit Codes---
# 0 - Exited normally due to 'Exit' command from Arduino
# 1 - Exited because no Arduino boards could be found
# 2 - Exited because the serial port closed unexpectedly


# Main flow calculation method
def flow_meter():
    # Create the serial object and set the port, baud rate, and timeout
    arduino_port = arduino.ArduinoPort()

    # Exit if no Arduino devices were found
    if arduino_port.arduino_serial is None:
        ctrl.set_speed_adjust(0)
        return 1

    # Initialize the streaming flag to false
    streaming = False

    # Initialize the data buffers to zeros
    tc1 = DataBuffer(BUFFER_SIZE)
    tc1.push(np.zeros(BUFFER_SIZE, dtype='f'))
    tc2 = DataBuffer(BUFFER_SIZE)
    tc2.push(np.zeros(BUFFER_SIZE, dtype='f'))
    flow_rate = DataBuffer(BUFFER_SIZE)
    flow_rate.push(np.zeros(BUFFER_SIZE, dtype='f'))
    rpm = DataBuffer(BUFFER_SIZE)
    rpm.push(np.zeros(BUFFER_SIZE, dtype='f'))
    tc1_filt = tc1.get()
    tc2_filt = tc2.get()
    tof_delay = 0

    # Filter with default sample rate
    fs = 10.0E6/4000
    cutoff = 20.0
    filter_b, filter_a = signal.butter(2, [cutoff / (fs / 2)], btype='low', analog=False)

    # Loop index for testing
    iteration = 0

    # Figure for plotting
    plt.figure(1)

    # Set the fan speed adjust to On and start the fan
    ctrl.set_speed_adjust(1)
    ctrl.set_fdc(FAN_START_FDC)

    # Continuously read from the serial port
    while arduino_port.is_open():
        # Read a line (terminated by '\n') from the serial port, and strip of the trailing '\n'
        line = arduino_port.read_line()

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
                ctrl.set_fdc(fdc)
                sleep(w)
            else:
                print('Received \'Set\' command while streaming. Ignoring.')

        # Received a 'Time' command
        elif 'Time' in line:
            if not streaming:
                ts = int(line.split(':')[1])
                # Update filter
                fs = 10.0E6/ts
                filter_b, filter_a = signal.butter(2, [cutoff / (fs / 2)], btype='low', analog=False)
                print('Received \'Time\' command. Sample time is ' + str(ts) + ' microseconds.')
            else:
                print('Received \'Time\' command while streaming. Ignoring.')

        # Received an 'Exit' command
        elif line == 'Exit':
            if not streaming:
                print('Received \'Exit\' command.')
                ctrl.set_speed_adjust(0)
                return 0
            else:
                print('Received \'Exit\' command while streaming. Ignoring.')
        
        # Not a command, and currently streaming data
        elif streaming:
            # Update thermocouple data buffers
            try:
                tc1.push(int(line.split(',')[0]) * np.ones(1, dtype='f'))
                tc2.push(int(line.split(',')[1])*np.ones(1, dtype='f'))
            except:
                print('Unparsable thermocouple data! Using previous values.')

            # Might be slow so only update every 50 iterations
            if (iteration % 50) == 0:
                # Update rpm buffer
                rpm.push(ctrl.get_rpm() * np.ones(1, dtype='f'))
                # Filtered thermocouple data
                tc1_filt = signal.filtfilt(filter_b, filter_a, tc1.get())
                tc2_filt = signal.filtfilt(filter_b, filter_a, tc2.get())

                # Delay
                tof_delay = get_flow_rate(tc1_filt, tc2_filt)

                # Unscaled flow rate
                if tof_delay == 0:
                    unscaled_flow_rate = 0
                else:
                    unscaled_flow_rate = 1.0 / tof_delay
                flow_rate.push(unscaled_flow_rate * np.ones(1, dtype='f'))

            iteration += 1
            if (iteration % 200) == 0:

                # Plotting
                plt.figure(1)
                plt.clf()

                # Filtered thermocouple buffers
                plt.subplot(311)
                plt.plot(tc1_filt)
                plt.plot(tc2_filt)

                # Unscaled flow rate
                plt.subplot(312)
                plt.plot(flow_rate.get()[-BUFFER_SIZE / 10:])

                # RPM
                plt.subplot(313)
                plt.plot(rpm.get()[-BUFFER_SIZE / 10:])

                plt.pause(0.0001)

                print(tof_delay)

    print('Arduino port unexpectedly closed.')
    ctrl.set_speed_adjust(0)
    return 2


class DataBuffer:
    # A 1D ring buffer using numpy arrays
    def __init__(self, length):
        self.data = np.zeros(length, dtype='f')
        self.index = 0

    def push(self, x):
        # Adds array x to ring buffer
        x_index = (self.index + np.arange(x.size)) % self.data.size
        self.data[x_index] = x
        self.index = x_index[-1] + 1

    def get(self):
        # Returns the first-in-first-out data in the ring buffer
        idx = (self.index + np.arange(self.data.size)) % self.data.size
        return self.data[idx]


def get_flow_rate(signal1, signal2):
    return np.argmax(signal.correlate(np.diff(normalize(signal2)), np.diff(normalize(signal1)))) - (BUFFER_SIZE - 1)


# Method to normalize arrays
def normalize(array):
    normalized_array = array - np.mean(array)
    normalized_array /= max(abs(normalized_array))
    return normalized_array


# This gets called when this is run as a script
if __name__ == '__main__':
    flow_meter()
