# Imports for reading from the Arduino
import arduino

# Imports for signal processing
from scipy import signal
import numpy as np
import matplotlib.pyplot as plt
from pyqtgraph.Qt import QtGui, QtCore
import pyqtgraph

# Imports for sending/receiving controller commands
import controller as ctrl

# Import for waiting for the fan to adjust
from time import sleep

import threading
import random

# ---Settings---
FAN_START_FDC = 1500
BUFFER_SIZE = 1000

# ---Exit Codes---
# 0 - Exited normally due to 'Exit' command from Arduino
# 1 - Exited because no Arduino boards could be found
# 2 - Exited because the serial port closed unexpectedly

# Main flow calculation method
def flow_meter():

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

    # Thread initialization
    controller = FuncThread(fake_data_thread, tc1, tc2)
    calculator = FuncThread(calculator_thread, tc1, tc2, flow_rate, rpm, filter_a, filter_b)
    calculator.daemon = True
    plotter = FuncThread(plotter_thread, tc1, tc2, flow_rate, rpm, filter_a, filter_b)
    plotter.daemon = True
    rpm_getter = FuncThread(rpm_thread, rpm)
    rpm_getter.daemon = True

    controller.start()
    calculator.start()
    plotter.start()
    #rpm_getter.start()


class FuncThread(threading.Thread):
    def __init__(self, target, *args):
        self._target = target
        self._args = args
        threading.Thread.__init__(self)

    def run(self):
        self._target(*self._args)


def fake_data_thread(tc1, tc2):
    # A thread that generates fake data, for testing when Arduino is unavailable

    delay = 20

    fake_buffer = DataBuffer(delay+1)
    fake_buffer.push(np.zeros(delay+1, dtype='f'))

    curr = 0

    while True:
        curr += random.randint(-1,1)*random.randint(-1,1)*random.randint(-1,1)
        fake_buffer.push(curr * np.ones(1, dtype='f'))

        fake_data = fake_buffer.get()
        tc1.push(fake_data[delay] * np.ones(1, dtype='f'))
        tc2.push(fake_data[0] * np.ones(1, dtype='f'))

        sleep(0.002)


def controller_thread(tc1, tc2):
    # A thread that controls the fan and manages Arduino communications, including data acquisition

    # Set the fan speed adjust to On and start the fan
    ctrl.set_speed_adjust(1)
    ctrl.set_fdc(FAN_START_FDC)

    # Create the serial object and set the port, baud rate, and timeout
    arduino_port = arduino.ArduinoPort()

    # Exit if no Arduino devices were found
    if arduino_port.arduino_serial is None:
        ctrl.set_speed_adjust(0)
        return 1

    # Initialize the streaming flag to false
    streaming = False

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
        # Currently will NOT work properly, these filter variables are not the same ones being used by the calculator.
        elif 'Time' in line:
            if not streaming:
                ts = int(line.split(':')[1])
                # Update filter
                fs = 10.0E6/ts
                cutoff = 20.0
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
                print('Invalid thermocouple data! Data buffers not modified.')

    print('Arduino port unexpectedly closed.')
    ctrl.set_speed_adjust(0)
    return 2


def calculator_thread(tc1, tc2, flow_rate, rpm, filter_a, filter_b):
    # A thread that calculates flow rates from the current data and records fan RPM
    while True:
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
        flow_rate.push(tof_delay * np.ones(1, dtype='f'))


class PlotUpdater(QtCore.QTimer):
    def __init__(self, timeout, *args):
        self._timeout = timeout
        self._args = args
        QtCore.QTimer.__init__(self)

    def run(self):
        self._target(*self._args)


def plotter_thread(tc1, tc2, flow_rate, rpm, filter_a, filter_b):
    # A thread that continuously plots the data

    app = QtGui.QApplication([])

    win = pyqtgraph.GraphicsWindow(title="Basic plotting examples")
    win.resize(1000, 600)
    win.setWindowTitle('pyqtgraph example: Plotting')

    pyqtgraph.setConfigOptions(antialias=True)

    plot_signals = win.addPlot(title="Filtered Signals")
    ptc1 = plot_signals.plot(pen='y')
    ptc2 = plot_signals.plot(pen='g')

    plot_flow_rate = win.addPlot(title="Flow Rate")
    pfr = plot_flow_rate.plot(pen='y')

    plot_rpm = win.addPlot(title="RPM")
    prpm = plot_rpm.plot(pen='y')

    def update():
        tc1_filt = signal.filtfilt(filter_b, filter_a, tc1.get())
        tc2_filt = signal.filtfilt(filter_b, filter_a, tc2.get())

        ptc1.setData(tc1_filt)
        ptc2.setData(tc2_filt)
        pfr.setData(flow_rate.get())
        prpm.setData(rpm.get())

    timer = QtCore.QTimer()
    timer.timeout.connect(update)
    timer.start(50)

    app.exec_()

def rpm_thread(rpm):
    # A thread that calculates flow rates from the current data and records fan RPM
    while True:
        # Update rpm buffer
        rpm.push(ctrl.get_rpm() * np.ones(1, dtype='f'))


class DataBuffer:
    # A 1D ring buffer using numpy arrays
    def __init__(self, length):
        self.data = np.zeros(length, dtype='f')
        self.index = 0
        self.lock = threading.Lock()

    def push(self, x):
        self.lock.acquire()

        # Adds array x to ring buffer
        x_index = (self.index + np.arange(x.size)) % self.data.size
        self.data[x_index] = x
        self.index = x_index[-1] + 1

        self.lock.release()

    def get(self):
        self.lock.acquire()

        # Returns the first-in-first-out data in the ring buffer
        idx = (self.index + np.arange(self.data.size)) % self.data.size
        current_data = self.data[idx]

        self.lock.release()

        return current_data


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
