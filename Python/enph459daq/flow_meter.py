# Imports for reading from the Arduino
import arduino

# Imports for signal processing
from scipy import signal
from scipy import interpolate
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
from datetime import datetime

# ---Settings---
FAN_START_FDC = 1500
BUFFER_SIZE = 2000
FILTER_CUTOFF = 25.0
INTERP_FACTOR = 1.0

# ---Globals---
curr_rpm = 0
curr_flow_rate = 0


class GlobalFilter():
    def __init__(self, dt, cutoff):
        fs = 1.0E6/dt
        self.filter_b, self.filter_a = signal.butter(2, [cutoff / (fs / 2.0)], btype='low', analog=False)

    def update(self, dt, cutoff):
        fs = 1.0E6/dt
        self.filter_b, self.filter_a = signal.butter(2, [cutoff / (fs / 2.0)], btype='low', analog=False)

    def filtfilt(self, data):
        return signal.filtfilt(self.filter_b, self.filter_a, data)


tc_filter = GlobalFilter(1000.0, FILTER_CUTOFF)

# ---Exit Codes---
# 0 - Exited normally due to 'Exit' command from Arduino
# 1 - Exited because no Arduino boards could be found
# 2 - Exited because the serial port closed unexpectedly

# Main flow calculation method
def flow_meter():

    # Initialize the data buffers to zeros
    tc_data = DataList(BUFFER_SIZE,2)
    tc_init = [np.zeros(BUFFER_SIZE, dtype='f') for i in range(2)]
    tc_data.push(tc_init)
    flow_rate = DataBuffer(BUFFER_SIZE)
    flow_rate.push(np.zeros(BUFFER_SIZE, dtype='f'))
    rpm = DataBuffer(BUFFER_SIZE)
    rpm.push(np.zeros(BUFFER_SIZE, dtype='f'))

    # Thread initialization
    controller = FuncThread(controller_thread, tc_data, flow_rate, rpm)
    calculator = FuncThread(calculator_thread, tc_data)
    calculator.daemon = True
    plotter = FuncThread(plotter_thread, tc_data, flow_rate, rpm)
    plotter.daemon = True
    rpm_getter = FuncThread(rpm_thread, rpm)
    rpm_getter.daemon = True

    controller.start()
    calculator.start()
    plotter.start()
    rpm_getter.start()

class FuncThread(threading.Thread):
    def __init__(self, target, *args):
        self._target = target
        self._args = args
        threading.Thread.__init__(self)

    def run(self):
        self._target(*self._args)


def fake_data_thread(tc_data, flow_rate, rpm):
    # A thread that generates fake data, for testing when Arduino is unavailable

    delay = 20

    fake_buffer = DataBuffer(delay+1)
    fake_buffer.push(np.zeros(delay+1, dtype='f'))

    curr = 0

    while True:
        curr += random.randint(-1,1)*random.randint(-1,1)*random.randint(-1,1)
        fake_buffer.push(curr * np.ones(1, dtype='f'))

        fake_data = fake_buffer.get()
        data_push = [fake_data[delay] * np.ones(1, dtype='f') + random.randint(-10,10), fake_data[0] * np.ones(1, dtype='f') + random.randint(-10,10)]
        #data_push = [fake_data[delay] * np.ones(1, dtype='f'), fake_data[0] * np.ones(1, dtype='f')]
        tc_data.push(data_push)

        flow_rate.push(curr_flow_rate * np.ones(1, dtype='f'))
        rpm.push(curr_rpm * np.ones(1, dtype='f'))

        for i in range(0,100):
            print(0)


def controller_thread(tc_data, flow_rate, rpm):
    # A thread that controls the fan and manages Arduino communications, including data acquisition
    global tc_filter

    # Set the fan speed adjust to On and start the fan
    if ctrl.get_rpm() == 0:
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
                fs = 1.0E6/ts
                cutoff = FILTER_CUTOFF
                tc_filter.update(fs, cutoff)
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
                push_data = [int(line.split(',')[0]) * np.ones(1, dtype='f'), int(line.split(',')[1]) * np.ones(1, dtype='f')]
                tc_data.push(push_data)
                flow_rate.push(curr_flow_rate * np.ones(1, dtype='f'))
                rpm.push(curr_rpm * np.ones(1, dtype='f'))
            except:
                print('Invalid thermocouple data! Data buffers not modified.')

    print('Arduino port unexpectedly closed.')
    ctrl.set_speed_adjust(0)
    return 2


def calculator_thread(tc_data):
    # A thread that calculates flow rates from the current data and records fan RPM
    global tc_filter
    global curr_flow_rate

    while True:
        # Filtered thermocouple data
        data = tc_data.get()
        #x = np.arange(0, BUFFER_SIZE)
        tc1_filt = tc_filter.filtfilt(data[0])
        tc2_filt = tc_filter.filtfilt(data[1])
        #f1_interp = interpolate.interp1d(x, tc1_filt)
        #f2_interp = interpolate.interp1d(x, tc2_filt)

        #xnew = 1.0/INTERP_FACTOR * np.arange(0, INTERP_FACTOR*(BUFFER_SIZE-1))

        #tc1_interp = f1_interp(xnew)
        #tc2_interp = f2_interp(xnew)

        # Delay
        tof_delay = get_flow_rate(tc1_filt, tc2_filt)
        #tof_delay = get_flow_rate(tc1_filt, tc2_filt)
        #tof_delay = get_flow_rate(tc1_interp, tc2_interp)/INTERP_FACTOR
        curr_flow_rate = tof_delay


def plotter_thread(tc_data, flow_rate, rpm):
    # A thread that continuously plots the data
    global tc_filter

    app = QtGui.QApplication([])

    win = pyqtgraph.GraphicsWindow(title="Basic plotting examples")
    win.resize(1000, 900)
    win.setWindowTitle('pyqtgraph example: Plotting')

    pyqtgraph.setConfigOptions(antialias=True)

    plot_signals = win.addPlot(title="Filtered Signals")
    ptc1 = plot_signals.plot(pen='y')
    ptc2 = plot_signals.plot(pen='g')

    win.nextRow()
    plot_flow_rate = win.addPlot(title="Time of Flight")
    pfr = plot_flow_rate.plot(pen='y')
    plot_flow_rate.setYRange(10, 30, padding=0)

    win.nextRow()
    plot_velocity = win.addPlot(title="Scaled Velocity")
    pvel = plot_velocity.plot(pen='y')
    plot_velocity.setYRange(0,0.0001,padding=0)

    win.nextRow()
    plot_rpm = win.addPlot(title="RPM")
    prpm = plot_rpm.plot(pen='y')
    plot_rpm.setYRange(0, 2500, padding=0)

    def update():
        data = tc_data.get()
        tc1_filt = tc_filter.filtfilt(data[0])
        tc2_filt = tc_filter.filtfilt(data[1])

        ptc1.setData(np.diff(normalize(tc1_filt)))
        ptc2.setData(np.diff(normalize(tc2_filt)))
        pfr.setData(flow_rate.get())
        pvel.setData(1.0/(np.multiply((flow_rate.get()+1.0E-6),(rpm.get()+1.0E-6))))
        prpm.setData(rpm.get())

    timer = QtCore.QTimer()
    timer.timeout.connect(update)
    timer.start(50)

    app.exec_()


def rpm_thread(rpm):
    # A thread that calculates flow rates from the current data and records fan RPM
    global curr_rpm

    while True:
        # Update rpm buffer
        curr_rpm = ctrl.get_rpm()


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

class DataList:
    # A list of DataBuffer objects that must be modified simultaneously
    def __init__(self, length, num_buffers):
        self.buffers = [DataBuffer(length) for i in range(num_buffers)]
        self.buffer_length = length
        self.num_buffers = num_buffers
        self.lock = threading.Lock()

    def push(self, x_arr):
        self.lock.acquire()

        for i in range(self.num_buffers):
            self.buffers[i].push(x_arr[i])

        self.lock.release()

    def get(self):
        self.lock.acquire()

        current_data = [np.zeros(self.buffer_length) for i in range(self.num_buffers)]
        for i in range(self.num_buffers):
            current_data[i] = self.buffers[i].get()

        self.lock.release()

        return current_data

def get_flow_rate(signal1, signal2):
    # return np.argmax(signal.correlate(np.diff(normalize(signal2)), np.diff(normalize(signal1)))) - (BUFFER_SIZE*INTERP_FACTOR - 1)
    return np.argmax(signal.correlate(np.diff(normalize(signal2)), np.diff(normalize(signal1)))) - (np.size(signal1) - 1)


# Method to normalize arrays
def normalize(array):
    normalized_array = array - np.mean(array)
    normalized_array /= max(abs(normalized_array))
    return normalized_array


# This gets called when this is run as a script
if __name__ == '__main__':
    flow_meter()
