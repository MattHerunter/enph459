# Imports for reading from the Arduino
import arduino

# Imports for signal processing
from scipy import signal
from scipy import fftpack
import numpy as np
# from scipy import interpolate

# Imports for plotting (pyqt for speed, matplotlib for testing)
from pyqtgraph.Qt import QtGui, QtCore
import pyqtgraph
# import matplotlib.pyplot as plt

# Imports for black witchcraft involving wavelets
import pywt
import pywt._thresholding
# import wden

# Imports for sending/receiving controller commands
import controller as ctrl

# Import for waiting for the fan to adjust
from time import sleep

# Import for time stamping files
from datetime import datetime

# Import for multithreading
import threading

# Import for fake data generation
import random

# ---Exit Codes---
# 0 - Exited normally due to 'Exit' command from Arduino
# 1 - Exited because no Arduino boards could be found
# 2 - Exited because the serial port closed unexpectedly

# ---Settings---
FAN_START_FDC = 2000
BUFFER_SIZE = 5000
BUFFER_PADDING = 15
FILTER_CUTOFF = 10.0
FILTER_ORD = 4
FILTER_TYPE = 'LOWPASS'  # LOWPASS or WAVE
INTERP_FACTOR = 1.0
WEIGHT_FUNCTION = np.linspace(1, 1, num=BUFFER_SIZE)

USE_FILTER = True
USE_DIFF = True
USE_NORMALIZE = True
USE_SIGN = False
USE_HVS_CORRELATION = False

PLOT_FILTER = False
PLOT_DIFF = False
PLOT_NORMALIZE = False
PLOT_SIGN = False

# ---Globals---
current_rpm = 1
current_flow_rate = 0

current_tc1_err = 0
current_tc2_err = 0
current_cmp1_err = 0
current_cmp2_err = 0
current_err = 0
current_err_integral = 0
current_cv = 0

current_freq_ratio1 = 0
current_freq_ratio2 = 0
current_relative_diffusion = 0

# Filter that can be updated from one thread and used in another
class GlobalFilter:
    def __init__(self, dt, cutoff):
        fs = 1.0E6/dt
        self.filter_b, self.filter_a = signal.bessel(FILTER_ORD, [cutoff / (fs / 2.0)], btype='low', analog=False)
        #self.filter_b, self.filter_a = signal.bessel(3, [0.2 / (fs / 2.0), 10.0 / (fs / 2.0)], btype='band', analog=False)

    def update(self, dt, cutoff):
        fs = 1.0E6/dt
        self.filter_b, self.filter_a = signal.bessel(FILTER_ORD, [cutoff / (fs / 2.0)], btype='low', analog=False)
        #self.filter_b, self.filter_a = signal.bessel(3, [0.2 / (fs / 2.0), 10.0 / (fs / 2.0)], btype='band', analog=False)

    def filtfilt(self, data):
        if FILTER_TYPE == 'LOWPASS':
            return signal.filtfilt(self.filter_b, self.filter_a, data)
        elif FILTER_TYPE == 'WAVE':
            wavelet = 'sym4'
            coeffs = pywt.wavedec(signal.filtfilt(self.filter_b, self.filter_a, data),wavelet,level=4)
            #coeffs = pywt.wavedec(data,wavelet,level=4)
            threshold = 1
            newcoeffs = map(lambda x: pywt._thresholding.soft(x, threshold),coeffs)
            return pywt.waverec(newcoeffs,wavelet)
            #return wden.wden(signal.filtfilt(self.filter_b, self.filter_a, data),'sqtwolog','soft','mln',4,'db2')

tc_filter = GlobalFilter(1000.0, FILTER_CUTOFF)


# Main flow calculation method
def flow_meter():
    # Initialize thermocouple/heater voltage/comparator buffers to zero
    tc_data = DataList(BUFFER_SIZE, 5)
    tc_init = [np.zeros(BUFFER_SIZE, dtype='f') for i in range(5)]
    tc_data.push(tc_init)

    # Initialize flow rate buffer to zero
    flow_rate = DataBuffer(BUFFER_SIZE)
    flow_rate.push(np.zeros(BUFFER_SIZE, dtype='f'))

    # Initialize RPM buffer to zero
    rpm = DataBuffer(BUFFER_SIZE)
    rpm.push(np.zeros(BUFFER_SIZE, dtype='f'))

    # Buffers for PID data
    pid_data = DataList(BUFFER_SIZE, 7)
    pid_init = [np.zeros(BUFFER_SIZE, dtype='f') for i in range(7)]
    pid_data.push(pid_init)

    # Buffers for FFT data
    fft_data = DataList(BUFFER_SIZE, 3)
    fft_init = [np.zeros(BUFFER_SIZE, dtype='f') for i in range(3)]
    fft_data.push(fft_init)

    # Read data from the test bench if an Arduino is detected, use fake data otherwise
    arduino_connected = arduino.find_arduino_port() is not None
    if arduino_connected:
        controller = FuncThread(controller_thread, tc_data, flow_rate, rpm, pid_data, fft_data)
    else:
        controller = FuncThread(fake_data_thread, tc_data, flow_rate, rpm, pid_data, fft_data)

    # Initialize other threads
    calculator = FuncThread(calculator_thread, tc_data)
    calculator.daemon = True
    plotter = FuncThread(plotter_thread, tc_data, flow_rate, rpm, pid_data, fft_data)
    plotter.daemon = True
    rpm_getter = FuncThread(rpm_thread)
    rpm_getter.daemon = True
    feedback = FuncThread(feedback_thread, tc_data)
    feedback.daemon = True
    data_getter = FuncThread(data_collector)
    data_getter.daemon = True

    # Start the threads
    controller.start()
    calculator.start()
    plotter.start()
    feedback.start()
    # data_getter.start()
    # Only start RPM thread if using the test bench
    # if arduino_connected:
        # rpm_getter.start()


# Implements threads that can take parameters
class FuncThread(threading.Thread):
    def __init__(self, target, *args):
        self._target = target
        self._args = args
        threading.Thread.__init__(self)

    def run(self):
        self._target(*self._args)


# A thread that generates fake data, for testing when Arduino is unavailable
def fake_data_thread(tc_data, flow_rate, rpm):

    delay = 20
    noise = 10
    curr = 0

    fake_buffer = DataBuffer(delay+1)
    fake_buffer.push(np.zeros(delay+1, dtype='f'))

    while True:
        # Generate next fake data point
        curr_rand = random.randint(-1, 1)*random.randint(-1, 1)*random.randint(-1, 1)
        curr += curr_rand
        fake_buffer.push(curr * np.ones(1, dtype='f'))
        fake_data = fake_buffer.get()
        data_push = [curr_rand * np.ones(1, dtype='f'),
                     fake_data[delay] * np.ones(1, dtype='f') + random.randint(-noise, noise),
                     fake_data[0] * np.ones(1, dtype='f') + random.randint(-noise, noise)]

        # Push the fake data to the data buffers
        tc_data.push(data_push)
        flow_rate.push(current_flow_rate * np.ones(1, dtype='f'))
        rpm.push(current_rpm * np.ones(1, dtype='f'))

        # Gross timing solution since data is not being read from the serial port
        for i in range(0, 100):
            a=0#print(0)


# A thread that controls the fan and manages Arduino communications, including data acquisition
def controller_thread(tc_data, flow_rate, rpm, pid_data, fft_data):

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
        elif 'Time' in line:
            if not streaming:
                ts = int(line.split(':')[1])
                # Update filter
                tc_filter.update(1.0E6/ts, FILTER_CUTOFF)
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
                push_data = [int(line.split(',')[0]) * np.ones(1, dtype='f'), int(line.split(',')[1]) *
                             np.ones(1, dtype='f'), int(line.split(',')[2]) * np.ones(1, dtype='f'),
                             int(line.split(',')[3]) * np.ones(1, dtype='f'), int(line.split(',')[4]) *
                             np.ones(1, dtype='f')]
                tc_data.push(push_data)
                flow_rate.push(current_flow_rate * np.ones(1, dtype='f'))
                rpm.push(current_rpm * np.ones(1, dtype='f'))
                pid_push_data = [current_tc1_err * np.ones(1, dtype='f'), current_tc2_err *
                             np.ones(1, dtype='f'), current_cmp1_err * np.ones(1, dtype='f'),
                             current_cmp2_err * np.ones(1, dtype='f'), current_err *
                             np.ones(1, dtype='f'), current_err_integral * np.ones(1, dtype='f'),
                             current_cv * np.ones(1, dtype='f')]
                pid_data.push(pid_push_data)
                fft_push_data = [current_freq_ratio1 * np.ones(1, dtype='f'), current_freq_ratio2 *
                             np.ones(1, dtype='f'), current_relative_diffusion * np.ones(1, dtype='f')]
                fft_data.push(fft_push_data)

                arduino_port.write_line(int(current_cv))

            except (ValueError, IndexError):
                print('Invalid thermocouple data! Data buffers not modified.')

    print('Arduino port unexpectedly closed.')
    ctrl.set_speed_adjust(0)
    return 2


def data_collector():
    sleep(10)

    outfilename = "flow_rates_" + datetime.now().strftime('%H-%M-%S')

    start_fdc = 1400
    end_fdc = 2200
    num_fdcs = 80
    fdc_sq_delta = (np.sqrt(end_fdc)-np.sqrt(start_fdc))/num_fdcs

    def sample_flow_rate():
        sleep(0.001)
        a = 0
        while a < 1 or a > 100:
            a = current_flow_rate

        return a

    for i in range(num_fdcs):
        fdc = (np.sqrt(start_fdc)+i*fdc_sq_delta)*(np.sqrt(start_fdc)+i*fdc_sq_delta)
        print(fdc)
        ctrl.set_fdc(fdc)
        sleep(15)
        print("start collecting")
        flow_rates = [sample_flow_rate() for j in range(5000)]
        print("stop collecting")

        with open(outfilename, "a") as outfile:
            outfile.write(str(current_rpm) + ' ' + str(np.mean(flow_rates)) + ' ' + str(np.std(flow_rates)) + '\n')

    print("finished")


# A thread that calculates flow rates from the current data
def calculator_thread(tc_data):
    global current_flow_rate
    while True:
        data = tc_data.get()
        tc1 = data[1][BUFFER_PADDING:-BUFFER_PADDING]
        tc2 = data[2][BUFFER_PADDING:-BUFFER_PADDING]
        hvs = data[0][BUFFER_PADDING:-BUFFER_PADDING]
        if USE_FILTER:
            tc1 = tc_filter.filtfilt(tc1)
            tc2 = tc_filter.filtfilt(tc2)
        if USE_DIFF:
            tc1 = np.diff(tc1)
            tc2 = np.diff(tc2)
        if USE_SIGN:
            tc1 = np.sign(tc1)
            tc2 = np.sign(tc2)
        if USE_NORMALIZE:
            tc1 = normalize(tc1)
            tc2 = normalize(tc2)
            hvs = normalize(hvs)
        if USE_DIFF:
            tc1 = normalize(tc1 * WEIGHT_FUNCTION[BUFFER_PADDING:-BUFFER_PADDING-1])
            tc2 = normalize(tc2 * WEIGHT_FUNCTION[BUFFER_PADDING:-BUFFER_PADDING-1])
        else:
            tc1 = normalize(tc1 * WEIGHT_FUNCTION[BUFFER_PADDING:-BUFFER_PADDING])
            tc2 = normalize(tc2 * WEIGHT_FUNCTION[BUFFER_PADDING:-BUFFER_PADDING])
        hvs = normalize(hvs * WEIGHT_FUNCTION[BUFFER_PADDING:-BUFFER_PADDING])
        if USE_HVS_CORRELATION:
            tc1_delay = np.argmax(signal.correlate(hvs, tc1)) - (np.size(tc1) - 2)
            tc2_delay = np.argmax(signal.correlate(hvs, tc2)) - (np.size(tc2) - 2)
            current_flow_rate = tc1_delay - tc2_delay
        else:
            current_flow_rate = np.argmax(signal.correlate(tc2, tc1)) - (np.size(tc1) - 2)


# A thread that continuously plots the data
def plotter_thread(tc_data, flow_rate, rpm, pid_data, fft_data):

    class PlotWindow(pyqtgraph.GraphicsWindow):
        def __init__(self):
            pyqtgraph.GraphicsWindow.__init__(self, title="Flow Meter Plots")
            self.paused = False

        # Pauses the plotting app if 'P' is pressed
        def keyPressEvent(self, event):
            key = event.key()
            if key == QtCore.Qt.Key_P:
                self.paused = self.paused is False

    app = QtGui.QApplication([])

    win = PlotWindow()
    win.resize(1000, 900)
    win.setWindowTitle('Plots')

    pyqtgraph.setConfigOptions(antialias=True)

    # Plot for thermocouple signals
    plot_signals = win.addPlot(title="Filtered Signals")
    ptc1 = plot_signals.plot(pen='y')
    ptc2 = plot_signals.plot(pen='g')

    # Plot for comparators
    plot_cmp = win.addPlot(title="Comparators")
    pcmp1 = plot_cmp.plot(pen='y')
    pcmp2 = plot_cmp.plot(pen='g')
    plot_cmp.setYRange(-20, 200, padding=0)

    # Plot for FFT1
    plot_fft1 = win.addPlot(title="FFT")
    pfft1 = plot_fft1.plot(x=4,pen='y')

    win.nextRow()

    # Plot for heater voltage signal
    plot_hvs = win.addPlot(title="Heater Voltage Signal")
    phvs = plot_hvs.plot(pen='y')
    plot_hvs.setYRange(-2, 2, padding=0)

    # Plot for PID things
    plot_pid = win.addPlot(title="PID Errors")
    ptc1err = plot_pid.plot(pen='g')
    ptc2err = plot_pid.plot(pen='g')
    pcmp1err = plot_pid.plot(pen='y')
    pcmp2err = plot_pid.plot(pen='y')
    perr = plot_pid.plot(pen='r')

    # Plot for FFT2
    plot_fft2 = win.addPlot(title="FFT")
    pfft2 = plot_fft2.plot(x=4, pen='g')

    win.nextRow()

    # Plot for time of flight delay
    plot_flow_rate = win.addPlot(title="Time of Flight")
    pfr = plot_flow_rate.plot(pen='y')
    # plot_flow_rate.setYRange(-300,0, padding=0)

    # Plot for scaled velocity (1 over time of flight)
    # win.nextRow()
    # plot_velocity = win.addPlot(title="Scaled Velocity")
    # pvel = plot_velocity.plot(pen='y')
    # plot_velocity.setYRange(0, 0.00001, padding=0)

    # Plot for RPM
    # win.nextRow()
    # plot_rpm = win.addPlot(title="RPM")
    # prpm = plot_rpm.plot(pen='y')
    # plot_rpm.setYRange(0, 2500, padding=0)

    # Plot for PID things
    plot_pid = win.addPlot(title="PID Control Signal")
    perrintegral = plot_pid.plot(pen='y')
    pcv = plot_pid.plot(pen='r')

    # Plot for frequency ratios
    plot_freq = win.addPlot(title="Frequency Ratios")
    pfreq1 = plot_freq.plot(pen='y')
    pfreq2 = plot_freq.plot(pen='g')
    preld = plot_freq.plot(pen='b')

    def update():
        global current_freq_ratio1
        global current_freq_ratio2
        global current_relative_diffusion
        if not win.paused:
            data = tc_data.get()
            pid_vars = pid_data.get()

            tc1 = data[1]
            tc2 = data[2]

            fft1 = np.abs(fftpack.rfft(tc1-np.mean(tc1)))[0:300]
            fft2 = np.abs(fftpack.rfft(tc2-np.mean(tc2)))[0:300]

            if PLOT_FILTER:
                tc1 = tc_filter.filtfilt(tc1)
                tc2 = tc_filter.filtfilt(tc2)
            if PLOT_DIFF:
                tc1 = np.diff(tc1)
                tc2 = np.diff(tc2)
            if PLOT_SIGN:
                tc1 = np.sign(tc1)
                tc2 = np.sign(tc2)
            if PLOT_NORMALIZE:
                tc1 = normalize(tc1)
                tc2 = normalize(tc2)
            #if PLOT_DIFF:
                #tc1 = normalize(tc1 * WEIGHT_FUNCTION[0:-1])
                #tc2 = normalize(tc2 * WEIGHT_FUNCTION[0:-1])
            #else:
                #tc1 = normalize(tc1 * WEIGHT_FUNCTION)
                #tc2 = normalize(tc2 * WEIGHT_FUNCTION)

            # Update the data for each plot
            ptc1.setData(tc1)
            ptc2.setData(tc2)


            pfft1.setData(fft1)
            pfft2.setData(fft2)
            current_freq_ratio1 = np.max(fft1[0:20]) / np.max(fft1[20:-1])
            current_freq_ratio2 = np.max(fft2[0:20]) / np.max(fft2[20:-1])
            current_relative_diffusion = 10*current_freq_ratio2/current_freq_ratio1
            freq_data = fft_data.get()
            pfreq1.setData(freq_data[0])
            pfreq2.setData(freq_data[1])
            preld.setData(freq_data[2])

            pfr.setData(flow_rate.get())
            #pvel.setData(1.0 / (np.multiply((flow_rate.get() + 1.0E-6), (rpm.get() + 1.0E-6))))
            #prpm.setData(rpm.get())
            phvs.setData(normalize(data[0]*WEIGHT_FUNCTION))
            pcmp1.setData(data[3])
            pcmp2.setData(data[4]+20)

            ptc1err.setData(pid_vars[0])
            ptc2err.setData(pid_vars[1])
            pcmp1err.setData(pid_vars[2])
            pcmp2err.setData(pid_vars[3])
            perr.setData(pid_vars[4])
            perrintegral.setData(pid_vars[5])
            pcv.setData(pid_vars[6])

    timer = QtCore.QTimer()
    timer.timeout.connect(update)
    timer.start(50)
    app.exec_()

# A thread that continuously plots the data
def feedback_thread(tc_data):
    global current_tc1_err
    global current_tc2_err
    global current_cmp1_err
    global current_cmp2_err
    global current_err
    global current_err_integral
    global current_cv

    tc_threshold = 750
    cmp_threshold = 50
    kp = 10
    ki = 10
    kd = 0
    err_integral = 0
    err_derivative = 0
    err = 0
    last_err = 0
    cv_max = 255
    cv_min = 0
    dt = 0.01

    amplifier_target = 0.1

    while True:
        sleep(dt)

        data = tc_data.get()
        tc1 = data[1][4000:-1]
        tc2 = data[2][4000:-1]
        cmp1 = data[3][4000:-1]
        cmp2 = data[4][4000:-1]

        # Thermocouples
        #tc1_err = (amplifier_target - np.mean(np.greater((tc1 - tc_threshold), 0 * (tc1 - tc_threshold))))/amplifier_target
        #tc2_err = (amplifier_target - np.mean(np.greater((tc2 - tc_threshold), 0 * (tc2 - tc_threshold))))/amplifier_target
        tc1_err = (tc_threshold - np.max(tc1)) / tc_threshold
        tc2_err = (tc_threshold - np.max(tc2)) / tc_threshold
        tc_err = min(tc1_err, tc2_err)

        # Comparators
        cmp1_err = (amplifier_target - np.mean(np.greater((cmp1 - cmp_threshold), 0 * (cmp1 - cmp_threshold))))/amplifier_target
        cmp2_err = (amplifier_target - np.mean(np.greater((cmp2 - cmp_threshold), 0 * (cmp2 - cmp_threshold))))/amplifier_target
        cmp_err = min(cmp1_err, cmp2_err)

        last_err = err
        err = min(tc_err, cmp_err)
        err_integral += err*dt
        err_derivative = (err - last_err)/dt

        if ki > 0:
            err_integral = min(err_integral, cv_max/ki)
            err_integral = max(err_integral, -cv_max/ki)
        else:
            err_integral = 0

        cv = kp*err + ki*err_integral + kd*err_derivative
        cv = min(cv, cv_max)
        cv = max(cv, cv_min)

        current_tc1_err = tc1_err
        current_tc2_err = tc2_err
        current_cmp1_err = cmp1_err
        current_cmp2_err = cmp2_err
        current_err = err
        current_err_integral = err_integral
        current_cv = cv



# A thread that calculates flow rates from the current data and records fan RPM
def rpm_thread():
    global current_rpm
    while True:
        # Update rpm buffer
        try:
            current_rpm = ctrl.get_rpm()
        except:
            print("Questionable RPM man.")


# A 1D buffer using numpy arrays
class DataBuffer:
    def __init__(self, length):
        self.data = np.zeros(length, dtype='f')
        self.index = 0
        self.lock = threading.Lock()

    def push(self, x):
        self.lock.acquire()

        # Adds array x to data buffer
        x_index = (self.index + np.arange(x.size)) % self.data.size
        self.data[x_index] = x
        self.index = x_index[-1] + 1

        self.lock.release()

    def get(self):
        self.lock.acquire()

        # Returns the first-in-first-out data in the data buffer
        idx = (self.index + np.arange(self.data.size)) % self.data.size
        current_data = self.data[idx]

        self.lock.release()

        return current_data


# A list of DataBuffer objects that must be modified simultaneously
class DataList:
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


# Normalizes arrays for zero mean and 1 absolute magnitude
def normalize(array):
    normalized_array = array - np.mean(array)
    normalized_array /= max(abs(normalized_array))
    return normalized_array


# This gets called when run as a script
if __name__ == '__main__':
    testing = False
    if not testing:
        flow_meter()
    else:
        testing = False

