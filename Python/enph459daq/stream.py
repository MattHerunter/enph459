# Import for reading from the Arduino
import arduino

# Import for time stamping files
from datetime import datetime

# Import for waiting for the fan to adjust
from time import sleep

# Import for creating folders to hold data
import os

# Import for communicating with the controller
import controller as ctrl

# ---Settings---
FAN_START_FDC = 1500

# ---Exit Codes---
# 0 - Exited normally due to 'Exit' command from Arduino
# 1 - Exited because no Arduino boards could be found
# 2 - Exited because the serial port closed unexpectedly


# Main data acquisition method+
def stream():
    # Open an Arduino serial port
    arduino_port = arduino.ArduinoPort()

    # Turn the fan off and exit if no Arduino is found
    if arduino_port.arduino_serial is None:
        ctrl.set_speed_adjust(0)
        return 1

    # Initialize the streaming flag to false
    streaming = False

    # Initialize the FDC, RPM, test number and sample time, and time to 0
    fdc = 0
    rpm = 0
    test_num = 0
    ts = 0

    # Set the fan speed adjust to On and start the fan
    ctrl.set_speed_adjust(1)
    ctrl.set_fdc(FAN_START_FDC)

    # Make a directory for a new day if it does not exist already
    directory_path = os.getcwd() + datetime.now().strftime('\Data\%Y-%m-%d')
    if not os.path.exists(directory_path):
        os.makedirs(directory_path)

    # Continuously read from the serial port
    while arduino_port.is_open():
        # Read a line (terminated by '\n') from the serial port, and strip of the trailing '\n'
        command = arduino_port.read_line()
        
        # Received a 'Start' command
        if command == 'Start':
            if not streaming:
                print('Received \'Start\' command.')
                streaming = True
                # Open a timestamped .csv file to append data to
                filename = directory_path + '\\fdc'+str(fdc)+'rpm'+str(rpm)+'ts'+str(ts)+'_'+str(test_num)\
                    + datetime.now().strftime('_%H-%M-%S')+'.csv'
                out_file = open(filename, mode='a')
                test_num += 1
            else:
                print('Received \'Start\' command while streaming. Ignoring.')
        
        # Received a 'Stop' command
        elif command == 'Stop':
            if streaming:
                print('Received \'Stop\' command.')
                streaming = False
                out_file.flush()
                out_file.close()
            else:
                print('Received \'Stop\' command while not streaming. Ignoring.')
        
        # Received a 'Set' command
        elif 'Set' in command:
            if not streaming:
                fdc = int(command.split(':')[1])
                w = int(command.split(':')[2])
                print('Received \'Set\' command for ' + str(fdc) + '.')
                ctrl.set_fdc(fdc)
                sleep(w)
                rpm = ctrl.get_rpm()
                # Arduino will repeat the same test for new FDC/RPM
                test_num = 0
            else:
                print('Received \'Set\' command while streaming. Ignoring.')

        # Received a 'Time' command
        elif 'Time' in command:
            if not streaming:
                ts = int(command.split(':')[1])
                print('Received \'Time\' command. Sample time is ' + str(ts) + ' microseconds.')
            else:
                print('Received \'Time\' command while streaming. Ignoring.')

        # Received an 'Exit' command
        elif command == 'Exit':
            if not streaming:
                print('Received \'Exit\' command.')
                ctrl.set_speed_adjust(0)
                return 0
            else:
                print('Received \'Exit\' command while streaming. Ignoring.')
        
        # Not a command, and currently streaming data
        elif streaming:
            out_file.write(command + '\n')

    # Turn off the fan and exit if Arduino loses communication
    print('Arduino port unexpectedly closed.')
    ctrl.set_speed_adjust(0)
    return 2


# This gets called when this is run as a script
if __name__ == '__main__':
    stream()
