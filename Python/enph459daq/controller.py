# Imports for sending/receiving controller commands
import requests
from requests.auth import HTTPBasicAuth
import json
import ast

# ---IP Address of the Controller---
controllerAddress = '192.168.137.140'


# Method to set the fan duty cycle on the controller
def set_fdc(fdc):
    if 1200 <= fdc <= 12000:
        command = 'http://' + controllerAddress + '/cgi-bin/bc2-cgi?json=' + '{ \'object_no\' : 21 , \'boiler_no\' : ' \
            '0 , \'FanDuty\' : ' + str(fdc) + ', \'Update\' : ' + str(get_update_number('FanDuty')) + '}'
        requests.get(command, auth=HTTPBasicAuth('admin', 'IBC-c3h8'), timeout=1)
    else:
        print(str(fdc) + ' is outside the fan operating range of 1200 to 12000. set_fdc failed!')


# Method to set the fan speed adjust (allows duty cycle override) on the controller
def set_speed_adjust(mode):
    command = 'http://' + controllerAddress + '/cgi-bin/bc2-cgi?json=' + '{ \'object_no\' : 21 , \'boiler_no\' : 0 , ' \
        '\'SpeedAdjust\' : ' + str(mode) + ', \'Update\' : ' + str(get_update_number('SpeedAdjust')) + '}'
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
