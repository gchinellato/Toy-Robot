#!/usr/bin/python
"""
*************************************************
* @Project: Self Balance
* @Platform: Raspberry PI 3
* @Description: Constants header file
* @Owner: Guilherme Chinellato
* @Email: guilhermechinellato@gmail.com
*************************************************
"""

#Service names
CLIENT_UDP_NAME = "Client-UDP-Thread"
SERVER_UDP_NAME = "Server-UDP-Thread"
BALANCE_NAME = "Balance-Thread"
PAN_TILT_NAME = "PanTilt-Thread"
PS3_CTRL_NAME = "PS3-Controller-Thread"
TRACKING_NAME = "Tracking-Thread"
SERIAL_NAME = "Serial-Thread"
LOG_FILE_NAME = "LogFile-Thread"

#Commands
CMD_BALANCE = "BALANCE"
CMD_PAN_TILT = "PAN_TILT"
CMD_PID_ANGLE = "PID_ANGLE"
CMD_PID_SPEED = "PID_SPEED"
CMD_UDP_CLIENT = "UDP_CLIENT"
CMD_UDP_SERVER = "UDP_SERVER"
CMD_SERIAL = "SERIAL"
CMD_MANAGER = "MANAGER"

STARTED = 0
DIRECTION = 1
STEERING = 2
SPEED_PID = 3
ANGLE_PID_AGGR = 4
ANGLE_PID_CONS = 5
CALIBRATED_ZERO_ANGLE = 6
ANGLE_LIMIT = 7
CF_IMU = 8

#Analog Constrains
ANALOG_MAX = 1.0
ANALOG_MIN = -1.0

#Position limits (in percentage)
PWM_MAX = 100
PWM_MIN = -100
