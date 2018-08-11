#!/usr/bin/python
"""
*************************************************
* @Project: Self Balance
*  @Platform: Raspberry PI 3 / Ubuntu / Qt
* @Description: Manager header file
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

#Initial values
SPEED_SETPOINT	=	0.0
SPEED_KP 		=	0.0
SPEED_KI 		=	0.0
SPEED_KD 		=	0.0
ANGLE_SETPOINT 	=	0.0
ANGLE_LIMIT 	=	45.0
ANGLE_KP_AGGR 	=	0.0
ANGLE_KI_AGGR 	=	0.0
ANGLE_KD_AGGR 	=	0.0
ANGLE_KP_CONS 	=	5.0
ANGLE_KI_CONS 	=	0.01
ANGLE_KD_CONS 	=	0.05
ANGLE_IRRECOVERABLE = 0.0
CALIBRATED_ZERO_ANGLE = -0.5
WINDUP_GUARD 		= 100
