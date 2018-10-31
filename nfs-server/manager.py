#!/usr/bin/python
"""
*************************************************
* @Project: Self Balance
* @Platform: Raspberry PI 2 B+
* @Description: Manager all threads/processes
* @Owner: Guilherme Chinellato
* @Email: guilhermechinellato@gmail.com
*************************************************
"""

"""
IMPORTANT
The balance thread is not managed by Raspberry anymore, now Arduino is responsible for all tasks related to Balance and Motion.
For this reason, Balance, IMU and Motion modules into NFS-server folder are descontinued, please check Arduino directory
"""

from Comm.UDP.UDP_Server import UDP_ServerThread
from Comm.UDP.UDP_Client import UDP_ClientThread
from Comm.Bluetooth.controller_ps3 import PS3_ControllerThread
from Comm.Serial.serialPort import SerialThread
from Comm.Serial.serialPort import LogFileThread
from PanTilt.panTilt import PanTiltThread
from ComputerVison.tracking import ComputerVisionThread
from Comm.Serial.constants import *
from PanTilt.constants import *
from Utils.traces.trace import *
from Utils.constants import *
from Utils.gpio_mapping import *
import time
import picamera
import RPi.GPIO as GPIO
import Queue as queue
import pygame
import argparse
import os
import subprocess

def argParse():
    #Construct the argument parse and parse the arguments
    ap = argparse.ArgumentParser()
    ap.add_argument("-v", "--verbosity", help="set verbosity level (info, debug, warning, error, critical)")
    ap.add_argument("-m", "--module", type=int, default=0, help="set modules to print (see trace.py)")
    return vars(ap.parse_args())

def main(args):
    try:
        #Set verbosity level (info, debug, warning, error, critical)
        if args.get("verbosity") != None:
            setVerbosity(args.get("verbosity"))
            logging.info("Verboseity level: " + str(args.get("verbosity")))

        #Set modules to print according verbosity level
        debug = MODULE_MANAGER | MODULE_PANTILT | MODULE_BLUETOOTH | MODULE_SERIAL # | MODULE_CLIENT_UDP

        #Eyes init
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(LED_RED_GPIO, GPIO.OUT)
        GPIO.setup(LED_GREEN_GPIO, GPIO.OUT)
        GPIO.setup(LED_BLUE_GPIO, GPIO.OUT)

        GPIO.output(LED_RED_GPIO, 0)
        GPIO.output(LED_GREEN_GPIO, 0)
        GPIO.output(LED_BLUE_GPIO, 0)

        #Message queues to communicate between threads
        clientUDPQueue = queue.Queue()
        eventQueue = queue.Queue()
        panTiltQueue = queue.Queue()
        serialToWrite = queue.Queue()
        logFileQueue = queue.Queue()

        logging.info("Starting threads and process...")
        threads = []

        #UDP Client thread
        clientUDP = UDP_ClientThread(name=CLIENT_UDP_NAME, queue=clientUDPQueue, debug=debug, UDP_IP="192.168.0.100", UDP_PORT=5000)
        clientUDP.daemon = True
        #threads.append(clientUDP)
        #clientUDP.start()

        #UDP Server thread
        serverUDP = UDP_ServerThread(name=SERVER_UDP_NAME, queue=eventQueue, debug=debug, UDP_IP="", UDP_PORT=5001)
        serverUDP.daemon = True
        threads.append(serverUDP)
        serverUDP.start()

        #Joystick thread
        joy = PS3_ControllerThread(name=PS3_CTRL_NAME, queue=eventQueue, debug=debug)
        joy.daemon = True
        threads.append(joy)
        joy.start()

        #Log File thread
        logFile = LogFileThread(name=LOG_FILE_NAME, queue=logFileQueue, debug=debug)
        logFile.daemon = True
        #threads.append(logFile)
        #logFile.start()

        #Serial thread
        serial = SerialThread(name=SERIAL_NAME, queue=serialToWrite, COM="/dev/ttyUSB0", debug=debug, callbackUDP=clientUDP.putMessage, callbackFile=logFile.putMessage)
        serial.daemon = True
        threads.append(serial)
        serial.start()

        #Computer Vision thread
        tracking = ComputerVisionThread(name=TRACKING_NAME, queue=eventQueue, debug=debug)
        tracking.daemon = True
        threads.append(tracking)
        tracking.start()
        #camera = picamera.PiCamera()
        #camera.resolution = (800, 480)
        #camera.framerate = 60
        #camera.start_preview()

        #Pan-Tilt thread
        panTilt = PanTiltThread(name=PAN_TILT_NAME, queue=panTiltQueue, debug=debug, callbackUDP=clientUDP.putMessage)
        panTilt.daemon = True
        threads.append(panTilt)
        panTilt.start()

        runSpeed = 0.0
        turnSpeed = 0.0

        lastTime = 0.0
        LP = 0.0
        runSpeedMax = RUN_SPEED
        runSpeed_last = 0.0
        speedA = 0.8

        trackingEn = False

        while True:
            try:
                currentTime = time.time()

                #Calculate time since the last time it was called
                #if (debug & MODULE_MANAGER):
                    #logging.debug("Duration: " + str(currentTime - lastTime))

                event = eventQueue.get(timeout=2)
                if event != None:
                    if event[0] == PS3_CTRL_NAME and joy.joyStatus != None:
                        if (event[1].type == pygame.JOYAXISMOTION) and (event[1].axis != joy.A_ACC_X) and (event[1].axis != joy.A_ACC_Y) and (event[1].axis != joy.A_ACC_Z):
                            #Head Vertical
                            if event[1].axis == joy.A_R3_V:
                                headV = -event[1].value
                                panTilt.putEvent((headV, None))
                            #Head Horizontal
                            if event[1].axis == joy.A_R3_H:
                                headH = -event[1].value
                                panTilt.putEvent((None, headH))
                            #Body run speed
                            if event[1].axis == joy.A_L3_V:
                                runSpeed = -event[1].value

                                if runSpeed < runSpeed_last:
                                    runSpeed = (float)(speedA*runSpeed_last) + (runSpeed*(1-speedA))

                                runSpeed = serial.convertTo(runSpeed, ANALOG_MAX, ANALOG_MIN, runSpeedMax, -runSpeedMax)
                                serial.putMessage(DIRECTION, runSpeed)
                                runSpeed_last = runSpeed
                            #Body turn speed
                            if event[1].axis == joy.A_L3_H:
                                turnSpeed = -event[1].value
                                turnSpeed = serial.convertTo(turnSpeed, ANALOG_MAX, ANALOG_MIN, 3, -3)
                                serial.putMessage(STEERING, turnSpeed)

                        if event[1].type == pygame.JOYBUTTONDOWN or event[1].type == pygame.JOYBUTTONUP:
                            if event[1].button == joy.B_START:
                                serial.putMessage(STARTED, 1)
                                serial.putMessage(SPEED_PID, (0, 0, 0))
                                time.sleep(0.2)
                                serial.putMessage(SPEED_PID, (5.0, 0, 0.02))
                                GPIO.output(LED_RED_GPIO, 0)
                                GPIO.output(LED_GREEN_GPIO, 1)
                                GPIO.output(LED_BLUE_GPIO, 1)
                                logging.info("Button Start")
                            if event[1].button == joy.B_SELECT:
                                serial.putMessage(STARTED, 0)
                                GPIO.output(LED_RED_GPIO, 0)
                                GPIO.output(LED_GREEN_GPIO, 0)
                                GPIO.output(LED_BLUE_GPIO, 0)
                                logging.info("Button Select")
                            
                        if event[1].type == pygame.JOYBUTTONUP:
                            if event[1].button == joy.B_L1:
                                runSpeedMax = RUN_SPEED
                                logging.info("Button release L1: " + str(runSpeedMax))

                            if event[1].button == joy.B_R2:
                                serial.putMessage(POS_PID, 0)
                                logging.info("Button release R2")

                        if event[1].type == pygame.JOYBUTTONDOWN:
                            if event[1].button == joy.B_L1:
                                runSpeedMax = RUN_SPEED*5
                                logging.info("Button press L1: " + str(runSpeedMax))

                            if event[1].button == joy.B_L2:
                                subprocess.Popen(['omxplayer','-b','/mnt/nfs/MynameisiStalker.mp3'], stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, shell=False)  
                                logging.info("Button press L2")

                            if event[1].button == joy.B_R2:
                                serial.putMessage(POS_PID, 1)
                                logging.info("Button press R2")

                            if event[1].button == joy.B_R1:
                                logging.info("Button press R1")
                                serial.putMessage(RESET_ENCODER, 1)

                            if event[1].button == joy.B_X:
                                logging.info("Button X") 
                                GPIO.output(LED_RED_GPIO, 0)
                                GPIO.output(LED_GREEN_GPIO, 1)
                                GPIO.output(LED_BLUE_GPIO, 1)

                            if event[1].button == joy.B_CIRC:
                                logging.info("Button Circle")
                                GPIO.output(LED_RED_GPIO, 1)
                                GPIO.output(LED_GREEN_GPIO, 0)
                                GPIO.output(LED_BLUE_GPIO, 0)
                        
                            if event[1].button == joy.B_TRI:                                
                                trackingEn ^= 1
                                logging.info("Button TRI: " + str(trackingEn))

                    #IP controller
                    #[(Thread)][(module),(data1),(data2),(data3),(...)(#)]
                    elif event[0] == SERVER_UDP_NAME:
                        logging.debug(event[1])  
                        if event[1][0] == "e8912037a63d":
                            headH = float(event[1][2])
                            headH = panTilt.convertTo(headH, 50, -50, ANALOG_MAX, ANALOG_MIN)
                            panTilt.putEvent((None, headH))                            
                            #headV = float(event[1][2])
                            #panTilt.putEvent((headV, None))
                        elif event[1][0] == CMD_PID_ANGLE:
                            logging.info("Set PID Angle parameters!")
                            angleKpCons = float(event[1][1])
                            angleKiCons = float(event[1][2])
                            angleKdCons = float(event[1][3])
                            serial.putMessage(ANGLE_PID, (angleKpCons, angleKiCons, angleKdCons))

                            angleKpAggr = float(event[1][4])
                            angleKiAggr = float(event[1][5])
                            angleKdAggr = float(event[1][6])
                            serial.putMessage(HEADING_PID, (0.0, 0.0, 0.0))

                            calibratedZeroAngle = float(event[1][7])
                            serial.putMessage(CALIBRATED_ZERO_ANGLE, calibratedZeroAngle)

                            cf = float(event[1][8])
                            serial.putMessage(CF_IMU, cf)
                        elif event[1][0] == CMD_PID_SPEED:
                            logging.info("Set PID Speed parameters!")
                            speedKp = float(event[1][1])
                            speedKi = float(event[1][2])
                            speedKd = float(event[1][3])
                            serial.putMessage(SPEED_PID, (speedKp, speedKi, speedKd)) 
                        elif event[1][0] == CMD_SERIAL:
                            runSpeed = float(event[1][1])
                            serial.putMessage(DIRECTION, runSpeed)
                            turnSpeed = float(event[1][2])
                            serial.putMessage(STEERING, turnSpeed)
                        elif event[1][0] == CMD_MANAGER:
                            enableArduino = int(event[1][1])
                            serial.putMessage(STARTED, enableArduino)
                            pass
                        else:
                            pass

                    #OpenCV controller
                    elif event[0] == TRACKING_NAME and trackingEn == True:
                        logging.info(event)
                        tracking.block.set()

                        #Delta measure from object up to center of the vision
                        dWidth, dHeight, radius = event[1]
                           
                        if (radius > 10):
                            #subprocess.Popen(['omxplayer','-b','/mnt/nfs/Opabolinhaverde.mp3'], stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, shell=False)                 
                            GPIO.output(LED_RED_GPIO, 1)
                            GPIO.output(LED_GREEN_GPIO, 0)
                            GPIO.output(LED_BLUE_GPIO, 0)  
                            time.sleep(1)                
                        else:
                            GPIO.output(LED_RED_GPIO, 0)
                            GPIO.output(LED_GREEN_GPIO, 1)
                            GPIO.output(LED_BLUE_GPIO, 1)

                        #Get current head angles
                        angleV, angleH = panTilt.getScaledAngles()

                        if (debug & MODULE_MANAGER):
                            logging.debug(("Distance to center X: {0}, Y: {1}, Radius:{2}".format(dWidth, dHeight, radius)))
                            logging.debug("Angles current: " + str((angleV, angleH)))

                        #Head Vertical
                        if dHeight < -100 or dHeight > 100:
                            if dHeight < -100:
                                angle = -10.0
                                if dHeight < -200:
                                    angle = -20.0
                            elif dHeight > 100:
                                angle = 10.0
                                if dHeight > 200:
                                    angle = 20.0
                            headV = panTilt.convertTo(angleV+angle, ANGLE_MAX, ANGLE_MIN, ANALOG_MAX, ANALOG_MIN)
                            panTilt.putEvent((headV, None))

                        #Head Horizontal
                        if dWidth < -100 or dWidth > 100:
                            if dWidth < -100:
                                angle = 10.0
                                if dWidth < -200:
                                    angle = 20.0
                            elif dWidth > 100:
                                angle = -10.0
                                if dWidth > 200:
                                    angle = -20.0
                            headH = panTilt.convertTo(angleH+angle, ANGLE_MAX, ANGLE_MIN, ANALOG_MAX, ANALOG_MIN)
                            panTilt.putEvent((None, headH))
                        tracking.block.clear()
            except queue.Empty:
                #if (debug & MODULE_MANAGER):
                    #logging.debug("Queue Empty")
                pass
            finally:
                lastTime = currentTime
                time.sleep(LP)
    finally:
        logging.info("########################################")
        logging.info("Exiting...")
        GPIO.cleanup()
        for t in threads:
            logging.info("Killing "+ str(t.name) + "...")
            t.join()
        print("Booting down system...")

if __name__ == '__main__':
    print("##### Mr. Robot #####")
    print("### by gchinellato ###\n")
    print("Booting up system...")
    args = argParse()
    main(args)
