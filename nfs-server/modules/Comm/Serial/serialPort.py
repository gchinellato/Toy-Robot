#!/usr/bin/python
"""
*************************************************
* @Project: Self Balance
* @Description: Serial API
* @Owner: Guilherme Chinellato
* @Email: guilhermechinellato@gmail.com
*************************************************
"""

import threading
import datetime
import time
import serial
import serial.tools.list_ports as prtlst
import Queue as queue
from logFile import * 
from constants import *
from Utils.traces.trace import *
from Utils.constants import *

class SerialThread(threading.Thread):
    def __init__(self, group=None, target=None, name=None, args=(), kwargs=None, queue=queue.Queue(), debug=0, COM="/dev/ttyUSB0", callbackUDP=None, callbackFile=None):
        threading.Thread.__init__(self, group=group, target=target, name=name)
        self.args = args
        self.kwargs = kwargs
        self.name = name
        self.debug = debug
        self.callbackUDP = callbackUDP
        self.callbackFile = callbackFile

        #Queue to communicate between threads
        self._workQueue = queue
        self._lock = threading.Lock()

        #Event to signalize between threads
        self._stopEvent = threading.Event()
        self._sleepPeriod = 0.0

        self.COM = COM
        self.port = None

        logging.info("Serial Module initialized")

    #Override method
    def run(self):
        logging.info("Serial Thread Started")

        self.ser = serial.Serial()

        while not self._stopEvent.wait(self._sleepPeriod):
            try:
                if not self.ser.isOpen():				
                    time.sleep(2)
                    self.ser.port = prtlst.comports()[0][0]
                    self.ser.baudrate = 115200
                    self.ser.timeout = 5
                    logging.info(("Opening serial port " + str(self.ser.port) + "," + str(self.ser.baudrate)))
                    self.ser.open()

                msg = self.getMessage()
                if msg != None:
                    size = self.ser.write(''.join(msg))

                    if (self.debug & MODULE_SERIAL):
                        logging.debug(("Writing to Arduino: " + str(msg)))
                        #logging.debug(("Writing to Arduino: " + self.converStrToHex(msg)))

                #Read trace from arduino
                recv = self.ser.readline()

                if (self.debug & MODULE_SERIAL):
                    logging.debug(("Reading from Arduino: " + str(recv)))
                    #logging.debug(("Reading from Arduino: " + self.converStrToHex(str(recv))))

                #Parse event
                msgList = self.parseData(recv)
                UDP_MSG = None
                LOG_MSG = ""

                #Writing in a file...
                if (self.callbackFile != None and msgList != None):
                    for msg in msgList:
                        LOG_MSG += (msg + ";")
                    self.callbackFile(str(LOG_MSG))

                #Sending UDP packets...
                if msgList != None:
                    #(module),(data1),(data2),(data3),(...)(#)
                    UDP_MSG = CMD_SERIAL
                    for msg in msgList:
                        UDP_MSG += ("," + msg)
                    UDP_MSG += "#"

                if (self.callbackUDP != None and UDP_MSG != None):
                    self.callbackUDP(UDP_MSG)
            except queue.Empty:
                if (self.debug & MODULE_SERIAL):
                    logging.debug("Queue Empty")
                pass
            except serial.SerialException:
                logging.warning("SerialException")
                self.ser.close()
                pass
            finally:
                pass

    #Override method
    def join(self, timeout=2):
        #Stop the thread and wait for it to end
        logging.info("Killing Serial Thread...")
        self._stopEvent.set()
        self.ser.close()
        threading.Thread.join(self, timeout=timeout)

    def getMessage(self, timeout=2):
        #Bypass if empty, to not block the current thread
        if not self._workQueue.empty():
            return self._workQueue.get(timeout=timeout)
        else:
            return None

    def putMessage(self, command, msg):
        #Bypass if full, to not block the current thread
        if not self._workQueue.full():
            msg = self.checkData(command, msg)
            self._workQueue.put(msg)

    def parseData(self, strData):
        #Check if message is completed
        if ("#" in strData):
            strData = strData.replace("#","")
            strData = strData.replace("\r","")
            strData = strData.replace("\n","")
            data = strData.split(",")
            return data
        else:
            #logging.warning("Invalid message")
            return None

    def checkData(self, command, msg):
        if command == STARTED:
            msg = str(command) + "," + str(msg)
        elif command == DIRECTION:
            msg = str(command) + "," + str(round(msg,2))
        elif command == STEERING:
            msg = str(command) + "," + str(round(msg,2))
        elif command == SPEED_PID:
            msg = str(command) + "," + str(round(msg[0],2)) + "," + str(round(msg[1],2)) + "," + str(round(msg[2],2))
        elif command == ANGLE_PID:
            msg = str(command) + "," + str(round(msg[0],2)) + "," + str(round(msg[1],2)) + "," + str(round(msg[2],2))
        elif command == HEADING_PID:
            msg = str(command) + "," + str(round(msg[0],2)) + "," + str(round(msg[1],2)) + "," + str(round(msg[2],2))
        elif command == CALIBRATED_ZERO_ANGLE:
            msg = str(command) + "," + str(round(msg,2))
        elif command == ANGLE_LIMIT:
            msg = str(command) + "," + str(round(msg,2))
        else:
            msg = "unknown"

        return msg + "#" + "\r\n"

    def converStrToHex(self, msg):
        return ":".join("{:02x}".format(ord(c)) for c in msg)

    def convertTo(self, value, fromMax, fromMin, toMax, toMin):
        if not value >= fromMin and value <= fromMax:
            logging.warning("Value out of the range (Max:"+str(fromMax)+" , Min:"+str(fromMin)+")")
            if value > fromMax:
                value = fromMax
            elif value < fromMin:
                value = fromMin

        factor = (value-fromMin)/(fromMax-fromMin)
        return factor*(toMax-toMin)+toMin

