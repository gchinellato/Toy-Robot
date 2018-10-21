#!/usr/bin/python
"""
*************************************************
* @Project: Self Balance
* @Description: Audio player
* @Owner: Guilherme Chinellato
* @Email: guilhermechinellato@gmail.com
*************************************************
"""

import os
import time
import threading
import Queue
import datetime
from constants import *
from Utils.gpio_mapping import *
from Utils.traces.trace import *
from Utils.constants import *

class Audiothread(threading.Thread):
    def __init__(self, group=None, target=None, name=None, args=(), kwargs=None, queue=Queue.Queue(), debug=0):
        threading.Thread.__init__(self, group=group, target=target, name=name)
        self.args = args
        self.kwargs = kwargs
        self.name = name
        self.debug = debug

        #Queue to communicate between threads
        self._workQueue = queue

        #Event to signalize between threads
        self._stopEvent = threading.Event()
        self._sleepPeriod = 0.0

        logging.info("Audio Module initialized")

    #Override method
    def run(self):
        logging.info("Audio Thread started")

        while not self._stopEvent.wait(self._sleepPeriod):
            try:
                os.system("omxplayer /home/pi/Music/220\ Volts\ -\ Capital\ Inicial.mp3")
                self._stopEvent.set()
            except Queue.Empty:
                if (self.debug & MODULE_AUDIO):
                    logging.debug("Queue Empty")
                pass
            finally:
                pass

    #Override method
    def join(self, timeout=None):
        #Stop the thread and wait for it to end
        logging.info("Killing Audio Thread...")
        self._stopEvent.set()
        threading.Thread.join(self, timeout=timeout)

    def getEvent(self, timeout=1):
        return self._workQueue.get(timeout=timeout)

    def putEvent(self, event):
        #Bypass if full, to not block the current thread
        if not self._workQueue.full():
            self._workQueue.put(event)

