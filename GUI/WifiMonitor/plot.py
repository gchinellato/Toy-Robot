#!/usr/bin/python3

import time
import queue
import logging
#import matplotlib.pyplot as plt
import numpy as np
import threading
from PyQt5 import QtGui, QtCore, QtWidgets
import random

class Plot(QtCore.QThread):
    def __init__(self, parent = None):
        QtCore.QThread.__init__(self, parent)
        self.parent = parent
        self.stop = False

        #Event to signalize between threads
        self._stopEvent = threading.Event()
        self._sleepPeriod = 0.1
        
    def run(self):
       pass

    def join(self):
        self._stopEvent.set()

