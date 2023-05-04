#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import time
import logging
logging.basicConfig(level=logging.INFO)
sys.path.append('../')

from obswebsocket import obsws, requests   # noqa: E402

class OBS():

    def __init__(self, host: str="localhost", port="4444", password="secret") -> None:
        self.host = host
        self.port = port
        self.password = password

        self.webSoccketOBS = obsws(self.host, self.port, self.password)

    def connectOBS(self):
        self.webSoccketOBS.connect()

    def disconnectOBS(self):
        self.webSoccketOBS.disconnect()
        
    def startRecord(self):
        try:
            self.webSoccketOBS.call(requests.StartStopRecording())

        except:
            pass

    def stopRecord(self):
        try:
            self.webSoccketOBS.call(requests.StartStopRecording())

        except:
            pass

