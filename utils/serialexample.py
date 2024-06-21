from serialhandler import *
from threadhandler import *
from threading import Thread, Lock
from copy import copy
import time
from serial import Serial

thLock = Lock()

def update():
    global serialHandler, thLock
    thLock.acquire()
    n = len(serialHandler.dataQueue)
    q = copy(serialHandler.dataQueue)
    serialHandler.dataQueue.clear()
    thLock.release()
    for k in range(n):
        data = q.popleft()
        strData = ''
        for i in range(0,32,2):
            strData += str(data[i]<<8 | data[i+1]) + ' '
            #strData = str(data[i]) + ' '
        print(strData + '\n')

    time.sleep(0.01)

serialHandler = SerialHandler(_port='COM5',_baud=115200,_timeout=0.5,_header=0x24,_end=0x21,_numDataBytes=34,_thLock=None)
serialHandler.open()
time.sleep(1)
thAcq = ThreadHandler(serialHandler.readPackage)
thProc = ThreadHandler(update)
thAcq.start()
thProc.start()
a = input()
