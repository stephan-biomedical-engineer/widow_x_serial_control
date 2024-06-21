
# -*- coding: utf-8 -*-
'''
#-------------------------------------------------------------------------------
# NATIONAL UNIVERSITY OF SINGAPORE - NUS
# SINGAPORE INSTITUTE FOR NEUROTECHNOLOGY - SINAPSE
# Singapore
# URL: http://www.sinapseinstitute.org
#-------------------------------------------------------------------------------
# Neuromorphic Engineering Group
# Author: Andrei Nakagawa-Silva, MSc
# Contact: nakagawa.andrei@gmail.com
#-------------------------------------------------------------------------------
# Description: This file contains a class for handling serial communication
# with embedded systems such as Arduino boards or ARM dev kits
#-------------------------------------------------------------------------------
# [HEADER][NBYTES][ADC0_MSB][ADC0_LSB][ADC1_MSB][ADC1_LSB][END]
#-------------------------------------------------------------------------------
'''
#-------------------------------------------------------------------------------
import serial
from serial import Serial
from threading import Timer
from ctypes import c_short
from struct import unpack
from collections import deque
from threading import Lock
import time
import sys
import glob
#-------------------------------------------------------------------------------
#method for listing all the serial ports available
def list_serial_ports():
    """ Lists serial port names

    :raises EnvironmentError:
    On unsupported or unknown platforms
    :returns:
    A list of the serial ports available on the system
    """
    if sys.platform.startswith('win'):
        ports = ['COM%s' % (i + 1) for i in range(256)]
    elif sys.platform.startswith('linux') or sys.platform.startswith('cygwin'):
        # this excludes your current terminal "/dev/tty"
        ports = glob.glob('/dev/tty[A-Za-z]*')
        #print(ports) #debugging
    elif sys.platform.startswith('darwin'):
        ports = glob.glob('/dev/tty.*')
    else:
        raise EnvironmentError('Unsupported platform')

    result = []
    for port in ports:
        try:
            s = Serial(port)
            s.close()
            result.append(port)
            #print(port)
        except (OSError, serial.SerialException):
            pass
    return result

class SerialHandler():
	def __init__(self,_port='/dev/ttyACM0',_baud=115200,_timeout=0.5,_header=0x24,_end=0x21,_numDataBytes=2,_thLock=None):
		self.port =  str(_port)
		self.baud = _baud
		self.timeout = _timeout
		self.waiting = False
		self.serialPort = None
		self.dataQueue = deque()
		self.thLock = _thLock
		self.pkgHeader = _header
		self.pkgEnd = _end
		self.dataBytes = _numDataBytes

	def open(self):
		try:
			self.serialPort = Serial(self.port,self.baud,timeout=self.timeout)
			#waits for the serial port to open
			time.sleep(0.1)
			#self.serialPort = Serial('/dev/ttyUSB0',self.baud,timeout=self.timeout)
			if self.serialPort.is_open:
				self.serialPort.flushInput()
				self.serialPort.flushOutput()
				return True
			else:
				return False
		except:
			return False

	def close(self):
		try:
			self.serialPort.flushInput()
			self.serialPort.flushOutput()
			self.serialPort.close()
			if self.serialPort.is_open:
				return False
			else:
				return True
		except:
			return False

	def waitBytes(self,_numBytes):
		try:
			self.waiting = True
			t = Timer(self.timeout,self.getTimeout)
			t.start()
			numWaiting = 0
			while True:				
				if(self.serialPort.is_open):
					numWaiting = self.serialPort.in_waiting					
					if numWaiting < _numBytes and self.waiting is True:												
						pass
					else:
						break
				else:
					break
			if numWaiting >= _numBytes:
				t.cancel()
				return True
			else:
				return False
		except:
			return False

	def getTimeout(self):
		self.waiting = False

	def waitSTByte(self,_startByte):
		receivedByte = 0
		while True:
			ret = self.waitBytes(1)
			if ret:				
				receivedByte = ord(self.serialPort.read())
				#print(_startByte,receivedByte) #debugging
				if receivedByte == _startByte:
					return True
				else:
					return False
			else:
				return False

	def readPackage(self):
		# print('reading')
		header = self.waitSTByte(self.pkgHeader)
		#print(header)
		if header:            
			ret = self.waitBytes(self.dataBytes)
			if ret:				
				#data = map(ord,self.serialPort.read(self.dataBytes))
				data = self.serialPort.read(self.dataBytes)
                #print(data[) #debugging
				ret = self.waitBytes(1)
				if ret:
					end = ord(self.serialPort.read())
					# print(end,self.pkgEnd,end==self.pkgEnd)
					if end == self.pkgEnd:
						# print('ok')
						if self.thLock is not None:
							self.thLock.acquire()
						self.dataQueue.append(data)
						if self.thLock is not None:
							self.thLock.release()

	def to_int16(self,_MSB,_LSB):
		return c_short((_MSB<<8) + _LSB).value

	def to_float(self,_byteVector):
		binF = ''.join(chr(i) for i in _byteVector)
		return unpack('f',binF)[0]
#-------------------------------------------------------------------------------
if __name__ == '__main__':
    from threading import Lock
    from threadhandler import ThreadHandler
    import datetime

    l = Lock()
    flagRec = False
    counter = 0
    numsamples = None
    #s = SerialHandler(_header=0x24,_end=0x21,_numDataBytes=162,_thLock=l)
    s = SerialHandler(_port='COM3',_header=0x24,_end=0x21,_numDataBytes=34,_thLock=l)
    t0 = None
    t1 = None
    def run():
    	global s, l, flagRec, counter, t0, t1, numsamples
    	l.acquire()
    	n = len(s.dataQueue)
    	# print('samples',n)
    	for k in range(n):
    		if flagRec:
    			counter = counter + 1
    			# numsamples = [n,k,len(s.dataQueue)]
    			# if counter >= 2400*2:
    			# 	t1 = datetime.datetime.now()
    			# 	print(t0,t1,t1-t0)
    			# 	flagRec = False
    		data = s.dataQueue.popleft()
    		strdata = ''
    		# print('begin')
    		# print(n, k, data[0]<<8|data[1])
    		for i in range(0,4,2):
    			strdata += str(data[i]<<8|data[i+1]) + ' '
    			# print(i,data[i],data[i+1],data[i]<<8|data[i+1])
    		# print('end')
    		# print(strdata)
    	l.release()
    	time.sleep(0.001)

    s.open()
    time.sleep(1)
    print('ready')
    # print(s.serialPort.in_waiting)
    # s.serialPort.read(s.serialPort.in_waiting)
    t = ThreadHandler(s.readPackage)
    t.start()
    p = ThreadHandler(run)
    p.start()
    a = input()
    flagRec = True
    t0 = datetime.datetime.now()
    time.sleep(2)
    t1 = datetime.datetime.now()
    flagRec = False
    print('total samples:', counter, t1-t0, numsamples)
    a = input()
    t.kill()
    p.kill()
    s.close()
#-------------------------------------------------------------------------------
