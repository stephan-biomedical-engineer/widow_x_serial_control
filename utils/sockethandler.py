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
# Description: This file handles socket communication
#-------------------------------------------------------------------------------
'''
#-------------------------------------------------------------------------------
#-------------------------------------------------------------------------------
#LIBRARIES
import socket
from threading import Thread, Lock
import socketserver
from collections import deque
from copy import copy
#-------------------------------------------------------------------------------
gLock = Lock()
gDataQueue = deque()
#-------------------------------------------------------------------------------
class SocketServerHandler():
    def __init__(self,_ip,_port):
        #create the server
        self.server = socketserver.TCPServer((_ip,_port),ThreadedTCPRequestHandler)
        #attach the request handler to a separate thread
        self.thServer = Thread(target=self.server.serve_forever)
        self.thServer.daemon = True #kill the server thread along the main thread
        self.thServer.start() #start listening
    #retrieving the data queue will make it empty afterwards. the application
    #side should, therefore, consume this queue or data will be lost
    def getDataQueue(self):
        global gLock,gDataQueue
        gLock.acquire() #lock for thread-safe access
        q = copy(gDataQueue) #copy the queue
        gDataQueue.clear() #clear the queue
        gLock.release() #release the lock
        return q #return a copy of the queue
class ThreadedTCPRequestHandler(socketserver.BaseRequestHandler):
    def handle(self):
        global gLock,gDataQueue
        #read incoming data
        data = self.request.recv(2000)
        #using lock for thread-safe access
        gLock.acquire()
        gDataQueue.append(data) #append the received data to the queue
        #debugging
        #print('\nreceived!',data,'\n')
        gLock.release() #release lock
class SocketClientHandler():
    def __init__(self,_ip,_port):
        self.client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.client.connect((_ip,_port))
    def write(self,_data):
        if isinstance(_data,bytearray):
            self.client.send(_data)
            return True
        else:
            return False
#-------------------------------------------------------------------------------
if __name__=='__main__':
    #create a server
    s = SocketServerHandler('127.0.0.1',8888)
    #create a client
    c = SocketClientHandler('127.0.0.1',8888)
    #write data to the server
    c.write(bytearray([0x24,0x50,0x49,0x21]))
    #wait for user input to finish
    input('press ENTER to finish...')
