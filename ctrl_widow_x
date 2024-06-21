
import socket
import serial as sr
import time
import pygame

import matplotlib.pyplot as plt
import copy
import numpy as np
import sys
import matplotlib.patches as patches

from collections import deque
sys.path.append('utils/')
from threading import Lock
from threadhandler import ThreadHandler





def main():
    global comunicacaoSerial,flagConnected
    flagConnected = False
    pos_atual_x = 110
    pos_atual_y = 115
    comunicacaoSerial = sr.Serial('/dev/ttyUSB0',
                                    38400,
                                    stopbits = sr.STOPBITS_ONE,
                                    bytesize = sr.EIGHTBITS,
                                    parity = sr.PARITY_NONE,)

    pack = sendValue(0,250,225)
    cmdList = [
            #[0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x20, 0xDF], #cartesian mode wrist 90 degree
            [0xff, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x30, 0xcf], #Set 3D Cylindrical mode / straight wrist and go to home
            #[0xFF, 0x02, 0x00, 0x00, 0xFA, 0x00, 0xE1, 0x00, 0x5A, 0x02, 0x00, 0x01, 0x00, 0x7D, 0x00, 0x00, 0x48],
            #pack,
            #[0xff, 0x1, 0x9c, 0x0, 0x96, 0x0, 0x96, 0x0, 0x5a, 0x2, 0x0, 0x0, 0x0, 0x80, 0x0, 0x0, 0x5a], #move x left
            #[0xff, 0x2, 0x0, 0x0, 0x96, 0x0, 0x96, 0x0, 0x5a, 0x2, 0x0, 0x2, 0x0, 0x80, 0x0, 0x0, 0xf3] #open gripper
            #[255, 0, 0, 0, 250, 0, 225, 0, 0, 2, 0, 1, 0, 125, 0, 0, 164],
            #[0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x60, 0x9F]
          ]
    #for i in [10,20,30,40,30,20,10]:
    #    cmdList.append(sendValue(i,250,225))


    input("Press Enter to continue...")
    #sendValue(1,1,1)
    #goHome()
    startUp()
    if flagConnected:
        for cmd in cmdList:
            sendCmdWaitForReply(cmd)
            time.sleep(1)
    input("Press Enter to continue...")
    cmdList = []
    cmdList.append(sendValue(2048,250,225))
    cmdList.append(sendValue(2088,250,225))
    cmdList.append(sendValue(2118,250,225))
    cmdList.append(sendValue(2148,250,225))
    cmdList.append(sendValue(2188,250,225))
    if flagConnected:
        for cmd in cmdList:
            sendCmdWaitForReply(cmd)
            time.sleep(1)

def isRXBufferEmpty(serial):
    qtdeInput = serial.in_waiting
    if qtdeInput > 0:
        return False
    else:
        return True


def sendCmdWaitForReply(cmd):
    CHECK_RESPONSE = "\xff\x03\x00\x00\xfc"
    flagResposta = False
    t0 = time.perf_counter()
    timeout = 1
    interacao = 0
    ret = b'0x00'
    while not flagResposta:
        while comunicacaoSerial.out_waiting > 0:
            print("aguardando enviar todos os bytes...")
        comunicacaoSerial.write(cmd)
        while comunicacaoSerial.out_waiting > 0:
            print("aguardando enviar todos os bytes...")
        print("enviando -> ", cmd)
        res = []
        time.sleep(0.5)
        while isRXBufferEmpty(comunicacaoSerial) and not(time.perf_counter() - t0 > timeout):
            time.sleep(0.001)
        while not isRXBufferEmpty(comunicacaoSerial) and ret != b'\xfc':
            ret = comunicacaoSerial.read()
            res.append(ret)
            time.sleep(0.01)
        print(res)
        flagResposta = True
        #if ret == b'\xff':
        #    flagResposta = True
        #    print("Resp. WidX -> ",ret)
        #elif interacao>10:
        #    print("comunicação falhou")
        #interacao = interacao + 1
        #print("---", interacao)


def startUp():
    global comunicacaoSerial,flagConnected

    a = [0xFF, 0x00, 0x00, 0x00, 0xC8, 0x00, 0xC8, 0x00, 0x00, 0x02, 0x00, 0x01, 0x00, 0x80, 0x00, 0x70, 0x7C]
    interacao = 0
    while not flagConnected:
        comunicacaoSerial.write(a)
        ret = comunicacaoSerial.readline()
        if ret == b'\xff\x03\x00\x00\xfcInterbotix Robot Arm Online.\r\n':
            flagConnected = True
            print("WidowX live")
        elif interacao>10:
            print("comunicação falhou")
        interacao = interacao + 1



def goHome():
    global comunicacaoSerial
    a = [0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x20, 0xdf]
    comunicacaoSerial.write(a)
    print(comunicacaoSerial.readline())


#LIMITES MÁXIMOS
LIMITE_SUPERIOR_X = 0
LIMITE_INFERIOR_X = 4059
LIMITE_SUPERIOR_Y = 400
LIMITE_INFERIOR_Y = 50
LIMITE_SUPERIOR_Z = 350
LIMITE_INFERIOR_Z = 20
LIMITE_INFERIOR_WRIST_ANGLE = 60
LIMITE_SUPERIOR_WRIST_ANGLE = 120
LIMITE_INFERIOR_WRIST_ROTATE = 0
LIMITE_SUPERIOR_WRIST_ANGLE = 1023
LIMITE_INFERIOR_GRIPPER = 0
LIMITE_SUPERIOR_GRIPPER = 512
#LIMITES SEGUROS
###NECESSARIO ANALISE

def verificaLimites(x,y,z):
    if(x < LIMITE_INFERIOR_X):
        x = LIMITE_INFERIOR_X
    elif(x>LIMITE_SUPERIOR_X):
        x = LIMITE_SUPERIOR_X
    if(y < LIMITE_INFERIOR_Y):
        y = LIMITE_INFERIOR_Y
    elif(y>LIMITE_SUPERIOR_Y):
        y = LIMITE_SUPERIOR_Y
    if(z < LIMITE_INFERIOR_Z):
        z = LIMITE_INFERIOR_Z
    elif(z>LIMITE_SUPERIOR_Z):
        z = LIMITE_SUPERIOR_Z
    return x,y,z

def sendValue(x=0,y=250,z=225):
    #https://learn.trossenrobotics.com/arbotix/arbotix-communication-controllers/31-arm-link-reference.html
    global comunicacaoSerial
    print("enviando comando com posição")
    #x,y,z = verificaLimites(x,y,z)
    gripper = 256
    wrist = 90
    wrist_rot = 512
    posicoes = [x,y,z,wrist,wrist_rot,gripper]
    package = preparePackage(posicoes)
    return package


def preparePackage(posicoes):
    HEADER = 0xFF
    EXTENDED_BYTE = 0x00 #move arm to position
    BUTTON_BYTE = 0x00 #do nothing
    TIME = 2000 #time in miliseconds
    package = []
    package.append(HEADER)
    for pos in posicoes:
        highByte = (int(pos) >> 8) & 0xFF
        lowByte = int(pos) & 0xFF
        package.append(highByte)
        package.append(lowByte)

    package.append(125)#delta value from the package. Range: 0 - 255 | 0 - 2000 miliseconds
    package.append(BUTTON_BYTE)
    package.append(EXTENDED_BYTE)
    package.append(checkSum(package))
    print(package)
    return package


def checkSum(package):
    soma = sum(package[1:-1])
    inv_check_sum = int(soma) & 0xFF
    checksum = 255 - inv_check_sum
    return checksum


if __name__ == "__main__":

    # 0xff 0x0 0x0 0x0 0xfa 0x0 0xe1 0x0 0x0 0x2 0x0 0x1 0x0 0x7d 0x0 0x0 0xa4
    #[255, 0,   0,  0, 250,  0, 225,  0,  0,  2,  0,  1,  0, 125,  0,  0, 164]

    #enviar o X para 554
    #0xFF 0x02 0x2A 0x00 0xFA 0x00 0xE1 0x00 0x5A 0x02 0x00 0x01 0x00 0x7D 0x00 0x00 0x1E
    #T ->
    #[255, 2, 42, 0, 250, 0, 225, 0, 90, 2, 0, 1, 0, 125, 0, 0, 161]

    #[255, 2, 42, 0, 250, 0, 225, 0, 0, 2, 0, 1, 0, 125, 0, 0, 120]

	main()
