import socket
import serial
import time
import matplotlib.pyplot as plt
import copy
import numpy as np
import sys
import matplotlib.patches as patches
from collections import deque
from threading import Lock


class WidowX:
    def __init__(self):
        self.MODE = "cylindrical"
        self.SET_CYLINDRICAL_MODE_CMD = [0xff, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x30, 0xcf]
        self.SET_CARTESIAN_MODE_CMD = [0xff, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x20, 0xdf]
        self.GO_HOME_CMD = [0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x20, 0xdf]
        self.START_UP_CMD = [0xFF, 0x00, 0x00, 0x00, 0xC8, 0x00, 0xC8, 0x00, 0x00, 0x02, 0x00, 0x01, 0x00, 0x80, 0x00, 0x70, 0x7C]
        self.GO_SLEEP_CMD = [0xff, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x60, 0x9f]
        self.EMERGENCY_STOP_CMD = [0xff, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x11, 0xee]
        self.START_POSITION_CMD = [0xFF, 0x05, 0x0A, 0x00, 0x7F, 0x00, 0xCA, 0x00, 0x5A, 0x02, 0x00, 0x01, 0x00, 0x7D, 0x00, 0x00, 0xCD]
        
        self.POSICAO_INICIAL_X = 1290
        self.POSICAO_INICIAL_Y = 127
        self.POSICAO_INICIAL_Z = 202
        
        # LIMITES MÁXIMOS
        self.LIMITE_SUPERIOR_X = 0
        self.LIMITE_INFERIOR_X = 4059
        self.LIMITE_SUPERIOR_Y = 400
        self.LIMITE_INFERIOR_Y = 50
        self.LIMITE_SUPERIOR_Z = 350
        self.LIMITE_INFERIOR_Z = 20
        self.LIMITE_INFERIOR_WRIST_ANGLE = 60
        self.LIMITE_SUPERIOR_WRIST_ANGLE = 120
        self.LIMITE_INFERIOR_WRIST_ROTATE = 0
        self.LIMITE_SUPERIOR_WRIST_ROTATE = 1023
        self.LIMITE_INFERIOR_GRIPPER = 0
        self.LIMITE_SUPERIOR_GRIPPER = 512
        
        # LIMITES SEGUROS
        self.LIMITE_SUPERIOR_SEGURANCA_X = 2264
        self.LIMITE_INFERIOR_SEGURANCA_X = 991
        self.LIMITE_SUPERIOR_SEGURANCA_Y = 400
        self.LIMITE_INFERIOR_SEGURANCA_Y = 122
        self.LIMITE_SUPERIOR_SEGURANCA_Z = 350
        self.LIMITE_INFERIOR_SEGURANCA_Z = 139
        
        self.RANGE_MOVIMENTO_X = self.LIMITE_SUPERIOR_SEGURANCA_X - self.LIMITE_INFERIOR_SEGURANCA_X
        self.RANGE_MOVIMENTO_Y = self.LIMITE_SUPERIOR_SEGURANCA_Y - self.LIMITE_INFERIOR_SEGURANCA_Y
        self.RANGE_MOVIMENTO_Z = self.LIMITE_SUPERIOR_SEGURANCA_Z - self.LIMITE_INFERIOR_SEGURANCA_Z
        
        # VALORES DO PACKAGE DE COMUNICAÇÃO
        self.HEADER = 0xFF
        self.EXTENDED_BYTE = 0x00
        self.BUTTON_BYTE = 0x00
        self.TIME = 2000
        self.DELTA = 50
        self.FREQ_MAX = 80

        # VARIAVEIS DE STATUS
        self.isConnected = False

    def connect(self):
        try:
            self.comunicacaoSerial = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
            print(self.comunicacaoSerial)
            self.isConnected = self.startUp()
            if self.isConnected:
                 input("Pressione Enter para continuar...")
                #  print("SETANDO MODO CILINDRICO: ", self.SET_CYLINDRICAL_MODE_CMD)
                 self.sendCmdWaitForReply(self.SET_CYLINDRICAL_MODE_CMD)
                #  time.sleep(0.5)
                #  print("MOVENDO PARA POSIÇÃO INICIAL: ", self.START_POSITION_CMD)
                #  self.sendCmdWaitForReply(self.START_POSITION_CMD)
                 time.sleep(0.5)
            return self.isConnected
        except serial.SerialException as e:
            print("Erro ao conectar:", e)
            return False

    def startUp(self):
        try:
            for tentativa in range(10):
                self.comunicacaoSerial.write(self.START_UP_CMD)
                #print("Enviado comando de inicialização:", self.START_UP_CMD)
                
                time.sleep(0.5)  # Aguardar antes de ler a resposta para garantir que todos os bytes tenham sido recebidos

                bytes_to_read = self.comunicacaoSerial.in_waiting
                resposta = self.comunicacaoSerial.read(bytes_to_read)  # Leia os bytes disponíveis
                #print("Resposta recebida:", resposta)

                if b'Interbotix Robot Arm Online.' in resposta:
                    #print("WidowX está ativa")
                    return True
                time.sleep(0.5)
            print("Falha na comunicação após 10 tentativas.")
            return False
        except serial.SerialTimeoutException:
            print("Tempo limite excedido ao aguardar resposta.")
            return False
        except serial.SerialException as e:
            print("Erro na comunicação serial:", e)
            return False

    def sendCmdWaitForReply(self, cmd, flagWaitForReply=True):
        try:
            self.comunicacaoSerial.flushInput() #Resent Input buffer
            self.comunicacaoSerial.flushOutput() #Resent Output buffer
            # while self.comunicacaoSerial.out_waiting > 0:
            #     print("Aguardando enviar todos os bytes...")
            self.comunicacaoSerial.write(cmd)
            # while self.comunicacaoSerial.out_waiting > 0:
            #     print("Aguardando enviar todos os bytes...")
            # print("Enviando -> ", cmd)
            if flagWaitForReply:
                res = self.waitForReply()
                # self.verifyResponse(res)
        except serial.SerialException as e:
            print("Erro ao enviar comando:", e)

    def waitForReply(self):
        timeout = 1
        t0 = time.perf_counter()
        res = []
        while time.perf_counter() - t0 < timeout:
            if self.comunicacaoSerial.in_waiting:
                res.append(self.comunicacaoSerial.read())
                if len(res) == 5:
                    break
        return res

    def verifyResponse(self, res):
        if len(res) != 5:
            print("RESPOSTA INCOMPLETA", res)
        else:
            print("RESPOSTA COMPLETA", res)
            if res[1] != b'\x03':
                print("FIRMWARE NÃO CONFIGURADO PARA WIDOW_X")
            else:
                print("WIDOW_X -- CHECK")
            if res[2] == b'\x00':
                print("Cartesian - Normal Wrist")
            elif res[2] == b'\x02':
                print("Cylindrical - Normal Wrist")
            if res[3] == b'\x00':
                print("FIM DA MENSAGEM")

    def isRXBufferEmpty(self):
        return self.comunicacaoSerial.in_waiting == 0

    def stopEmergency(self):
        try:
            self.comunicacaoSerial.write(self.EMERGENCY_STOP_CMD)
            print(self.comunicacaoSerial.readline())
        except serial.SerialException as e:
            print("Erro ao enviar comando de emergência:", e)

    def goSleep(self):
        try:
            self.comunicacaoSerial.write(self.GO_SLEEP_CMD)
            print(self.comunicacaoSerial.readline())
        except serial.SerialException as e:
            print("Erro ao enviar comando de dormir:", e)

    def goHome(self):
        try:
            self.comunicacaoSerial.write(self.GO_HOME_CMD)
            print(self.comunicacaoSerial.readline())
        except serial.SerialException as e:
            print("Erro ao enviar comando de ir para casa:", e)

    def verificaLimites(self, x, y, z, gripper, wrist_angle, wrist_rot):
        x = min(max(x, self.LIMITE_INFERIOR_X), self.LIMITE_SUPERIOR_X)
        y = min(max(y, self.LIMITE_INFERIOR_Y), self.LIMITE_SUPERIOR_Y)
        z = min(max(z, self.LIMITE_INFERIOR_Z), self.LIMITE_SUPERIOR_Z)
        gripper = min(max(gripper, self.LIMITE_INFERIOR_GRIPPER), self.LIMITE_SUPERIOR_GRIPPER)
        wrist_angle = min(max(wrist_angle, self.LIMITE_INFERIOR_WRIST_ANGLE), self.LIMITE_SUPERIOR_WRIST_ANGLE)
        wrist_rot = min(max(wrist_rot, self.LIMITE_INFERIOR_WRIST_ROTATE), self.LIMITE_SUPERIOR_WRIST_ROTATE)
        return x, y, z, gripper, wrist_angle, wrist_rot

    def sendValue(self, x=2048, y=250, z=225, gripper=256, wrist=90, wrist_rot=512):
        # print("Enviando comando com posição")
        # print(f"x: {x} y: {y} z: {z} wrist_angle: {wrist} wrist_rot: {wrist_rot} gripper: {gripper}")
        x, y, z, gripper, wrist, wrist_rot = self.verificaLimites(x, y, z, gripper, wrist, wrist_rot)
        posicoes = [x, y, z, wrist, wrist_rot, gripper]
        package = self.preparePackage(posicoes)
        self.sendCmdWaitForReply(package, False)

    def preparePackage(self, posicoes):
        package = [self.HEADER]
        for pos in posicoes:
            highByte = (int(pos) >> 8) & 0xFF
            lowByte = int(pos) & 0xFF
            package.append(highByte)
            package.append(lowByte)
        package.append(self.DELTA)
        package.append(self.BUTTON_BYTE)
        package.append(self.EXTENDED_BYTE)
        package.append(self.checkSum(package))
        # print(package)
        return package

    def checkSum(self, package):
        soma = sum(package[1:-1])
        inv_check_sum = int(soma) & 0xFF
        checksum = 255 - inv_check_sum
        return checksum


# widowx = WidowX()
# widowx.connect()