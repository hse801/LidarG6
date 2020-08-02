#-*- coding:utf-8 -*-
import sys
import time
import serial
import threading
import queue
from serial_Lidar import Protocol, ReaderThread
import struct
import math


# 프로토콜
class rawProtocal(Protocol):
    # 연결 시작시 발생
    def connection_made(self, transport):
        self.transport = transport
        self.running = True
        self.data = b''

    # 연결 종료시 발생
    def connection_lost(self, exc):
        self.transport = None
        print(exc)

    # 데이터 보낼 때 함수
    def write(self, data):
        print(data)
        self.transport.write(data)

    # 종료 체크
    def isDone(self):
        return self.running


class LidarG6(rawProtocal):

    def send_cmd(self, cmd):
        data = struct.pack('BB', 0xA5, cmd)
        self.write(data)

    def start(self): # Scan mode
        print('Start')
        self.send_cmd(0x60)

    def end(self): # Stop mode
        print('End')
        self.send_cmd(0x65)

    def data_received(self, data):

        print(data)
        if self.data: # 따로 출력되는 데이터 합치기
            data = self.data + data
        if len(data) < 10:
            data = self.data
            return
        print('Received Data', data)
        PH, CT, LSN, FSA, LSA, CS = struct.unpack('<HBBHHH', data[:10]) # '<'는 little endian
        #print('PH: ', PH, 'CT: ', CT, 'LSN: ', LSN, 'FSA: ', FSA, 'LSA: ', LSA, 'CS: ', CS)
        print(f'PH = {PH}, CT = {CT}, LSN = {LSN}, FSA = {FSA}, LSA = {LSA}, CS = {CS}') #fstring
        sampling_data = []
        for i in range(11, len(data)-1, 2):
            p = struct.unpack('<H', data[i:i + 2])
            sampling_data.append(p[0])
        print('Sampling Data = ', sampling_data)
        print('Length of Data = ', len(sampling_data))
        real_data = []
        for i in range(len(sampling_data)):
            real_data.append(sampling_data[i]*0.25)
        print('Distance= ', real_data)

        an_fsa = (data[2] >> 1) // 64
        an_lsa = (data[3] >> 1) // 64

        if an_fsa > an_lsa:
            an_fsa = 360 - an_fsa

        lsn = data[1] >> 8
        angle = []

        i = 0

        for i in range(5, len(data)):
            angle.append(((abs(an_fsa - an_lsa) / (lsn - 1)) * i) + an_fsa)

        ang_correct = []

        for cor in range(len(r4eal_data)):
            if real_data[cor] == 0:
                ang_correct.append(0)
            else:
                ang_correct.append(math.degrees(math.atan(21.8 * (155.3 - real_data[cor]) / (155.3 * real_data[cor]))))

        an_fsa = angle[0] + ang_correct[0]
        an_lsa = angle[len(real_data) - 1] + ang_correct[len(real_data) - 1]

        corr_angle = []
        i = 0

        for i in range(lsn):
            corr_angle.append(((abs(an_fsa - an_lsa) / (lsn - 1)) * i) + an_fsa)

        print('Angle = ', corr_angle)

        # ang_FSA = (FSA >> 1) / 64
        # ang_LSA = (LSA >> 1) / 64
        # angle = []
        #
        # for i in range(5, len(sampling_data)):
        #     angle.append(((abs(ang_FSA - ang_LSA) / (LSN - 1)) * i) + ang_FSA)


        # if real_data[0] == 0:
        #     AngCorrect_i = 0
        # else:
        #     AngCorrect_i = math.atan(21.8*(155.3-real_data[0])/(155.3*real_data[0]))
        #     Angle_FSA = (FSA[0:2] + LSN[1]) / 64 + AngCorrect_i
        #
        # if real_data[] == 0:
        #     AngCorrect_i = 0
        # else:
        #     AngCorrect_i = math.atan(21.8*(155.3-real_data[LSN])/(155.3*real_data[LSN]))
        #     Angle_LSA = (LSA[0:2] + FSA[3]) / 64 + AngCorrect_i
        #
        # for i in range(2, LSN - 1):
        #     if real_data[i] == 0:
        #         AngCorrect_i = 0
        #     else:
        #         AngCorrect_i = math.atan(21.8*(155.3-real_data[i])/(155.3*real_data[i]))
        #         Angle_i = (Angle_LSA - Angle_FSA) * (i - 1) / (LSN - 1) + Angle_FSA + AngCorrect_i
        return


# 포트 설정
PORT = 'COM13'
# 연결
ser = serial.serial_for_url(PORT, baudrate=512000, timeout=1)

# 쓰레드 시작
with ReaderThread(ser, LidarG6) as p:
    print('p', p)

    p.end()
    time.sleep(5)
    p.start()
    time.sleep(5)
    p.end()

    while p.isDone():
        time.sleep(1)