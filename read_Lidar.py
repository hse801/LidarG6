#-*- coding:utf-8 -*-
import sys
import time
import serial
import threading
import queue
from serial_Lidar import Protocol, ReaderThread
import struct


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
        # print(len(data))
        # sampling_data = []
        # for i in range(11, len(data)):
        #     sampling_data.append(list(struct.unpack('<H', data[i])))
        #
        # print('Sampling Data = ', sampling_data)

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