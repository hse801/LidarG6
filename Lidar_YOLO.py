from threading import Thread
import cv2
import numpy as np
import time
import serial
from math import atan, pi, floor
import matplotlib.pyplot as plt
import math
from yolo import *
# from yolo import startyolo

dist_i = 0
# anglecheck = 0
# anglecheck2 = 0
# dist_sum = 0
angcheckdone = []
distcheckdone = []
angcheckdone2 = []
distcheckdone2 = []
num = 0
num_Mean = 0
num2 = 0
num_Mean2 = 0


def read_Lidar():
    def plot_lidar(distdict):
        x = [0 for i in range(360)]
        y = [0 for i in range(360)]
        for angle in range(0, 360):
            x[angle] = distdict[angle] * math.cos(math.radians(angle))
            y[angle] = distdict[angle] * math.sin(math.radians(angle))

        plt.figure(1)
        plt.cla()
        plt.ylim(-3000, 3000)
        plt.xlim(-3000, 3000)
        plt.scatter(x, y, c='r', s=8)
        plt.pause(0.001)

    def _CheckSum(data):
        try:
            ocs = _HexArrToDec((data[6], data[7]))
            LSN = data[1]
            cs = 0x55AA ^ _HexArrToDec((data[0], data[1])) ^ _HexArrToDec((data[2], data[3])) ^ _HexArrToDec(
                (data[4], data[5]))
            for i in range(0, 2 * LSN, 2):
                cs = cs ^ _HexArrToDec((data[8 + i], data[8 + i + 1]))

            if cs == ocs:
                return True
            else:
                return False
        except Exception as e:
            return False

    def _HexArrToDec(data):
        littleEndianVal = 0
        for i in range(0, len(data)):
            littleEndianVal = littleEndianVal + (data[i] * (256 ** i))
        return littleEndianVal

    def _AngleCorr(dist):
        if dist == 0:
            return 0
        else:
            return atan(21.8 * ((155.3 - dist) / (155.3 * dist))) * (180 / pi)

    def _Calculate(d):
        # global dist_sum
        global anglecheckdone
        global distcheckdone
        global anglecheckdone2
        global distcheckdone2
        ddict = []
        LSN = d[1]
        Angle_fsa = ((_HexArrToDec((d[2], d[3])) >> 1) / 64.0)
        Angle_lsa = ((_HexArrToDec((d[4], d[5])) >> 1) / 64.0)

        if Angle_fsa < Angle_lsa:
            Angle_diff = Angle_lsa - Angle_fsa
        else:
            Angle_diff = 360 + Angle_lsa - Angle_fsa

        for i in range(0, 2 * LSN, 2):
            global dist_i
            dist_i = _HexArrToDec((d[8 + i], d[8 + i + 1])) / 4
            # dist_i = 2
            Angle_i_tmp = ((Angle_diff / float(LSN)) * (i / 2)) + Angle_fsa

            #####3rint(dist_i)
            if Angle_i_tmp > 360:
                Angle_i = Angle_i_tmp - 360
            elif Angle_i_tmp < 0:
                Angle_i = Angle_i_tmp + 360
            else:
                Angle_i = Angle_i_tmp

            Angle_i = Angle_i + _AngleCorr(dist_i)
            # print(Angle_i)
            ddict.append((dist_i, Angle_i))

            # angcheckdone = []
            # distcheckdone = []
            # if Angle_i >200:
            # Angle_i - 100 <= anglecheck + 149 <= Angle_i + 100

            # print('Angle check1: ', startyolo.anglecheck)
            # print('Angle check2: ', startyolo.anglecheck2)
            if (Angle_i - 16 <= startyolo.anglecheck + 149 <= Angle_i + 16) and (134 <= Angle_i <= 234):
            # +-16의 오차 / +149를 하여 욜로와 라이다의 각도를 맞춰준다
                angcheckdone.append(Angle_i)
                distcheckdone.append(dist_i * 2)
            else:
                angcheckdone.append(0)
                distcheckdone.append(0)
            print('Angle check = ', angcheckdone)
            print('Distance check = ', distcheckdone)
            if (Angle_i - 16 <= startyolo.anglecheck2 + 149 <= Angle_i + 16) and (134 <= Angle_i <= 234):
                 angcheckdone2.append(Angle_i)
                 distcheckdone2.append(dist_i * 2)
            else:
                 angcheckdone2.append(0)
                 distcheckdone2.append(0)
            print('Angle check = ', angcheckdone2)
            print('Distance check = ', distcheckdone2)

            if i == (LSN - 1) * 2:
                nonzero_distcheckdone = [float(v) for v in distcheckdone if v > 0]
                nonzero_distcheckdone2 = [float(v) for v in distcheckdone2 if v > 0]

                # mean_dist = (dist_sum / len(distcheckdone))
            mean_dist = sum(nonzero_distcheckdone) / len(nonzero_distcheckdone)
            mean_dist2 = sum(nonzero_distcheckdone2) / len(nonzero_distcheckdone2)
            print('Distance Mean = ', mean_dist)
            print('Distance Mean2 = ', mean_dist2)

            global num
            global num2
            global num_Mean
            global num_Mean2

            num += 1
            num2 += 1
            num_Mean += mean_dist
            num_Mean2 += mean_dist2
            print('num = ', num)
            print('num2 = ', num2)
            print('total_mean = ', num_Mean)
            print('total_mean2 = ', num_Mean2)
            print('avg_mean =', num_Mean / num)
            print('avg_mean2 =', num_Mean2 / num2)
            # print(len(distcheckdone))
            # print('LSN = ', LSN)
            # print('ddict = ', ddict)
            return ddict

    def _Mean(data):
        length_of_data_without_zero = sum([i != 0 for i in data])
        if len(data) > 0 and length_of_data_without_zero != 0:
            #        return int(sum(data)/len(data)) # original By ydlidar
            return float(sum(data) / length_of_data_without_zero)  # modified for remove zero value
        return 0

    def code(ser):
        data1 = ser.read(4000)
        data2 = data1.split(b"\xaa\x55")[1:-1]

        distdict = {}
        for i in range(0, 360):
            distdict.update({i: []})
        for i, e in enumerate(data2):
            try:
                if e[0] == 0:
                    if _CheckSum(e):
                        d = _Calculate(e)
                        for ele in d:
                            angle = floor(ele[1])
                            if 0 <= angle < 360:
                                distdict[angle].append(ele[0])
            except Exception as e:
                pass
        for i in distdict.keys():
            distdict[i] = _Mean(distdict[i])
        yield distdict

    def main():
        # Open Serial
        ser = serial.Serial(port='COM13', baudrate=512000)
        ser.isOpen()

        # Scan start
        values = bytearray([int('a5', 16), int('60', 16)])
        ser.write(values)

        for i in range(10):
            angle_data = code(ser)
            plot_lidar(next(angle_data))

        # Scan End
        values = bytearray([int('a5', 16), int('65', 16)])
        ser.write(values)

        # Close Serial
        ser.close()

    if __name__ == '__main__':
        main()


t1 = Thread(target=startyolo, args="")
t2 = Thread(target=read_Lidar(), args="")

t1.start()
t2.start()