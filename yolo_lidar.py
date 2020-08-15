#from threading import Thread
import cv2
import numpy as np
import time
import serial
import math
from math import atan, pi, floor
import matplotlib.pyplot as plt
import math
#from yolo import startyolo
#from yolo import *
#import startyolo.anglecheck


dist_i = 0
#anglecheck = 0
#anglecheck2 = 0
# dist_sum = 0
angcheckdone = []
distcheckdone = []

ACD_sized = np.array([[0.0 for col in range(200)] for row in range(10)])
DCD_sized = np.array([[0.0 for col in range(200)] for row in range(10)])
sum_sized = [0 for i in range(10)]
#angcheckdone2 = []
#distcheckdone2 = []
num = 0
total_mean = 0
#num2 = 0
#num_Mean2 = 0
num1 = 0
num2 = 0
num3 = 0
person = np.array([[0.0 for col in range(50)] for row in range(50)])
bottle = np.array([[0.0 for col in range(50)] for row in range(50)])

def startyolo():
    global num1
    cap = cv2.VideoCapture(1, cv2.CAP_DSHOW)
    cap.set(3, 1920)
    cap.set(4, 1080)
    ret, image = cap.read()
    cv2.imwrite("C:/Users/user/1.jpg", image)
    cap.release()
    img = cv2.imread("C:/Users/user/1.jpg")

    # Yolo 로드
    net = cv2.dnn.readNet("C:/Users/user/yolov3.weights", "C:/Users/user/yolov3.cfg")
    classes = []
    with open("C:/Users/user/coco.names", "r") as f:
        classes = [line.strip() for line in f.readlines()]
    layer_names = net.getLayerNames()
    output_layers = [layer_names[i[0] - 1] for i in net.getUnconnectedOutLayers()]
    colors = np.random.uniform(0, 255, size=(len(classes), 3))

    # img = cv2.resize(img, None, fx=0.4, fy=0.4)
    height, width, channels = img.shape

    # Detecting objects
    blob = cv2.dnn.blobFromImage(image, 0.00392, (320, 320), (0, 0, 0), True, crop=False)
    net.setInput(blob)
    outs = net.forward(output_layers)

    # 정보를 화면에 표시
    class_ids = []
    confidences = []
    boxes = []
    for out in outs:
        for detection in out:
            scores = detection[5:]
            class_id = np.argmax(scores)
            confidence = scores[class_id]
            if confidence > 0.5:
                # Object detected
                center_x = int(detection[0] * width)
                center_y = int(detection[1] * height)
                w = int(detection[2] * width)
                h = int(detection[3] * height)
                # 좌표
                x = int(center_x - w / 2)
                y = int(center_y - h / 2)
                boxes.append([x, y, w, h])
                confidences.append(float(confidence))
                class_ids.append(class_id)

                # 노이즈 제거
                indexes = cv2.dnn.NMSBoxes(boxes, confidences, 0.5, 0.4)

                font = cv2.FONT_HERSHEY_PLAIN
                for i in range(len(boxes)):
                    if i in indexes:
                        x, y, w, h = boxes[i]
                        label = str(classes[class_ids[i]])
                        color = colors[i]
                        cv2.rectangle(img, (x, y), (x + w, y + h), color, 2)
                        cv2.putText(img, label, (x, y + 30), font, 3, color, 3)
                        if label == "person":

                            global person
                            global num2

                            if (x + w / 2) <= 960:
                                person[num1, num2] = 35 - atan(960 - (x + w / 2) / 1371)
                            else:
                                person[num1, num2] = 35 + atan((x + w / 2) / 1371)

                            num2 += 1

                        # if label == "bottle":
                        #
                        #     global bottle
                        #     global num3
                        #
                        #     if (x + w / 2) <= 960:
                        #         bottle[num1, num3] = 35 - atan(960 - (x + w / 2) / 1371)
                        #     else:
                        #         bottle[num1, num3] = 35 + atan((x + w / 2) / 1371)
                        #
                        #     num3 += 1

                cv2.imshow("Image", img)
                cv2.waitKey(0)
                cv2.destroyAllWindows()
                num1 += 1

    np.save('personlist', person)
    # np.save('bottlelist', bottle)


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
        global angcheckdone
        global distcheckdone
        # global angcheckdone2
        # global distcheckdone2
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

            # print(anglecheck)
            # print(anglecheck2)

            for j in range(len(anglecheck) - 1):
                if (Angle_i - 20 <= anglecheck[j] + 149 <= Angle_i + 20) and (104 <= Angle_i <= 264):
                    angcheckdone.append(Angle_i)
                    distcheckdone.append(dist_i)
                    # ACD_sized[j,:] = angcheckdone
                    DCD_sized[j][i] = dist_i
                    print("LSN=", LSN)
                    print("i =", i)
                else:
                    angcheckdone.append(0)
                    distcheckdone.append(0)

                print('Angle check = ', angcheckdone)
                print('Distance check = ', distcheckdone)
                # if (Angle_i - 16 <= anglecheck2 + 149 <= Angle_i + 16) and (134 <= Angle_i <= 234):
                # angcheckdone2.append(Angle_i)
                # distcheckdone2.append(dist_i * 2)
                # else:
                # angcheckdone2.append(0)
                # distcheckdone2.append(0)
                # print('Angle check = ', angcheckdone2)
                # print('Distance check = ', distcheckdone2)

                # ACD_sized[j, :] = angcheckdone
                # DCD_sized[j, :] = distcheckdone

                # angcheckdone = []
                # distcheckdone = []

                # for k in range(len(anglecheck) - 1):
                global sum_sized
                sum_sized[j] = sum(DCD_sized[j])
        # print("sum_sized=",sum_sized)

        min_index = np.argmin(sum_sized)

        print("min_index = ", min_index)

        distcheckdone_minindex = DCD_sized[min_index] # 인덱스 수정

        if i == (LSN - 1) * 2:
            nonzero_distcheckdone = [float(v) for v in distcheckdone_minindex if v > 0]
            # nonzero_distcheckdone2 = [float(v) for v in distcheckdone2 if v > 0]

            # mean_dist = (dist_sum / len(distcheckdone))
        mean_dist = sum(nonzero_distcheckdone) / len(nonzero_distcheckdone)
        # mean_dist2 = sum(nonzero_distcheckdone2) / len(nonzero_distcheckdone2)
        print('Distance Mean = ', mean_dist)
        # print('Distance Mean2 = ', mean_dist2)

        global num
        # global num2
        global total_mean
        # global num_Mean2

        num += 1
        # num2 += 1
        total_mean += mean_dist
        # num_Mean2 += mean_dist2
        print('num = ', num)
        # print('num2 = ', num2)
        print('total_mean = ', total_mean)
        # print('total_mean2 = ', num_Mean2)
        print('avg_mean =', total_mean / num)
        # print('avg_mean2 =', num_Mean2 / num2)
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
        data1 = ser.read(6000)
        data2 = data1.split(b"\xaa\`x55")[1:-1]

        distdict = {}
        for i in range(0, 360):
            distdict.update({i: []})
        for i, e in enumerate(data2):
            try:
                if e[0] == 0:
                    if (_CheckSum(e)):
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


startyolo()

angcheck_person_cut = person[num1 - 1, :]

# anglecheckf[anglecheckf.nonzero()]

# print(anglecheckf)


# print("anglecheck")
# print(anglecheck)

# anglecheck4 = np.load('bottlelist2.npy')
# print(anglecheck)
# print("\n")
# print(anglecheck4)

anglecheck = angcheck_person_cut[angcheck_person_cut > 0]
print("anglecheck")
print(anglecheck)

read_Lidar()

# print(sum_sized)

print(np.argmin(sum_sized))

print(DCD_sized)

# t1 = Thread(target=startyolo, args=(""))
# t2 = Thread(target=read_Lidar(), args=(""))

# t1.start()
# t2.start()

print("sum_sized=", sum_sized)
print("sum=", sum(DCD_sized[0]))