import cv2
import numpy as np
import time
from math import atan

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
                            # num1이 사진의 개수, num2가 마지막 사진에서 label이 person인 bounding box 개수
                            # num3는 마지막 사진에서 label이 bottle인 bounding box 개수
                            # 배열에 나머지 값은 0이고 조건에 해당될때만 anglecheck 값을 할당
                            # 배열을 장애물 별로 따로 저장하여 라이다로 넘긴다
                            # num1 - 1 번째 행만 가져와야함

                            if (x + w / 2) <= 960:
                                person[num1, num2] = 35 - atan(960 - (x + w / 2) / 1371)
                            else:
                                person[num1, num2] = 35 + atan((x + w / 2) / 1371)

                            num2 += 1

                        if label == "bottle":

                            global bottle
                            global num3

                            if (x + w / 2) <= 960:
                                bottle[num1, num3] = 35 - atan(960 - (x + w / 2) / 1371)
                            else:
                                bottle[num1, num3] = 35 + atan((x + w / 2) / 1371)

                            num3 += 1

                cv2.imshow("Image", img)
                cv2.waitKey(0)
                cv2.destroyAllWindows()
                num1 += 1

    # startyolo()
    np.save('personlist', person)
    print("\n")
    np.save('bottlelist', bottle)