import cv2
import numpy as np
import time
from math import atan


def startyolo():
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
                        if label == "laptop":
                            print("x1 =",x)
                            print("w1 =",w)
                            if (x + w / 2) <= 960:
                                anglecheck = 35-atan(960-(x + w / 2) / 1371)
                            else:
                                anglecheck = 35+atan((x + w / 2) / 1371)

                cv2.imshow("Image", img)
                cv2.waitKey(0)
                cv2.destroyAllWindows()
