import cv2
import numpy as np

import torch

from models.experimental import attempt_load
from utils.general import non_max_suppression, scale_coords

def get_direction(frame):
    
    direction = None

    height, width, _ = frame.shape

    roi = frame[int(height * 2 / 3):, :]

    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    
    lower_yellow = np.array([30, 100, 100], dtype=np.uint8)
    upper_yellow = np.array([40, 255, 255], dtype=np.uint8)
    mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if len(contours) >= 1:
        max_contour = max(contours, key=cv2.contourArea)
        cv2.drawContours(roi, [max_contour], -1, (0, 255, 0), 2)

        M = cv2.moments(max_contour)

        if M["m00"] != 0:
            cx = int(M["m10"] / M["m00"])
            center_line = width // 2

            print(cx)
        
            if cx < center_line - 55:
                direction = 2
            elif cx > center_line + 55:
                direction = 3
            else:
                direction = 1

    else:
        direction = 4

    return direction

class TrafficInfo:
    def __init__(self):
        self.temp = 123

        self.weights = '/home/er/myagv_ros/src/linetracing/scripts/yolov7-tiny.pt'
        self.device = 'cpu'

        self.model = attempt_load(self.weights, map_location=self.device)

        self.names = self.model.module.names if hasattr(self.model, 'module') else self.model.names
        self.colors = [[np.random.randint(0, 255) for _ in range(3)] for _ in self.names]

    def get_go_stop_yolov7(self, frame):

        def frame_to_tensor(frame):
            img = torch.from_numpy(frame).to(self.device)
            img = img.float()  # uint8 to fp16/32
            img /= 255.0  # 0 - 255 to 0.0 - 1.0
            img = img.permute(2, 0, 1).unsqueeze(0)

            return img

        ret = False

        img = frame_to_tensor(frame)

        with torch.no_grad():
            pred = self.model(img, augment=False)[0]

        pred = non_max_suppression(pred, 0.25, 0.45, classes=None, agnostic=False)

        for det in pred:
            if len(det):
                det[:, :4] = scale_coords(img.shape[2:], det[:, :4], frame.shape).round()

                for *xyxy, conf, cls in reversed(det):
                    label = f'{self.names[int(cls)]}'
                    if 'traffic' in label:
                        print(label)
                        ret = True

        return ret

    def get_go_stop_cv2(self, frame):

        ret = False

        threshold = {'red': [np.array([0, 120, 120]), np.array([5, 255, 255])],
                     'green': [np.array([50, 100, 100]), np.array([70, 255, 255])]}
        
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        masks = list()
        for k, v in threshold.items():
            masks.append(cv2.inRange(hsv, v[0], v[1]))

        contours = list()
        for mask in masks:
            contours.append(cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE))

        for i, (contour, _) in enumerate(contours):
            if len(contour) > 0 and i == 0:
                ret = True

            else:
                ret = False

        return ret
