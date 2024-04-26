import cv2
import numpy as np

lower_red = np.array([0, 120, 120])
upper_red = np.array([10, 255, 255])
lower_green = np.array([50, 100, 100])
upper_green = np.array([70, 255, 255])
lower_yellow = np.array([30, 100, 100])
upper_yellow = np.array([40, 255, 255])

def draw_bounding_box(contours, color):
    for contour in contours:
        x, y, w, h = cv2.boundingRect(contour)
        min_size = 80
        if w >= min_size and h >= min_size:
            cv2.rectangle(frame, (x, y), (x + w, y + h), color, 2)


cap = cv2.VideoCapture(0)

while True:
    ret, frame = cap.read()
    
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    mask_red = cv2.inRange(hsv, lower_red, upper_red)  #����
    mask_green = cv2.inRange(hsv, lower_green, upper_green)  #�ʷ�
    mask_yellow = cv2.inRange(hsv, lower_yellow, upper_yellow) #���

    contours_red, _ = cv2.findContours(mask_red, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contours_green, _ = cv2.findContours(mask_green, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contours_yellow, _ = cv2.findContours(mask_yellow, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if len(contours_red) >= 1:
        max_contour_red = max(contours_red, key=cv2.contourArea)
        draw_bounding_box([max_contour_red], (0, 255, 255))
        

    if len(contours_green) >= 1:
        max_contour_green = max(contours_green, key=cv2.contourArea)
        draw_bounding_box([max_contour_green], (255, 255, 0))

    if len(contours_yellow) >= 1:
        max_contour_yellow = max(contours_yellow, key=cv2.contourArea)
        draw_bounding_box([max_contour_yellow], (255, 0, 255))
      
  
    cv2.imshow('frame', frame)
    
    # 'q' Ű�� ������ �����մϴ�.
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# ���� ĸó�� �����ϰ� â�� �ݽ��ϴ�.
cap.release()
cv2.destroyAllWindows()
