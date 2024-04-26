import cv2
import numpy as np

lower_yellow = np.array([30, 120, 130])
upper_yellow = np.array([40, 255, 255])



cap = cv2.VideoCapture(0)

while True:
    ret, frame = cap.read()

    height, width, _ = frame.shape

    roi = frame[int(height * 2 / 3):, :]
    
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    mask_yellow = cv2.inRange(hsv, lower_yellow, upper_yellow)

    contours, _ = cv2.findContours(mask_yellow, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if len(contours) >= 1:
        max_contour = max(contours, key=cv2.contourArea)
        cv2.drawContours(roi, [max_contour], -1, (0, 255, 0), 2)

        M = cv2.moments(max_contour)

        if M["m00"] != 0:
            cx = int(M["m10"] / M["m00"])
            center_line = width // 2

            print(cx)


    cv2.imshow('frame', frame)
    
    # 'q' Ű�� ������ �����մϴ�.
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# ���� ĸó�� �����ϰ� â�� �ݽ��ϴ�.
cap.release()
cv2.destroyAllWindows()

