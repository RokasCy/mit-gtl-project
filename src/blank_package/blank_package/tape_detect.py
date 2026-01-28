import cv2
import numpy as np

def detect_parking(frame):

    # Hue Saturation Value
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    #getting red
    lower_red = np.array([0, 120, 120], np.uint8)
    upper_red = np.array([5, 255, 255], np.uint8)
    mask = cv2.inRange(hsv, lower_red, upper_red)


    #removing noise
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    contours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    for cnt in contours:
       area = cv2.contourArea(cnt)
       if area > 5000:  # only keep large blobs
           #x, y, w, h = cv2.boundingRect(cnt)
           #cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
           return True

    return False
