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

    detected_pixels = cv2.countNonZero(mask)
    if detected_pixels > 35000:
        return True
    else:
        return False
