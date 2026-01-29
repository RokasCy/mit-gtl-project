import cv2
import numpy as np

def get_postion(cx, cy, frame):
    imgh, imgw = frame.shape[0], frame.shape[1]
    bw = imgw // 3
    bh = imgh // 3

    position = ""
    if cy < bh:
        position += "top "
    elif cy < bh * 2:
        position += "middle "
    else:
        position += "bottom "

    if cx < bw:
        position += "left "
    elif cx < bw * 2:
        position += "middle "
    else:
        position += "right "

    return position

def draw_borders(frame):
    imgh, imgw = frame.shape[0], frame.shape[1]
    bw = imgw // 3
    bh = imgh // 3

    cv2.line(frame, (bw, 0), (bw, imgh), (0, 0, 255), 2)
    cv2.line(frame, (bw*2, 0), (bw*2, imgh), (0, 0, 255), 2)

    cv2.line(frame, (0, bh), (imgw, bh), (0, 0, 255), 2)
    cv2.line(frame, (0, bh*2), (imgw, bh*2), (0, 0, 255), 2)

def detect_parking(frame):
    # Hue Saturation Value
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    #getting red
    lower_red = np.array([0, 100, 150], np.uint8)
    upper_red = np.array([5, 255, 255], np.uint8)

    lower_red2 = np.array([170, 100, 150], np.uint8)
    upper_red2 = np.array([179, 255, 255], np.uint8)

    mask = cv2.inRange(hsv, lower_red, upper_red)
    mask2 = cv2.inRange(hsv, lower_red2, upper_red2)

    red_mask = cv2.bitwise_or(mask, mask2)

    #removing noise
    kernel = np.ones((5, 5), np.uint8)
    red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_OPEN, kernel)
    red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_CLOSE, kernel)

    draw_borders(frame)

    #finding contours
    contours, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    max_cnt = max(contours, key=cv2.contourArea, default=None)

    if max_cnt is None or cv2.contourArea(max_cnt) < 200:
        return (False, 0, 0)

    x, y, w, h = cv2.boundingRect(max_cnt)
    cx = x + w // 2
    cy = y + h // 2

    print(get_postion(cx, cy, frame))

    return (True, cx, cy)

