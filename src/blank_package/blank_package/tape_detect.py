import cv2
import numpy as np

def get_postion(cx, cy, bh, bw):
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

def detect_parking(frame):
    img_width, img_height = frame.shape[0], frame.shape[1]
    buffer_width = img_width // 3
    buffer_height = img_height // 3

    # Hue Saturation Value
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    #getting red
    lower_red = np.array([0, 120, 100], np.uint8)
    upper_red = np.array([5, 255, 255], np.uint8)
    mask = cv2.inRange(hsv, lower_red, upper_red)


    #removing noise
    kernel = np.ones((10, 10), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    contours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    max_cnt, max_area = None, 0
    for cnt in contours:
        area = cv2.contourArea(cnt)

        if area > max_area and area > 500:
            max_area = area
            max_cnt = cnt

    if max_cnt is None:
        return False


    x, y, w, h = cv2.boundingRect(max_cnt)
    cx = x + w // 2
    cy = y + h // 2

    print(get_postion(cx, cy, buffer_width, buffer_height))
    cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
    cv2.imshow("mask", mask)
    #print(f"coordinates: {(cx, cy)}")

    return True

cap = cv2.VideoCapture(0)
while True:
    ret, frame = cap.read()
    detect_parking(frame)
    cv2.imshow("frame", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
q