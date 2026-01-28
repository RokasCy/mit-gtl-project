import cv2

cap = cv2.VideoCapture(0)
while True:
    ret, frame = cap.read()

    # if frame not captured
    if not ret:
        print("failed to capture image")
        break

    # turns frame into black and white
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # displays the frame in a window
    cv2.imshow("Camera", gray)

    # cv2.waitKey(1) & 0xFF => removes removes extra bits, leaving only the first 8 bits to compare to the ascii code for q
    if cv2.waitKey(1) & 0xFF == ord('q'):
        print("Quitting...")
        break

cap.release()
cv2.destroyAllWindows()

# _, mask = cv2.threshold(gray, 50, 255, cv2.THRESH_BINARY_INV)
