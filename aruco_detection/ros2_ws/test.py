import cv2

cap = cv2.VideoCapture("dev/video0")

while 1 != 2:
    ret, frame = cap.read()
    if not ret: 
        print('fuck')
        break
    cv2.imshow('blinger', frame)
    cv2.waitKey(1)