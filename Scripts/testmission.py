import cv2 as cv

capture = cv.VideoCapture(0, cv.CAP_DSHOW)
capture.set(3, 640)
capture.set(4, 480)

while True:
    #read frame
    success, frame = capture.read()
    frame = cv.flip(frame, 2)

    #Break
    cv.imshow("test", frame)
    if cv.waitKey(1) & 0xFF == ord('q'):
        break
capture.release()
cv.destroyAllWindows()