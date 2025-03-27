import cv2 as cv
from imutils.video import VideoStream
import imutils

def show_color(event, x, y, flags, param):
    if event == cv.EVENT_MOUSEMOVE:
        color = frame[y, x]
        #print("X: ", x, "Y: ", y, "Color: ", color)

def edge_cam():
    global frame
    cap = VideoStream(src=0).start()
    edgeshow = -1

    while True:
        if cv.waitKey(20) == ord(' '):
            edgeshow = edgeshow * -1

        frame = cap.read()
        frame = imutils.resize(frame, width=1280, height=720)

        edge = cv.Canny(frame, 75, 100)

        if edgeshow == 1:
            cv.imshow('Canny Edge', edge)
            cv.setMouseCallback('Canny Edge', show_color)
        else:
            cv.imshow('Canny Edge', frame)
            cv.setMouseCallback('Canny Edge', show_color)

        if cv.waitKey(20) == ord('q'):
            break

    cap.stop()
    cv.destroyAllWindows()

if __name__ == '__main__':
    edge_cam()

