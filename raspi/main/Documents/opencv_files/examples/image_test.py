import cv2

image = cv2.VideoCapture(0)

image.set (3,600)
image.set (4,320)

ret,frame = image.read()
cv2.imwrite('test_image.jpg', frame)
image.release()
