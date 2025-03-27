import imutils
import cv2

# creat image obj from jpg
image = cv2.imread('test_image.jpg')

# get dim from image
(h,w,d) = image.shape
print("width={}, height{}, depth={}".format(w,h,d))

# show image in window named window name and wait for any key
cv2.imshow("window name",image)
cv2.waitKey(0)

#get a pix print its value
(b,g,r) = image[100,100]
print("r={}, g={}, b={}".format(r,g,b))

# resize with imutils
resized = imutils.resize(image, height=480)
cv2.imshow("lksdjf",resized)
cv2.waitKey(0)
