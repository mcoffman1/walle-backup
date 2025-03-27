import cv2 as cv


img = cv2.imread('NCAT_North.png')

ret, bw_img = cv2.threshold(img, 240, 255, cv2.THRESH_BINARY)

bw = cv2.threshold(img, 240, 255, cv2.THRESH_BINARY)

cv2.imshow("Binary", bw_img)
cv2.imwrite("NCAT_North_binary.png", bw_img)
cv2.waitkey(0)


cv2.destoryAllWindows()





