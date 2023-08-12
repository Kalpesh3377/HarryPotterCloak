import cv2
import numpy as np
import time

cap = cv2.VideoCapture(0)  # default camera on
time.sleep(3)
background = 0
for i in range(60):
    ret, background = cap.read()  # fetching background
background = np.flip(background, axis=1)
while True:
    ret, img = cap.read()  # read the video
    img = np.flip(img, axis=1)  # flip the image
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)  # convert the bgr to hsv
    blur = cv2.GaussianBlur(img, (35, 35), 0)  # create a blur image

    lower = np.array([0, 120, 70])
    upper = np.array([10, 255, 255])
    mask1 = cv2.inRange(hsv, lower, upper)

    lower_red = np.array([170, 120, 70])
    upper_red = np.array([180, 255, 255])
    mask2 = cv2.inRange(hsv, lower_red, upper_red)

    mask1 = mask1 + mask2
    mask1 = cv2.morphologyEx(mask1, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8), iterations=2)
    mask1 = cv2.dilate(mask1, np.ones((3, 3), np.uint8), iterations=1)
    mask2 = cv2.bitwise_not(mask1)

    res1 = cv2.bitwise_and(background, background, mask=mask1)
    res2 = cv2.bitwise_and(img, img, mask=mask2)
    final_output = cv2.addWeighted(res1, 1, res2, 1, 0)
    cv2.imshow("Display", final_output)  # show image
    # cv2.imshow("Background",background)#show background
    # cv2.imshow("mask01",mask01)#show mask01
    if cv2.waitKey(1) == 13:
        break
cap.release()
cv2.destroyAllWindows()