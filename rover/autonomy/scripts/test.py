import cv2
import numpy as np

template = cv2.imread('/scripts/1.png')
image = cv2.imread('/scripts/aruco_phone1_detected.png')

w, h = template.shape[:-1]

templateGray = cv2.cvtColor(template, cv2.COLOR_BGR2GRAY)

ret, mask = cv2.threshold(templateGray, 200, 255, cv2.THRESH_BINARY)

mask_inv = cv2.bitwise_not(mask)
mask_inv = cv2.cvtColor(mask_inv,cv2.COLOR_GRAY2RGB)

print(image.shape, image.dtype)
print(template.shape, template.dtype)
print(mask_inv.shape, mask_inv.dtype)

method = cv2.TM_SQDIFF 

result = cv2.matchTemplate(image, template, method, None, mask=mask_inv)