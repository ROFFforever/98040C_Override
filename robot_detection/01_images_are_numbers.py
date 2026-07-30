import cv2
import numpy as np

img = np.zeros((300, 400, 3), dtype=np.uint8)

print("shape:", img.shape)
print("dtype:", img.dtype)
print("total numbers in this image:", img.size)
print("pixel at row 0, col 0:", img[0, 0])

img[150, 200] = (255, 255, 255)

img[20:80, 20:180] = (255, 0, 0)
img[100:160, 20:180] = (0, 255, 0)
img[180:240, 20:180] = (0, 0, 255)

print("pixel inside first bar: ", img[50, 100])
print("pixel inside second bar:", img[130, 100])
print("pixel inside third bar: ", img[210, 100])

cv2.circle(img, (300, 100), 45, (0, 255, 255), thickness=3)
cv2.line(img, (220, 180), (380, 260), (255, 0, 255), thickness=2)
cv2.putText(img, "BGR not RGB", (200, 290), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)

gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
print("gray shape:", gray.shape)

cv2.imshow("color (press any key to close)", img)
cv2.imshow("grayscale", gray)
cv2.waitKey(0)
cv2.destroyAllWindows()
