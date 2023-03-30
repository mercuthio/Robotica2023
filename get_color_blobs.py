# -*- coding: utf-8 -*-
#!/usr/bin/python

# Standard imports
import cv2
import numpy as np

# Read image
img_BGR = cv2.imread("bola.jpg")
# img_BGR = cv2.imread("many.jpg")

# Setup default values for SimpleBlobDetector parameters.
params = cv2.SimpleBlobDetector_Params()

# These are just examples, tune your own if needed
# Change thresholds
params.minThreshold = 10
params.maxThreshold = 200

# Filter by Area
params.filterByArea = True
params.minArea = 200
params.maxArea = 10000000000

# Filter by Circularity
params.filterByCircularity = True
params.minCircularity = 0.1

# Filter by Color
params.filterByColor = False
# not directly color, but intensity on the channel input
params.blobColor = 255
params.filterByConvexity = False
params.filterByInertia = False


# Create a detector with the parameters
ver = (cv2.__version__).split('.')
if int(ver[0]) < 3:
    detector = cv2.SimpleBlobDetector(params)
else:
    detector = cv2.SimpleBlobDetector_create(params)

#  keypoints on original image (will look for blobs in grayscale)
keypoints = detector.detect(img_BGR)
# Draw detected blobs as red circles.
# cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS ensures
# the size of the circle corresponds to the size of blob
im_with_keypoints = cv2.drawKeypoints(img_BGR, keypoints, np.array(
    []), (0, 0, 255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

# Show blobs
cv2.imshow("Keypoints on Gray Scale", im_with_keypoints)
cv2.waitKey(0)

#  filter certain COLOR channels

# Pixels with 100 <= R <= 255, 15 <= B <= 56, 17 <= G <= 50 will be considered red.
#  similar for BLUE

# BY DEFAULT, opencv IMAGES have BGR format

redMin1 = np.array([0, 100, 100])
redMax1 = np.array([3, 255, 255])

redMin2 = np.array([170, 100, 100])
redMax2 = np.array([180, 255, 255])

img_hsv = cv2.cvtColor(img_BGR, cv2.COLOR_BGR2HSV)

# Definimos la mascara final como la suma de las dos anteriores aplicadas a la imagen
mask_red1 = cv2.inRange(img_hsv, redMin1, redMax1)
mask_red2 = cv2.inRange(img_hsv, redMin2, redMax2)
mask_red = cv2.bitwise_or(mask_red1, mask_red2)

keypoints_red = detector.detect(mask_red)

# documentation of SimpleBlobDetector is not clear on what kp.size is exactly, but it looks like the diameter of the blob.
for kp in keypoints_red:
    print(kp.pt[0], kp.pt[1], kp.size)

# Draw detected blobs as red circles.
# cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS ensures
# the size of the circle corresponds to the size of blob
im_with_keypoints = cv2.drawKeypoints(mask_red1, keypoints_red, np.array([]),
                                      (255, 255, 255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

# Show mask and blobs found
cv2.imshow("Keypoints on RED", im_with_keypoints)
cv2.waitKey(0)
