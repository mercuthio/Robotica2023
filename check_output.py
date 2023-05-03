import cv2
import numpy as np
from cv2 import VideoCapture
from image_match import match_images

cam = VideoCapture(0)
# cam = VideoCapture('http://192.168.7.172:8080/video')
cam.set(cv2.CAP_PROP_BUFFERSIZE, 1)


def get_img():
    _, img = cam.read()
    return img


def check_output(robot_img, test_img):
    found, w_pos = match_images(robot_img, test_img)
    if found == True:
        image_width = test_img.shape[1]
        return "izquierda" if w_pos < (image_width / 2) else "derecha"

# Se podria hacer el match con los dos robots y en base a cual es el mayor
# saber cual esta a la derecha o a la izquierda. Acertaria siempre pero tendria
# el doble de coste


img_r2 = cv2.imread("imagenes/R2-D2_s.png")
# img_test = cv2.imread("imagenes/test5.jpg")
img_test = get_img()
print(check_output(img_r2, img_test))
