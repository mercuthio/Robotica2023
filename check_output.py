import cv2
import numpy as np
from cv2 import VideoCapture
from image_match import match_images


def get_img(cam):
    _, img = cam.read()
    return img


def check_output(robot_img_r2, robot_img_bb8, test_img, mapa):
    found_r2, w_pos_r2 = match_images(robot_img_r2, test_img)
    found_bb8, w_pos_bb8 = match_images(robot_img_bb8, test_img)

    if found_r2 and found_bb8:
        # image_width = test_img.shape[1]

        # If
        if mapa == "A":
            return "izquierda" if w_pos_r2 < w_pos_bb8 else "derecha"
        else:
            return "izquierda" if w_pos_bb8 < w_pos_r2 else "derecha"
    else:
        return "No encontrado"

# Se podria hacer el match con los dos robots y en base a cual es el mayor
# saber cual esta a la derecha o a la izquierda. Acertaria siempre pero tendria
# el doble de coste


img_r2 = cv2.imread("imagenes/R2-D2_s.png")
img_bb8 = cv2.imread("imagenes/BB8_s.png")

img_test = get_img()

print(check_output(img_r2, img_bb8, img_test, "A"))
