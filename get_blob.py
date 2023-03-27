import cv2
from cv2 import VideoCapture
import numpy as np
import time

# Precálculo de parametros-----------------------------------------

redMin1 = np.array([0, 50, 50])
redMax1 = np.array([3, 255, 255])

redMin2 = np.array([170, 50, 50])
redMax2 = np.array([180, 255, 255])

# Hace el setup de los parámetros para el detector de blobs
params = cv2.SimpleBlobDetector_Params()

params.minThreshold = 10
params.maxThreshold = 200

# Filtro de aceptación del área de los blobs
params.filterByArea = True
params.minArea = 500
params.maxArea = 200000

# Filtro de aceptación de la circularidad (forma) de los blobs
params.filterByCircularity = True
params.minCircularity = 0.35

# Desactivamos los filtros de color, convexidad e inercia
params.filterByColor = False
# params.blobColor = 0
params.filterByConvexity = False
params.filterByInertia = False

# Create a detector with the parameters
ver = (cv2.__version__).split('.')
if int(ver[0]) < 3:
    detector = cv2.SimpleBlobDetector(params)
else:
    detector = cv2.SimpleBlobDetector_create(params)

# Para detectar solo color
params.filterByCircularity = False
params.minArea = 1000
params.maxArea = 99999999

if int(ver[0]) < 3:
    detector2 = cv2.SimpleBlobDetector(params)
else:
    detector2 = cv2.SimpleBlobDetector_create(params)

cam = VideoCapture(0)
# cam = VideoCapture('http://192.168.7.172:8080/video')
cam.set(cv2.CAP_PROP_BUFFERSIZE, 1)


def get_img():
    _, img = cam.read()
    return img


def get_red(show):

    # Toma una foto
    img = get_img()

    img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    # Definimos la mascara final como la suma de las dos anteriores aplicadas a la imagen
    mask_red1 = cv2.inRange(img_hsv, redMin1, redMax1)
    mask_red2 = cv2.inRange(img_hsv, redMin2, redMax2)
    mask_red = cv2.bitwise_or(mask_red1, mask_red2)

    keypoints_red = detector2.detect(255-mask_red)

    if show:
        red = cv2.bitwise_and(img_hsv, img_hsv, mask=mask_red)

        # Dibujamos en la imagen los keypoints detectados
        im_with_keypoints = cv2.drawKeypoints(red, keypoints_red, np.array([]),
                                              (255, 255, 255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

        img_blob = cv2.cvtColor(im_with_keypoints, cv2.COLOR_HSV2BGR)
        cv2.imshow("Keypoints on RED", img_blob)
        cv2.waitKey(1)

    if len(keypoints_red) > 0:
        return True
    else:
        return False


def get_blob(show):

    # Toma una foto
    img = get_img()

    # Debug para mostrar fotos sacadas
    # cv2.imshow("Keypoints on RED", img)
    # cv2.waitKey(1)

    # Convierte la imagen a HSV
    img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    # Definimos la mascara final como la suma de las dos anteriores aplicadas a la imagen
    mask_red1 = cv2.inRange(img_hsv, redMin1, redMax1)
    mask_red2 = cv2.inRange(img_hsv, redMin2, redMax2)
    mask_red = cv2.bitwise_or(mask_red1, mask_red2)

    # Detectamos los blobs en la imagen
    keypoints_red = detector.detect(255-mask_red)

    # ELiminamos todos los pixeles de la imagen original que no contengan
    # ningun tono de rojo en la mascara (Solo para debug)
    if show:
        red = cv2.bitwise_and(img_hsv, img_hsv, mask=mask_red)

        # Dibujamos en la imagen los keypoints detectados
        im_with_keypoints = cv2.drawKeypoints(red, keypoints_red, np.array([]),
                                              (255, 255, 255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

        img_blob = cv2.cvtColor(im_with_keypoints, cv2.COLOR_HSV2BGR)
        cv2.imshow("Keypoints on RED", img_blob)
        cv2.waitKey(1)

    # Devuelvo y printeo la coordenada x, y el diametro del blob más grande de los que
    # ha detectado, en caso de no detectar nada se devuelve -1
    if len(keypoints_red) > 0:
        return [keypoints_red[0].pt[0],  keypoints_red[0].size]
    else:
        return -1


# tiempos = 0
# veces = 100
# for i in range(0, veces):
#     start = time.time()
#     get_blob(True)
#     end = time.time()
#     tiempos += end-start

# media = tiempos / veces

# print("MEDIA DE TIEMPOS: {}".format(media))

while True:
    blob = get_red(True)
