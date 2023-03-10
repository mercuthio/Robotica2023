import cv2
from cv2 import VideoCapture
import numpy as np

redMin1 = np.array([0,50,50])
redMax1 = np.array([3,255,255])

redMin2 = np.array([170,50,50])
redMax2 = np.array([180,255,255])

# Hace el setup de los parámetros para el detector de blobs
params = cv2.SimpleBlobDetector_Params()

# Umbrales de ejemplo para el detector de blobs
params.minThreshold = 10
params.maxThreshold = 200

# Filtro de aceptación del área de los blobs
params.filterByArea = True
params.minArea = 350
params.maxArea = 20000

# Filtro de aceptación de la circularidad (forma) de los blobs
params.filterByCircularity = True
params.minCircularity = 0.10

# Desactivamos los filtros de color, convexidad e inercia
params.filterByColor = False
#params.blobColor = 0
params.filterByConvexity = False
params.filterByInertia = False

def get_blob():
    # Abre la cámara y toma una foto
    cam = VideoCapture(0)
    out, img = cam.read()

    # cv2.imshow("Keypoints on RED", img)
    # cv2.waitKey(1)

    # Convierte la imagen a HSV
    img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    # Create a detector with the parameters
    ver = (cv2.__version__).split('.')
    if int(ver[0]) < 3 :
        detector = cv2.SimpleBlobDetector(params)
    else :
        detector = cv2.SimpleBlobDetector_create(params)    

    mask_red1 = cv2.inRange(img_hsv, redMin1, redMax1)
    mask_red2 = cv2.inRange(img_hsv, redMin2, redMax2)
    mask_red = mask_red1 + mask_red2

    img_aux = img_hsv.copy()
    img_aux[np.where(mask_red==0)] = 0

    # Detectamos los blobs en la imagen
    keypoints_red = detector.detect(255-mask_red)

    # Printeo la coordenada x, la y y el tamaño del blob más grande de los que
    # ha detectado
    if len(keypoints_red) > 0:
        print(keypoints_red[0].pt[0])
        print(keypoints_red[0].size)
    else:
        print("No se ha encontrado ninguna pelota")

    if len(keypoints_red) == 0:
        return -1

    # Dibujamos en la imagen los keypoints detectados
    im_with_keypoints = cv2.drawKeypoints(img_aux, keypoints_red, np.array([]),
	(255,255,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

    img_blob = cv2.cvtColor(im_with_keypoints, cv2.COLOR_HSV2BGR)

    cv2.imshow("Keypoints on RED", img_blob)
    cv2.waitKey(1)

    # Devuelvo la coordenada x, la y y el tamaño del blob más grande de los que
    # ha detectado
    if len(keypoints_red) > 0:
        return [keypoints_red[0].pt[0],  keypoints_red[0].size]
    
# get_blob()