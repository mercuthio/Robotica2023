import cv2
import numpy as np

# Para mostrar imagen con matches
DEBUG = 1

# max number of features to extract per image
MAX_FEATURES = 500
# REQUIRED number of correspondences (matches) found:
MIN_MATCH_COUNT = 20          # initially
MIN_MATCH_OBJECTFOUND = 15    # after robust check, to consider object-found


def drawMatches2(img1, kp1, img2, kp2, matches, color=None, thickness=2, mask=None):
    """
    Similar to drawMatches in newer versions of open CV
    Draws lines between matching keypoints (kp1, kp2) of the two input images
    color and thickness: line plot properties
    matches: n x Match_objects
    mask: n x bool. List of booleans to indicate which matches should be displayed 
    """
    # We're drawing them side by side.  Get dimensions accordingly.
    # Handle both color and grayscale images.
    if len(img1.shape) == 3:
        new_shape = (max(img1.shape[0], img2.shape[0]),
                     img1.shape[1]+img2.shape[1], img1.shape[2])
    elif len(img1.shape) == 2:
        new_shape = (max(img1.shape[0], img2.shape[0]),
                     img1.shape[1]+img2.shape[1])
    new_img = np.zeros(new_shape, type(img1.flat[0]))
    # Place images onto the new image.
    new_img[0:img1.shape[0], 0:img1.shape[1]] = img1
    new_img[0:img2.shape[0], img1.shape[1]:img1.shape[1]+img2.shape[1]] = img2

    # Draw lines between matches.
    if color:
        c = color
    for i, m in enumerate(matches):
        if mask is None or (mask is not None and mask[i]):
            # Generate random color for RGB/BGR and grayscale images as needed.
            if not color:
                c = np.random.randint(0, 256, 3) if len(
                    img1.shape) == 3 else np.random.randint(0, 256)
            p1 = tuple(np.round(kp1[m.queryIdx].pt).astype(int))
            p2 = tuple(np.round(kp2[m.trainIdx].pt).astype(
                int) + np.array([img1.shape[1], 0]))
            cv2.line(new_img, p1, p2, c, thickness)
    return new_img


def match_images(img1_bgr, img2_bgr):

    # Feature extractor uses grayscale images
    img1 = cv2.cvtColor(img1_bgr, cv2.COLOR_BGR2GRAY)
    img2 = cv2.cvtColor(img2_bgr, cv2.COLOR_BGR2GRAY)

    ver = (cv2.__version__).split('.')
    if int(ver[0]) < 3:  # CURRENT RASPBERRY opencv version is 2.4.9
        # Initiate ORB detector --> you could use any other detector, but this is the best performing one in this version
        binary_features = True

        detector = cv2.ORB()
    else:
        # Initiate BRISK detector --> you could use any other detector, including NON binary features (SIFT, SURF)
        # but this is the best performing one in this version
        binary_features = True
        detector = cv2.BRISK_create()

    # find the keypoints and corresponding descriptors
    kp1, des1 = detector.detectAndCompute(img1, None)
    kp2, des2 = detector.detectAndCompute(img2, None)

    if des1 is None or des2 is None:
        print("WARNING: empty detection?")
        return False
    if len(des1) < MIN_MATCH_COUNT or len(des2) < MIN_MATCH_COUNT:
        print("WARNING: not enough FEATURES (im1: %d, im2: %d)" %
              (len(des1), len(des2)))
        return False
    print(" FEATURES extracted (im1: %d, im2: %d)" % (len(des1), len(des2)))

    if binary_features:
        bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
        matches = bf.match(des1, des2)
        matches = sorted(matches, key=lambda x: x.distance)
        good = matches
    else:
        FLANN_INDEX_KDTREE = 0
        index_params = dict(algorithm=FLANN_INDEX_KDTREE, trees=5)
        search_params = dict(checks=50)
        flann = cv2.FlannBasedMatcher(index_params, search_params)
        matches = flann.knnMatch(des1, des2, k=2)
        # store all the good matches as per Lowe's ratio test.
        good = []
        for m, n in matches:
            if m.distance < 0.7*n.distance:
                good.append(m)

    print(" Initial matches found: %d" % (len(good)))
    if DEBUG > 1:
        ver = (cv2.__version__).split('.')
        if int(ver[0]) < 3:  # CURRENT RASPBERRY opencv version is 2.4.9
            img_tmp = drawMatches2(img1, kp1, img2, kp2, good)
        else:
            img_tmp = cv2.drawMatches(img1, kp1, img2, kp2, good, None)
        cv2.imshow("All matches", img_tmp)
        cv2.waitKey(0)

    if len(good) > MIN_MATCH_COUNT:
        src_pts = np.float32(
            [kp1[m.queryIdx].pt for m in good]).reshape(-1, 1, 2)
        dst_pts = np.float32(
            [kp2[m.trainIdx].pt for m in good]).reshape(-1, 1, 2)
        H_21, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC, 3.0)
        matchesMask = mask.ravel().tolist()
        num_robust_matches = np.sum(matchesMask)
        if num_robust_matches < MIN_MATCH_OBJECTFOUND:
            found = False
            print("NOT enough ROBUST matches found - %d (required %d)" %
                  (num_robust_matches, MIN_MATCH_OBJECTFOUND))
            return found
        h, w = img1.shape
        pts = np.float32([[0, 0], [0, h-1], [w-1, h-1],
                         [w-1, 0]]).reshape(-1, 1, 2)
        dst = cv2.perspectiveTransform(pts, H_21)
        found = True
        print("ROBUST matches found - %d (out of %d) --> OBJECT FOUND" %
              (np.sum(matchesMask), len(good)))
    else:
        print("Not enough initial matches are found - %d (required %d)" %
              (len(good), MIN_MATCH_COUNT))
        matchesMask = None
        found = False

    if DEBUG:
        img3 = drawMatches2(img1_bgr, kp1, img2_bgr, kp2, good, color=(0, 255, 0),
                            mask=matchesMask)
        cv2.imshow("INLIERS", img3)
        cv2.waitKey(0)  # WAIT is run outside

    return found


# img_r2 = cv2.imread("imagenes/R2-D2_s.png")
# img_test1 = cv2.imread("imagenes/test3.jpg")


# match_images(img_r2, img_test1)
