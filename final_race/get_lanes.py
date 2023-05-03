import cv2
import numpy as np
import pdb
from skimage import morphology, filters, img_as_ubyte

#################### X-Y CONVENTIONS #########################
# 0,0  X  > > > > >
#
#  Y
#
#  v  This is the image. Y increases downwards, X increases rightwards
#  v  Please return bounding boxes as ((xmin, ymin), (xmax, ymax))
#  v
#  v
#  v
###############################################################

def image_print(img):
    """
    Helper function to print out images, for debugging. Pass them in as a list.
    Press any key to continue.
    """
    cv2.imshow("image", img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

def cd_color_segmentation(img, template=None):
    """
    Implement the cone detection using color segmentation algorithm
    Input:
        img: np.3darray; the input image with a cone to be detected. BGR.
        template_file_path; Not required, but can optionally be used to automate setting hue filter values.
    Return:
        bbox: ((x1, y1), (x2, y2)); the bounding box of the cone, unit in px
                (x1, y1) is the top left of the bbox and (x2, y2) is the bottom right of the bbox
    """
#   image_print(img)

    # hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    # mask = cv2.inRange(hsv, (0, 0, 200), (360, 50, 255))
    # image_print(mask)
    
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY) #convert to grayscale
    threshold = 210 # filters.threshold_otsu(gray)
    binary = (gray>threshold)
    skeleton = morphology.skeletonize(binary)
    cv_skel = img_as_ubyte(skeleton)
    lines = cv2.HoughLines(cv_skel, 10, np.pi/18, 150)
    print(np.shape(lines))
    print(lines)
    for rho,theta in lines[0]:
        print(rho, theta)
        a = np.cos(theta)
        b = np.sin(theta)
        x0 = a*rho
        y0 = b*rho
        x1 = int(x0 + 1000*(-b))
        y1 = int(y0 + 1000*(a))
        x2 = int(x0 - 1000*(-b))
        y2 = int(y0 - 1000*(a))
        print(x1, y1, x2, y2)
        cv2.line(cv_skel, (x1, y1), (x2, y2), (0, 0, 255), 2)
        cv2.imshow("image", cv_skel)
        cv2.waitKey(0)
        cv2.destroyAllWindows()
    # vert_pad = len(cv_skel)//3
    # cropped = cv_skel[vert_pad:19*len(cv_skel)//20]
    # return cropped
    # image_print(cropped)

img = cv2.imread('/Users/katherinelin/Documents/6.141/racecar_docker/home/racecar_ws/src/final_challenge2023/media/track33.png')
cd_color_segmentation(img)
