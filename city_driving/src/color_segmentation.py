import cv2
import numpy as np
import pdb
import rospy
# import matplotlib.pyplot as plt
# from skimage.io import imread, imshow 

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
    ########## YOUR CODE STARTS HERE ##########
#	image_print(img)

    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, (0, 150, 100), (35, 255, 255)) # orange
    #mask = cv2.inRange(hsv, (0, 0, 200), (360, 50, 255)) # white line
    e = 1
    d = 5
    kernele = np.ones((e, e), np.uint8)
    kerneld = np.ones((d, d), np.uint8)
    for i in range(2):
        mask = cv2.erode(mask, kernele) 
        mask = cv2.dilate(mask, kerneld)
    contours, hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    img_copy = img.copy()
    print("found: ", len(contours))
    if len(contours) == 0:
        return None
    best_contour = max(contours, key = cv2.contourArea)
    print("area: ", cv2.contourArea(best_contour))
    if cv2.contourArea(best_contour) < 750:
        return None
    x, y, w, h = cv2.boundingRect(best_contour)
    cv2.rectangle(img, (x, y), (x+w,y+h), (0,255,0),3)
#	image_print(mask)
#	image_print(img_copy)
        
    bounding_box = ((x,y),(x+w,y+h))

    ########### YOUR CODE ENDS HERE ###########

    # Return bounding box
    return bounding_box

# returns a mask to find the orange pixels
def orange_mask_segmentation(img, thresholds):
    img_hsv = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)
    print("thresholds: ", thresholds)   
    hue_mask = (img_hsv[:, :, 0] >= thresholds[0])  # Color thresholding
    saturation_mask = (img_hsv[:, :, 1] >= thresholds[2])  # Saturation thresholding
    value_mask = (img_hsv[:, :, 2] >= thresholds[3])  # Value thresholding
    full_mask = hue_mask * saturation_mask * value_mask
    # if visualize:
    #     plt.imshow(img_hsv)
    # 	plt.show()

    # 	plt.imshow(hue_mask)
    # 	plt.show()

    # 	plt.imshow(saturation_mask)
    # 	plt.show()

    # 	plt.imshow(value_mask)
    # 	plt.show()

    # 	
    # 	plt.imshow(full_mask)
    # 	plt.show()

    # 	plt.close('all')

    return full_mask

def line_centroid(img, thresholds):
    orange_mask = orange_mask_segmentation(img, thresholds)
    indices = np.argwhere(orange_mask==True) 
    
    if len(indices) == 0:
        return (0, 0)
    # rows correspond to y/v, cols corespond to x/u
    v, u = indices.sum(0)/len(indices)
    return(u, v)


def line_farthest_corner(img):
    orange_mask = orange_mask_segmentation(img)
    indices = np.argwhere(orange_mask==True)
    # rows correspond to y/v, cols corespond to x/u
    centroid_v, centroid_u = indices.sum(0)/len(indices)
    closest_10_indices = np.argwhere(orange_mask[-10:, :] == True)
    _, line_u = closest_10_indices.sum(0)/len(closest_10_indices)
    print("line: ", line_u)
    print("px between line and centroid:", line_u - centroid_u)
    if abs(centroid_u - line_u) > 50:
        if centroid_u < line_u:
            print("turn left")
            return 0, 300
        else:
            print("turn right")
            return 660, 300
    else:
        return centroid_u, centroid_v
