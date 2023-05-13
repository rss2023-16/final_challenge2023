from skimage.io import imread, imshow
import cv2
import os, glob
import matplotlib.pyplot as plt
import numpy as np


fname = "/home/fiona/racecar_docker/home/racecar_ws/src/corner_img/huge_box.png"
img = imread(fname)

def segment(img, thresholds, area=750, template=None):
    hsv = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)
    hue_min, hue_max, sat, val = thresholds
    mask = cv2.inRange(hsv, (hue_min, sat, val), (hue_max, 255, 255)) # orange
    #mask = cv2.inRange(hsv, (0, 0, 200), (360, 50, 255)) # white line
    e = 1
    d = 5
    #kernele = np.ones((e, e), np.uint8)
    kernele = np.ones((e, e), np.uint8)
    kerneld = np.ones((d, d), np.uint8)
    dh = 3
    kerneld_horizontal = np.array([[0, 0, 0, 0, 0], 
                                   [0, 0, 0, 0, 0],
                                    [dh, dh, dh, dh, dh],
                                     [0, 0, 0, 0, 0],
                                      [0, 0, 0, 0, 0]])
    kerneld_horizontal = kerneld_horizontal.astype(np.uint8)


    for i in range(1):
        mask = cv2.erode(mask, kernele) 
        

    for i in range(3):
        mask = cv2.dilate(mask, kerneld)
        #mask = cv2.dilate(mask, kerneld_horizontal)

    plt.imshow(mask)
    plt.show()

    contours, hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)[-2:]
    img_copy = img.copy()
    # print("found: ", len(contours))
    if len(contours) == 0:
        return None
    best_contour = max(contours, key = cv2.contourArea)
    print("area: ", cv2.contourArea(best_contour))
    if cv2.contourArea(best_contour) < area:
        return None
    x, y, w, h = cv2.boundingRect(best_contour)

    if w > 400:
        cropped_img = mask[y:int(y + h/3), :]
        indices = np.argwhere(cropped_img==255) 

        if len(indices) == 0:
            print("No indices??")
            return (0, 0)
        # rows correspond to y/v, cols corespond to x/u
        v, u = indices.sum(0)/len(indices)
        v = int(v)
        u = int(u)

        print("center x y: ", u, v+y)
        img_width = 672
        if u > int(672/2):
            u = 600
        else:
            u = 0

        x = u
        y = v + y
        w = 2
        h = 2

    rect = cv2.rectangle(img, (x, y), (x+w,y+h), (0,255,0),3)
    plt.imshow(rect)
    plt.show()
#	image_print(mask)
#	image_print(img_copy)
        
    bounding_box = ((x,y),(x+w,y+h))
    return bounding_box



# returns a mask to find the orange pixels
def orange_mask_segmentation(img, thresholds):
    img_hsv = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)
    hue_min, hue_max, sat, val = thresholds
    mask = cv2.inRange(img_hsv, (hue_min, sat, val), (hue_max, 255, 255))

    plt.imshow(img_hsv)
    plt.show()
    return mask


def line_centroid(img, thresholds):
    ((x1, y1), (x2, y2)) = segment(img, thresholds)

    # only look between y1 and (y1+y2)//2
    cropped_img = img[y1:int(y1*2/3+y2*1/3), :]
    plt.imshow(cropped_img)
    plt.show()
    mask = orange_mask_segmentation(cropped_img, thresholds)

    plt.imshow(mask)
    plt.show()

    e = 1
    d = 5
    #kernele = np.ones((e, e), np.uint8)
    kernele = np.ones((e, e), np.uint8)
    kerneld = np.ones((d, d), np.uint8)

    for i in range(1):
        mask = cv2.erode(mask, kernele) 
        
    # plt.imshow(mask)
    # plt.show()

    # for i in range(3):
    #     mask = cv2.dilate(mask, kerneld)

    plt.imshow(mask)
    plt.show()

    print(mask)
        
    indices = np.argwhere(mask==255) 

    if len(indices) == 0:
        return (0, 0)
    # rows correspond to y/v, cols corespond to x/u
    v, u = indices.sum(0)/len(indices)
    return(u, v)

res = segment(img, [10, 35, 150, 100])
print(res)
#	image_print(mask)
#	image_print(img_copy)
        
#     bounding_box = ((x,y),(x+w,y+h))

#     ########### YOUR CODE ENDS HERE ###########




#     hue_mask = ((img_hsv[:, :, 0] >= 10) & (img_hsv[:, :, 0] <= 19))  # Color thresholding
#     saturation_mask = (img_hsv[:, :, 1] >= 100)  # Saturation thresholding
#     value_mask = (img_hsv[:, :, 2] >= 150)  # Value thresholding

#     plt.imshow(img_hsv)
#     plt.show()

#     plt.imshow(hue_mask)
#     plt.show()

#     plt.imshow(saturation_mask)
#     plt.show()

#     plt.imshow(value_mask)
#     plt.show()

#     strap_mask = hue_mask * saturation_mask * value_mask

#     indices = np.argwhere(strap_mask==True)
#     print("inds: " ,indices)
#     centroid_v, centroid_u = indices.sum(0)/len(indices)
#     print(fname + " has center u, v: ", centroid_u, centroid_v)

#     # check if turning left or turning right
#     # get where line is
#     # look at 10 closest rows to front, find average u
#     closest_10_indices = np.argwhere(strap_mask[-10:, :] == True)
#     _, line_u = closest_10_indices.sum(0)/len(closest_10_indices)
#     print("line: ", line_u)
#     print("px between line and centroid:", line_u - centroid_u)
#     if centroid_u < line_u:
#         print("turn left")
#     else:
#         print("turn right")

#     plt.imshow(strap_mask)
#     plt.show()

#     plt.close('all')


#     # try the old bb thing
# #     hsv = img_hsv#cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
# #     mask = cv2.inRange(hsv, (10, 150, 100), (19, 255, 255)) # orange
# #     #mask = cv2.inRange(hsv, (0, 0, 200), (360, 50, 255)) # white line
# #     kernele = np.ones((5, 5), np.uint8)
# #     kerneld = np.ones((5, 5), np.uint8)
# #     for i in range(2):
# #         mask = cv2.erode(mask, kernele, cv2.BORDER_CONSTANT) 
# #         mask = cv2.dilate(mask, kerneld)
# #     contours, hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)[-2:]
# #     img_copy = img.copy()
# #     best_contour = max(contours, key = cv2.contourArea)
# #     x, y, w, h = cv2.boundingRect(best_contour)
# #     boxed = cv2.rectangle(img, (x, y), (x+w,y+h), (0,255,0),3)
# #     cv2.imshow("boxed", boxed)
# #     cv2.waitKey(5000)
# # #	image_print(mask)
# # #	image_print(img_copy)
        
# #     ((x1, y1), (x2, y2)) = ((x,y),(x+w,y+h))
# #     u = (x1+x2)//2
# #     v = y2
# #     print("bb found pixel: ", u, v)




