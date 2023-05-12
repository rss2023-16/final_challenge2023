from skimage.io import imread, imshow
import cv2
import os, glob
import matplotlib.pyplot as plt
import numpy as np

fname = "/Users/lasya/6.141/racecar_docker/home/racecar_ws/src/final_challenge2023/city_driving/corner3.png"
img = imread(fname)

def segment(img):
    hsv = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)
    hue_min, hue_max, sat, val = [0, 35, 120, 100]
    print(hsv.shape)
    plt.imshow(hsv)
    plt.show()
    mask = cv2.inRange(hsv, (hue_min, sat, val), (hue_max, 255, 255)) # orange
    #mask = cv2.inRange(hsv, (0, 0, 200), (360, 50, 255)) # white line
    e = 3
    d = 9
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
    plt.imshow(mask)
    plt.show()

    for i in range(2):
        mask = cv2.erode(mask, kernele) 
        mask = cv2.dilate(mask, kerneld)

    plt.imshow(mask)
    plt.show()

    for i in range(10):
        mask = cv2.dilate(mask, kerneld_horizontal)
    plt.imshow(mask)
    plt.show()
    contours, hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)[-2:]
    img_copy = img.copy()
    # print("found: ", len(contours))
    if len(contours) == 0:
        return None
    best_contour = max(contours, key = cv2.contourArea)
    print("area: ", cv2.contourArea(best_contour))
    if cv2.contourArea(best_contour) < 2000:
        return None
    x, y, w, h = cv2.boundingRect(best_contour)
    rect = cv2.rectangle(img, (x, y), (x+w,y+h), (0,255,0),3)
    plt.imshow(rect)
    plt.show()

res = segment(img)
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




