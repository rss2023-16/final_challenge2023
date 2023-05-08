from skimage.io import imread, imshow
import cv2
import os, glob
import matplotlib.pyplot as plt
import numpy as np

folder = "/home/fiona/racecar_docker/home/racecar_ws/src/corner_img"





def get_images_from_folder(folder):
    # print("finding images from ", folder)
    imgs = []
    for filename in glob.glob(os.path.join(folder, '*.png')):
        with open(os.path.join(os.getcwd(), filename), 'r') as f: # open in readonly mode
            imgs.append(f.name)
    # print("found ", len(imgs), " images")

    for filename in glob.glob(os.path.join(folder, '*.jpg')):
        with open(os.path.join(os.getcwd(), filename), 'r') as f: # open in readonly mode
            imgs.append(f.name)
    return imgs

#fnames = get_images_from_folder(folder)
fnames = [folder+"/car_4_ft.png"]
thresholds = [10, 130, 150]



for fname in fnames:
    img = imread(fname)
    # print("image size: ", len(img), len(img[0]))
    img_hsv = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)

    hue_mask = (img_hsv[:, :, 0] >= thresholds[0])  # Color thresholding
    saturation_mask = (img_hsv[:, :, 1] >= thresholds[1])  # Saturation thresholding
    value_mask = (img_hsv[:, :, 2] >= thresholds[2])  # Value thresholding

    plt.imshow(img_hsv)
    plt.show()

    
    plt.show()

    plt.imshow(saturation_mask)
    plt.show()

    plt.imshow(value_mask)
    plt.show()

    strap_mask = hue_mask * saturation_mask * value_mask
    plt.imshow(strap_mask)
    plt.show()
    indices = np.argwhere(strap_mask==True)
    print("inds: " ,indices)
    centroid_v, centroid_u = indices.sum(0)/len(indices)
    print("center u, v: ", centroid_u, centroid_v)

    # check if turning left or turning right
    # get where line is
    # look at 10 closest rows to front, find average u
    closest_10_indices = np.argwhere(strap_mask[-10:, :] == True)
    _, line_u = closest_10_indices.sum(0)/len(closest_10_indices)
    print("line: ", line_u)
    print("px between line and centroid:", line_u - centroid_u)
    if centroid_u < line_u:
        print("turn left")
    else:
        print("turn right")

    plt.imshow(strap_mask)
    plt.show()

    plt.close('all')




