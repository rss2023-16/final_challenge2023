import cv2
import numpy as np
import pdb
from skimage.io import imread, imshow
import matplotlib.pyplot as plt

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

def calc_intersection_with_bottom(rho, theta, bottom_y, epsilon):
    if theta<np.pi/2+epsilon and np.pi/2-epsilon<theta:
        return np.inf
    a = np.cos(theta)
    b = np.sin(theta)
    x0 = a*rho
    y0 = b*rho
    slope = -a/b 
    intercept = y0+a*x0/b 
    return (bottom_y-intercept)/slope

def find_intersection(rho1, theta1, rho2, theta2):
    a1 = np.cos(theta1)
    b1 = np.sin(theta1)
    x1 = a1*rho1
    y1 = b1*rho1
    slope1 = -a1/b1
    intercept1 = y1+a1*x1/b1
    a2 = np.cos(theta2)
    b2 = np.sin(theta2)
    x2 = a2*rho2
    y2 = b2*rho2
    slope2 = -a2/b2
    intercept2 = y2+a2*x2/b2
    x = int((intercept2-intercept1)/(slope1-slope2))
    y = int(slope1*x+intercept1)
    
    return (x, y)

def show_lanes(rho1, theta1, rho2, theta2, img):
    a1 = np.cos(theta1)
    b1 = np.sin(theta1)
    x1 = a1*rho1
    y1 = b1*rho1
    a2 = np.cos(theta2)
    b2 = np.sin(theta2)
    x2 = a2*rho2
    y2 = b2*rho2
    
    x11 = int(x1 + 1000*(-b1))
    y11 = int(y1 + 1000*(a1))
    x21 = int(x1 - 1000*(-b1))
    y21 = int(y1 - 1000*(a1))
    img = cv2.line(img, (x11, y11), (x21, y21), (0, 0, 255), 2)

    x12 = int(x2 + 1000*(-b2))
    y12 = int(y2 + 1000*(a2))
    x22 = int(x2 - 1000*(-b2))
    y22 = int(y2 - 1000*(a2))
    img = cv2.line(img, (x12, y12), (x22, y22), (0, 0, 255), 2)

    return img

def cd_color_segmentation(img, bisect_y=0.7):
    best_left_dist = np.inf
    best_right_dist = np.inf
    right_rho, right_theta, left_rho, left_theta = 0, 0, 0, 0
    epsilon = np.pi/18
    y = int(len(img)*bisect_y)
    bottom_y = len(img)
    
    for v in range(len(img)//2):
        for u in range(len(img[0])):
            img[v, u] = 0

    plt.imshow(img)
    plt.show()


    # filter for white
    # TODO change back to BGR for car!!!!
    hsv = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)
    mask = cv2.inRange(hsv, (0, 0, 200), (360, 50, 255)) # white line
    e = 5
    d = 5
    kernele = np.ones((e, e), np.uint8)
    kerneld = np.ones((d, d), np.uint8)

    for i in range(2):
        mask = cv2.erode(mask, kernele) 
        mask = cv2.dilate(mask, kerneld)

    # plt.imshow(mask)
    # plt.show()

   #blur = cv2.GaussianBlur(mask, (3, 3), 1)
    # plt.imshow(blur)
    # plt.show()

    threshold = 210 
    cv_skel = cv2.Canny(mask, 0, threshold)
    plt.imshow(cv_skel)
    plt.show()

    lines = cv2.HoughLines(cv_skel, 1, np.pi/180, threshold = 50)

    print("lines: ", lines)
    if lines is None:
        print("!!!! found no lines")
        return None
    if len(lines)==1:
        x = int(calc_intersection_with_bottom(lines[0][0][0], lines[0][0][1], y, epsilon))
        return x, y, lines[0][0][0], lines[0][0][1], lines[0][0][0], lines[0][0][1]
    
    rhos = np.array([tup[0][0] for tup in lines])
    thetas = np.array([tup[0][1] for tup in lines])
    x_intersect = []

    for i in range(len(lines)):
        rho = rhos[i]
        theta = thetas[i]
        intersect = calc_intersection_with_bottom(rho, theta, bottom_y, epsilon)
        x_intersect.append(intersect)
        dist_center = len(img[0])//2-intersect
        dist = abs(dist_center)
        if dist_center > 0: # right
            if dist < best_right_dist:
                right_rho, right_theta = rho, theta
                best_right_dist = dist
        else:
            if dist < best_left_dist:
                left_rho, left_theta = rho, theta
                best_left_dist = dist


    # didn't find any good lines
    if best_right_dist == np.inf or best_left_dist == np.inf:
        return None
    # if best_right_dist == np.inf:
    #     # just use left 
    #     x = int(calc_intersection_with_bottom(left_rho, left_theta, y, epsilon))
    #     return x, y, left_rho, left_theta, left_rho, left_theta
    # if best_left_dist == np.inf:
    #     # just use right 
    #     x = int(calc_intersection_with_bottom(right_rho, right_theta, y, epsilon))
    #     return x, y, right_rho, right_theta, right_rho, right_theta
    else:
        x_top, y_top = find_intersection(left_rho, left_theta, right_rho, right_theta)
        x_right = int(calc_intersection_with_bottom(right_rho, right_theta, bottom_y, epsilon))
        x_left = int(calc_intersection_with_bottom(left_rho, left_theta, bottom_y, epsilon))
        x_bottom = (x_left+x_right)//2
        y_bottom = len(img)

        m = (y_top-y_bottom)/(x_top-x_bottom)*1.0
        b = y_top -m*x_top

        # we want to find the x along the y=mx+b line that corresponds with y (y=mx+b --> x = 1/m(y-b))
        #newX = 1/m*(y-b)
        #newY = y
        print("intersection: ", x_top, y_top)
        print("x left, x right, x bottom", x_left, x_right, x_bottom)
        print("m, b", m, b)
        try:
            newX = 1/m*(y-b)
            newY = y
            print("newX, newY: ", newX, newY)
        
            return newX, newY, left_rho, left_theta, right_rho, right_theta
        except:
            return None


fname = "/home/fiona/racecar_docker/home/racecar_ws/src/corner_img/track.png"
im = imread(fname)
u, v, rho1, theta1, rho2, theta2 = cd_color_segmentation(im)

img = show_lanes(rho1, theta1, rho2, theta2, im)
box_radius = 10
img = cv2.rectangle(img, (int(u)-box_radius, int(v)-box_radius), (int(u)+box_radius, int(v)+box_radius), (0, 255, 0), 2)
plt.imshow(img)
plt.show()

