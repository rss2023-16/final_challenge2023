import cv2
import numpy as np
import pdb
import rospy

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
    # x1 = int(x0 + 1000*(-b))
    # y1 = int(y0 + 1000*(a))
    # x2 = int(x0 - 1000*(-b))
    # y2 = int(y0 - 1000*(a))
    # cv2.line(img, (x1, y1), (x2, y2), (0, 0, 255), 2)
    # cv2.imshow("image", img)
    # cv2.waitKey(0)
    # cv2.destroyAllWindows()
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
    
    # x11 = int(x1 + 1000*(-b1))
    # y11 = int(y1 + 1000*(a1))
    # x21 = int(x1 - 1000*(-b1))
    # y21 = int(y1 - 1000*(a1))
    # cv2.line(img, (x11, y11), (x21, y21), (0, 0, 255), 2)

    # x12 = int(x2 + 1000*(-b2))
    # y12 = int(y2 + 1000*(a2))
    # x22 = int(x2 - 1000*(-b2))
    # y22 = int(y2 - 1000*(a2))
    # cv2.line(img, (x12, y12), (x22, y22), (0, 0, 255), 2)

    # cv2.line(img, (x+10, y+10), (x-10, y-10), (0, 255, 0), 5)
    # cv2.imshow("image", img)
    # cv2.waitKey(0)
    # cv2.destroyAllWindows()
    return (x, y)

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
    rospy.loginfo("Starting img processing inside get_lanes")
    blur = cv2.GaussianBlur(img, (3, 3), 1)
    threshold = 210 
    # binary = cv2.threshold(gray, threshold, 255, cv2.THRESH_BINARY)[1]
    cv_skel = cv2.Canny(blur, 0, threshold)
    # image_print(cv_skel)
    lines = cv2.HoughLines(cv_skel, 1, np.pi/180, threshold = 150)
    if lines is None:
        # image_print(img)
        # image_print(cv_skel)
        return None
    if len(lines)==1:
        y = len(img)//3
        epsilon = np.pi/18
        x = int(calc_intersection_with_bottom(lines[0][0][0], lines[0][0][1], y, epsilon))
        # cv2.line(img, (x+10, y+10), (x-10, y-10), (0, 255, 0), 5)
        # cv2.imshow("image", img)
        # cv2.waitKey(0)
        # cv2.destroyAllWindows()
        return (x, y)
    rhos = np.array([tup[0][0] for tup in lines])
    thetas = np.array([tup[0][1] for tup in lines])
    if len(lines)==2:
        x, y = find_intersection(rhos[0], thetas[0], rhos[1], thetas[1])
        return (x, y)
    x_intersect = []
    epsilon=np.pi/18
    bottom_y = len(img)
    for i in range(len(lines)):
        rho = rhos[i]
        theta = thetas[i]
        x_intersect.append(calc_intersection_with_bottom(rho, theta, bottom_y, epsilon))
    x_intersect = np.array(x_intersect)
    dist_center = np.abs(len(img[0])-x_intersect)
    idx = np.argpartition(dist_center, 2)[:2]
    rho1 = rhos[idx[0]]
    theta1 = thetas[idx[0]]
    rho2 = rhos[idx[1]]
    theta2 = thetas[idx[1]]
    x, y = find_intersection(rho1, theta1, rho2, theta2)
    rospy.loginfo("finishing img processing inside get_lanes")
    return (x, y)




# img = cv2.imread('/Users/katherinelin/Documents/6.141/racecar_docker/home/racecar_ws/src/final_challenge2023/media/track37.png')
# cd_color_segmentation(img)
