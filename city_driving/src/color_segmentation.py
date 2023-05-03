import cv2
import numpy as np
import pdb

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
        kernele = np.ones((5, 5), np.uint8)
	kerneld = np.ones((5, 5), np.uint8)
	for i in range(2):
		mask = cv2.erode(mask, kernele, cv2.BORDER_CONSTANT) 
		mask = cv2.dilate(mask, kerneld)
	contours, hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)[-2:]
	img_copy = img.copy()
	best_contour = max(contours, key = cv2.contourArea)
	x, y, w, h = cv2.boundingRect(best_contour)
	cv2.rectangle(img, (x, y), (x+w,y+h), (0,255,0),3)
#	image_print(mask)
#	image_print(img_copy)
        
	bounding_box = ((x,y),(x+w,y+h))

	########### YOUR CODE ENDS HERE ###########

	# Return bounding box
	return bounding_box
