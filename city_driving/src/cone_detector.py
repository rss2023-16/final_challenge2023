#!/usr/bin/env python

import numpy as np
import rospy

import cv2
from cv_bridge import CvBridge, CvBridgeError

from sensor_msgs.msg import Image
from geometry_msgs.msg import Point #geometry_msgs not in CMake file
from final_challenge2023.msg import ConeLocationPixel

# import your color segmentation algorithm; call this function in ros_image_callback!
import color_segmentation


class ConeDetector():
    """
    A class for applying your cone detection algorithms to the real robot.
    Subscribes to: /zed/zed_node/rgb/image_rect_color (Image) : the live RGB image from the onboard ZED camera.
    Publishes to: /relative_cone_px (ConeLocationPixel) : the coordinates of the cone in the image frame (units are pixels).
    """
    def __init__(self):
        # toggle line follower vs cone parker
        self.LineFollower = False

        # Subscribe to ZED camera RGB frames
        self.cone_pub = rospy.Publisher("/relative_cone_px", ConeLocationPixel, queue_size=10)
        self.debug_pub = rospy.Publisher("/cone_debug_img", Image, queue_size=10)
        self.image_sub = rospy.Subscriber("/zed/zed_node/rgb/image_rect_color", Image, self.image_callback)
        self.bridge = CvBridge() # Converts between ROS images and OpenCV Images
        # self.turn_left = False
        # self.turn_right = False
        # self.corner = False
        # self.time = 0

    def image_callback(self, image_msg):
        #rospy.logerr("IMAGE CALLBACK")
        # Apply your imported color segmentation function (cd_color_segmentation) to the image msg here
        # From your bounding box, take the center pixel on the bottom
        # (We know this pixel corresponds to a point on the ground plane)
        # publish this pixel (u, v) to the /relative_cone_px topic; the homography transformer will
        # convert it to the car frame.
        image = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")
        #################################
        # YOUR CODE HERE
        # detect the cone and publish its
        # pixel location in the image.
        use_bbox = True
        u, v = -1, -1
        thresholds = [rospy.get_param("~hue_min"), rospy.get_param("~hue_max"), rospy.get_param("~saturation"), rospy.get_param("~value")]
        area = rospy.get_param("~area")
        if use_bbox:
            bbox = color_segmentation.cd_color_segmentation(image, thresholds, area)

            if bbox is None:
                u, v = 0, 0
                # self.left_turn = False
                # self.right_turn = False
            else:
                ((x1, y1), (x2, y2)) = bbox
         #       print(bbox)
                im_width = 672
                u = (x1+x2)//2 - 50
                v = y1 #(y1+y2)//2
                eps = rospy.get_param("~corner")
                if x1 < eps and x2 > im_width - eps:
                    u, v = 0, 0
                if x1 < eps and x2 < im_width - eps:
                    u = x1
                if x2 > im_width -eps and x1 > eps:
                    u = x2
                image = cv2.rectangle(image, (x1, y1), (x2, y2), (0, 255, 0), 2)
                # not_corner_eps = rospy.get_param("~not_corner")
        
                # if not self.turn_left and not self.turn_right:
                #     if x1 < eps and x2 < 672 - eps:
                #         self.turn_left = True
                #     if x2 > 672 -eps and x1 > eps:
                #         self.turn_right = True
                                
                # elif self.turn_left:
                #     if x1 > not_corner_eps:
                #         self.turn_left = False
                # elif self.turn_right:
                #       if x2 < 650 - not_corner_eps:
                #           self.turn_right = False

                # width = rospy.get_param("~width")

                # print("width: ", x2-x1)

                # if not self.corner:
                #     if (x2-x1) >= width:
                #         self.corner = True
                #         self.time = rospy.get_time()

                #         if x1 < 672 - x2:
                #             print("left turn")
                #             self.turn_left = True
                #         elif 672 - x2 < x1:
                #             print("right turn")
                #             self.turn_right = True
                    
                # if self.corner: 
                #     if (x2-x1) < width:
                #         self.corner = False
                #         self.turn_right = False
                #         self.turn_left = False
                #     else:
                #         if self.turn_left:
                #             print("left turn")
                #             u = 168
                #         elif self.turn_right:
                #             print("right turn")
                #             u = 504
                    
                #         ####### new code for turning based on time #######
                #         seconds = rospy.get_param("~seconds")
                        
                #         if rospy.get_time() - self.time > seconds:
                #             self.corner = False
                #             self.turn_right = False
                #             self.turn_left = False
                #         else:
                #             if self.turn_left:
                #                 u = 168
                #             elif self.turn_right:
                #                 u = 504

        # if self.turn_left:
        #     u = rospy.get_param("~crop")
        #                 #u = max(x1, rospy.get_param("~crop"))
        #     print("turning LEFT")
        # if self.turn_right:
        #     u = 672 - rospy.get_param("~crop")
        #                 #u = min(x2, 672 - rospy.get_param("~crop"))
        #     print("turning RIGHT")
    
                image = cv2.rectangle(image, (x1, y1), (x2, y2), (0, 255, 0), 2)
        else:
            thresholds = [rospy.get_param("~hue_min"), rospy.get_param("~hue_max"), rospy.get_param("~saturation"), rospy.get_param("~value")]
            u, v = color_segmentation.line_centroid(image, thresholds)
            if u != 0:
                u -= 80
            # u, v = color_segmentation.line_farthest_corner(image)
        cone_location = ConeLocationPixel()
        cone_location.u = u
        cone_location.v = v
        #print("U, V", u, v)
        self.cone_pub.publish(cone_location)
        # vvvvvvvvvvvvvvvvvvvvvvvvvvvvvv
        #################################
        debug_msg = self.bridge.cv2_to_imgmsg(image, "bgr8")
        self.debug_pub.publish(debug_msg)


if __name__ == '__main__':
    try:
        rospy.init_node('ConeDetector', anonymous=True)
        ConeDetector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    
