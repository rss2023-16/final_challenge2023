#!/usr/bin/env python

import rospy
import numpy as np

import cv2
from cv_bridge import CvBridge, CvBridgeError

from std_msgs.msg import String
from sensor_msgs.msg import Image
from ackermann_msgs.msg import AckermannDriveStamped
from visualization_msgs.msg import Marker
from final_challenge2023.msg import ConeLocation, ConeLocationPixel
from geometry_msgs.msg import Point

#The following collection of pixel locations and corresponding relative
#ground plane locations are used to compute our homography matrix

# PTS_IMAGE_PLANE units are in pixels
# see README.md for coordinate frame description

######################################################
## DUMMY POINTS -- ENTER YOUR MEASUREMENTS HERE
# PTS_IMAGE_PLANE = [[191, 272], #left side bottom left
#                    [335, 163], #left side top right
#                    [419, 161], #right side top left
#                    [575, 272]] #right side bottom right
# PTS_IMAGE_PLANE = [[183, 331], #left side bottom left
#                    [599, 335], #left side top right
#                    [313, 227], #right side top left
#                    [500, 231]] #right side bottom right
PTS_IMAGE_PLANE = [[174, 314], #left side bottom left
                   [529, 311], #left side top right
                   [276, 232], #right side top left
                   [434, 229],
                   [222, 355],
                   [271, 269],
                   [503, 339],
                   [446, 272],
                   [634, 223],
                   [87, 238]] #right side bottom right

######################################################

# PTS_GROUND_PLANE units are in inches
# car looks along positive x axis with positive y axis to left

######################################################
## DUMMY POINTS -- ENTER YOUR MEASUREMENTS HERE
# PTS_GROUND_PLANE = [[19+7/8.0, 12, ], # left side bottom left
#                     [28+3/8.0, 3+11/16.0], #left side top right
#                     [29+5/16.0, 3+1/4.0], #right side top left
#                     [19+7/8.0, 10+9/16.0]] #right side bottom right
# PTS_GROUND_PLANE = [[12+6/16.0, 9+6/16.0], # left side bottom left
#                     [12+5/16.0, -6-14/16.0], #left side top right
#                     [27+11/16.0, 6], #right side top left
#                     [26+15/16.0, -9+4/16.0]] #right side bottom right
PTS_GROUND_PLANE = [[14+7/16.0, 7+7/16.0], # left side bottom left
                    [14+12/16.0, -8-9/16.0], #left side top right
                    [30+10/16.0, 5+7/16.0], #right side top left
                    [30+15/16.0, -9+1/16.0],
                    [11.0, 4+4/16.0],
                    [19+15/16.0, 3 + 15/16.0],
                    [12+14/16.0, -6-7/16.0],
                    [19+13/16.0, -6-8/16.0],
                    [33+4/16.0,-28-4/16.0],
                    [27+5/16.0,21 + 3/16.0]] #right side bottom right
######################################################

METERS_PER_INCH = 0.0254


class HomographyTransformer:
    def __init__(self):
        self.cone_px_sub = rospy.Subscriber("/relative_cone_px", ConeLocationPixel, self.cone_detection_callback)
        self.cone_pub = rospy.Publisher("/relative_cone", ConeLocation, queue_size=10)

        self.stop_sign_px_sub = rospy.Subscriber("/stop_sign_px", ConeLocationPixel, self.stop_sign_detection_callback)
        self.stop_sign_pub = rospy.Publisher("/stop_sign", ConeLocation, queue_size=10)

        self.click_sub = rospy.Subscriber("/zed/zed_node/rgb/image_rect_color_mouse_left", Point, self.cone_detection_callback)
        self.marker_pub = rospy.Publisher("/cone_marker",
            Marker, queue_size=1)
        self.point_pub = rospy.Publisher("/zed/rgb/image_rect_color", Point, queue_size=1)

        if not len(PTS_GROUND_PLANE) == len(PTS_IMAGE_PLANE):
            rospy.logerr("ERROR: PTS_GROUND_PLANE and PTS_IMAGE_PLANE should be of same length")

        #Initialize data into a homography matrix

        np_pts_ground = np.array(PTS_GROUND_PLANE)
        np_pts_ground = np_pts_ground * METERS_PER_INCH
        np_pts_ground = np.float32(np_pts_ground[:, np.newaxis, :])

        np_pts_image = np.array(PTS_IMAGE_PLANE)
        np_pts_image = np_pts_image * 1.0
        np_pts_image = np.float32(np_pts_image[:, np.newaxis, :])

        self.h, err = cv2.findHomography(np_pts_image, np_pts_ground)

    def convert_and_publish(self, msg, point_pub, msg_pub):
        u = msg.u
        v = msg.v
        
        point = Point()
        point.x = u
        point.y = v
        point_pub.publish(point)

        #Call to main function
        x, y = self.transformUvToXy(u, v)
        if u == 0 and v == 0 or u == -1 and v == -1:
            x, y = 0, 0
#        print(x,y)

        #Publish relative xy position of object in real world
        relative_xy_msg = ConeLocation()
        relative_xy_msg.x_pos = x
        relative_xy_msg.y_pos = y
        
        msg_pub.publish(relative_xy_msg)

    # Transform pixel to real via homography and publish to cone publishers
    def cone_detection_callback(self, msg):
        self.convert_and_publish(msg, self.point_pub, self.cone_pub)

    # Transform pixl to real via homography and publish to stop sign publishers
    def stop_sign_detection_callback(self, msg):
        self.convert_and_publish(msg, self.point_pub, self.stop_sign_pub)


    def transformUvToXy(self, u, v):
        """
        u and v are pixel coordinates.
        The top left pixel is the origin, u axis increases to right, and v axis
        increases down.

        Returns a normal non-np 1x2 matrix of xy displacement vector from the
        camera to the point on the ground plane.
        Camera points along positive x axis and y axis increases to the left of
        the camera.

        Units are in meters.
        """
#	print(self.h)
        homogeneous_point = np.array([[u], [v], [1]])
        xy = np.dot(self.h, homogeneous_point)
        scaling_factor = 1.0 / xy[2, 0]
        homogeneous_xy = xy * scaling_factor
        x = homogeneous_xy[0, 0]
        y = homogeneous_xy[1, 0]
        return x, y

    def draw_marker(self, cone_x, cone_y, message_frame):
        """
        Publish a marker to represent the cone in rviz.
        (Call this function if you want)
        """
        marker = Marker()
        marker.header.frame_id = message_frame
        marker.type = marker.CYLINDER
        marker.action = marker.ADD
        marker.scale.x = .2
        marker.scale.y = .2
        marker.scale.z = .2
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = .5
        marker.pose.orientation.w = 1.0
        marker.pose.position.x = cone_x
        marker.pose.position.y = cone_y
        self.marker_pub.publish(marker)


if __name__ == "__main__":
    rospy.init_node('homography_transformer')
    homography_transformer = HomographyTransformer()
    rospy.spin()
