#!/usr/bin/env python

import rospy
import numpy as np
import time

import homography_transformer as homography
import get_lanes as imging

import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseArray, PoseStamped, PointStamped, Point
from std_msgs.msg import Float32
from visualization_msgs.msg import Marker
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry


class PurePursuit(object):
    """ Implements Pure Pursuit trajectory tracking with a fixed lookahead and speed.
    """
    def __init__(self):
        rospy.loginfo("Pure pursuit obj initialized!")
        self.bridge = CvBridge()

        self.image_sub = rospy.Subscriber("/zed/zed_node/rgb/image_rect_color", Image, self.image_callback)

        self.drive_pub = rospy.Publisher("/drive", AckermannDriveStamped, queue_size=1)
        self.error_pub = rospy.Publisher("/error", Float32, queue_size=1)

        self.wheelbase_length = 0.325
        self.speed = rospy.get_param("~speed", 1.0)
        self.lookahead = rospy.get_param("~lookahead", 1.0) #0.5 # FILL IN #

        self.update_params()



    def update_params(self):
        self.speed = rospy.get_param("~speed", 0.4)
        self.lookahead = rospy.get_param("~lookahead", 1.0) #0.5 # FILL IN #

        

    def image_callback(self, image_msg): 
        '''
        Inputs: 
            laneArray: A 2xN numpy array of [u,v] coordinates that signify the pixel position of the lanes in ZED camera. 
            [ 
            [[x,y], [x,y], [x,y], [x,y], ... ],
            [[x,y], [x,y], [x,y], [x,y], ... ]
            ]
        Outputs: 
            A drive command to car for it to stay in the lane. 
        '''
        rospy.loginfo("Going into callback")
        img = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")

        realPointx, realPointy = imging.cd_color_segmentation(img)
        rospy.loginfo("grabbed imagepointX and Y")
        realPointx, realPointy = homography.transformUvToXy(realPointx, realPointy)

        steering_angle = np.arctan(abs(realPointy/realPointx))
        # rospy.logerr("mag of steering angle: " + str(steering_angle))
        insideArcTan = 2*self.wheelbase_length*np.sin(steering_angle)/self.lookahead

        # apply direction
        steering_angle = np.arctan(insideArcTan)

        steering_angle *= np.sign(realPointy)

        drive_cmd = AckermannDriveStamped()
        drive_cmd.drive.steering_angle = steering_angle
        drive_cmd.drive.speed = self.speed
        rospy.loginfo("About to publish steering cmds")
        self.drive_pub.publish(drive_cmd)






if __name__=="__main__":
    rospy.init_node("pure_pursuit", log_level = rospy.DEBUG)
    rospy.logerr("init pure pursuit")
    pf = PurePursuit()
    rospy.spin()
    
