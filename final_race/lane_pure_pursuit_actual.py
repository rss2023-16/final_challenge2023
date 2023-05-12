#!/usr/bin/env python

import rospy
import numpy as np
import time

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

        self.drive = rospy.get_param("~drive_topic")

        self.bridge = CvBridge()

        self.image_sub = rospy.Subscriber("/zed/zed_node/rgb/image_rect_color", Image, self.image_callback)

        self.drive_pub = rospy.Publisher(self.drive, AckermannDriveStamped, queue_size=1)
        self.error_pub = rospy.Publisher("/error", Float32, queue_size=1)
        self.debug_pub = rospy.Publisher("/cone_debug_img", Image, queue_size=10)
        self.wheelbase_length = 0.325
        self.speed = rospy.get_param("~speed", 1.0)
        self.p = rospy.get_param("~prop", 1.0)

        self.lookahead = rospy.get_param("~lookahead", 1.0) #0.5 # FILL IN #
        self.h = np.array([[-6.17702706e-05,  1.99202338e-04, -4.35124244e-01], 
                           [ 1.21539529e-03, -2.65671832e-05, -4.01352071e-01],
                           [-8.38281951e-05, -6.51686963e-03,  1.00000000e+00]])
        self.prev_steering_angle = 0
        self.update_params()
        self.last_time = rospy.get_time()
        self.last_angle_err = 0.0

    def update_params(self):
        self.speed = rospy.get_param("~speed", 0.4)
        self.lookahead = rospy.get_param("~lookahead", 1.0) #0.5 # FILL IN #
        self.height = rospy.get_param("~height", 0.3)
        self.p = rospy.get_param("~prop", 1.0) 
        self.angleKp = rospy.get_param("~angleKp")
        self.angleKd = rospy.get_param("~angleKd")
        self.angleOffset = rospy.get_param("~angleOffset")

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
        self.update_params()
        #Create Drive Command# 
        drive_cmd = AckermannDriveStamped()

        # rospy.loginfo("Going into callback")
        img = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")
   
        imgLenX = len(img[0])
        imgLenY = len(img)
        bottomMidY = imgLenY
        bottomMidX = imgLenX/2
        before_time = rospy.get_time()
        segmentation_result = imging.cd_color_segmentation(img, 1.0 - self.height)
        after_time = rospy.get_time()
        print(after_time - before_time)
        if segmentation_result == None: 
            drive_cmd.drive.steering_angle = self.p * self.prev_steering_angle
            drive_cmd.drive.speed = self.speed
           # rospy.loginfo("Could not find lookaheadPoint. Publishing previous steering cmd.")
            self.drive_pub.publish(drive_cmd)
            return 
        
        u, v, rho1, theta1, rho2, theta2 = segmentation_result
        
       # print(u, v, "I am pixel coords")
        realPointx, realPointy = self.transformUvToXy(u, v)
       # print(realPointx,realPointy, "I AM REAL X AND Y IN CAR FRAME")
        self.steering_angle = np.arctan(abs(realPointy/realPointx))
        # rospy.logerr("mag of steering angle: " + str(steering_angle))
        insideArcTan = 2*self.wheelbase_length*np.sin(self.steering_angle)/realPointx

        # apply direction
        self.steering_angle = np.arctan(insideArcTan)

        self.steering_angle *= np.sign(realPointy)

        drive_cmd.drive.steering_angle = self.angleOffset + self.pid(self.steering_angle)
        drive_cmd.drive.speed = self.speed
        #rospy.loginfo("About to publish steering cmds")
        #rospy.loginfo(drive_cmd)
        #print(drive_cmd)
        self.drive_pub.publish(drive_cmd)
        self.prev_steering_angle = self.steering_angle
        
        img = imging.show_lanes(rho1, theta1, rho2, theta2, img)
        box_radius = 10
        img = cv2.rectangle(img, (int(u)-box_radius, int(v)-box_radius), (int(u)+box_radius, int(v)+box_radius), (0, 255, 0), 2)
        # img = cv2.line(img, (realPointx-1, realPointy-1), (realPointx+1, realPointy+1), (0, 255, 0), 2)
        # debug_msg = self.bridge.cv2_to_imgmsg(img, "bgr8")
        # self.debug_pub.publish(debug_msg)

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
        homogeneous_point = np.array([[u], [v], [1]])
        xy = np.dot(self.h, homogeneous_point)
        scaling_factor = 1.0 / xy[2, 0]
        homogeneous_xy = xy * scaling_factor
        x = homogeneous_xy[0, 0]
        y = homogeneous_xy[1, 0]
        return x, y

    def pid(self, angle):
        time = rospy.get_time()
        dt = time - self.last_time

        angle_output = self.angleKp*angle + self.angleKd*(angle-self.last_angle_err)/dt

        self.last_angle_err = angle
        self.last_time = time

        return angle_output


if __name__=="__main__":
    rospy.init_node("pure_pursuit", log_level = rospy.DEBUG)
    rospy.logerr("init pure pursuit")
    pf = PurePursuit()
    rospy.spin()
    
