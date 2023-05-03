import cv2
import numpy as np
import rospy

from ackermann_msgs.msg import AckermannDriveStamped
from stop_detector.stop_detector import SignDetector


class StopSign():
     def __init__(self):
        rospy.Subscriber("/relative_cone", SignDetector, self.stop_detection_callback)
        DRIVE_TOPIC = rospy.get_param("~drive_topic") # set in launch file; different for simulator vs racecar
        self.drive_pub = rospy.Publisher(DRIVE_TOPIC, AckermannDriveStamped, queue_size=10)

    def stop_detection_callback():
        
