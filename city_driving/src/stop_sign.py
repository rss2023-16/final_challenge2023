import cv2
import numpy as np
import rospy

from ackermann_msgs.msg import AckermannDriveStamped
from stop_detector.stop_detector import SignDetector

from final_challenge2023.msg import ConeLocation, ConeLocationPixel


class StopSignController():
    def __init__(self):
        rospy.Subscriber("/stop_sign", ConeLocation, self.stop_detection_callback)
        DRIVE_TOPIC = rospy.get_param("~drive_topic") # set in launch file; different for simulator vs racecar
        self.drive_pub = rospy.Publisher(DRIVE_TOPIC, AckermannDriveStamped, queue_size=10)

    def stop_detection_callback(self, pos):
        a = AckermannDriveStamped()
        a.header.stamp = rospy.Time.now()

        a.drive.speed = 0 #TODO
        a.drive.steering_angle = 0 #TODO

        self.drive_pub.publish(a)
        pass

if __name__ == '__main__':
    try:
        rospy.init_node('StopSignController', anonymous=True)
        StopSignController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
