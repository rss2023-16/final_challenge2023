import cv2
import rospy

import numpy as np
from sensor_msgs.msg import Image
from detector import StopSignDetector

from final_challenge2023.msg import ConeLocation, ConeLocationPixel

class SignDetector:
    def __init__(self):
        self.detector = StopSignDetector()
        self.publisher = rospy.Publisher("/stop_sign_px", ConeLocationPixel, queue_size=1) #TODO
        self.subscriber = rospy.Subscriber("/zed/zed_node/rgb/image_rect_color", Image, self.callback)

    def callback(self, img_msg):
        # Process image without CV Bridge
        np_img = np.frombuffer(img_msg.data, dtype=np.uint8).reshape(img_msg.height, img_msg.width, -1)
        bgr_img = np_img[:,:,:-1]
        rgb_img = cv2.cvtColor(bgr_img, cv2.COLOR_BGR2RGB)

        #TODO: 
        detected, bounding_box = self.detector.predict(rgb_img) # x_min, y_min, x_max, y_max

        if detected:
            x_min, y_min, x_max, y_max = bounding_box

            u = (x_min + x_max)//2
            v = y_max

            stop_sign_location = ConeLocationPixel()
            stop_sign_location.u = u
            stop_sign_location.v = v
            self.publisher.publish(stop_sign_location)


if __name__=="__main__":
    rospy.init_node("stop_sign_detector")
    detect = SignDetector()
    rospy.spin()
