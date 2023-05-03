#!/usr/bin/env python

import rospy
import numpy as np

from visual_servoing.msg import ConeLocation, ParkingError
from ackermann_msgs.msg import AckermannDriveStamped

class ParkingController():
    """
    A controller for parking in front of a cone.
    Listens for a relative cone location and publishes control commands.
    Can be used in the simulator and on the real robot.
    """
    def __init__(self):
        rospy.Subscriber("/relative_cone", ConeLocation,
            self.relative_cone_callback)

        DRIVE_TOPIC = rospy.get_param("~drive_topic") # set in launch file; different for simulator vs racecar
        self.drive_pub = rospy.Publisher(DRIVE_TOPIC,
            AckermannDriveStamped, queue_size=10)
        self.error_pub = rospy.Publisher("/parking_error",
            ParkingError, queue_size=10)

        self.parking_distance = 0 #.5 # meters; try playing with this number!
        self.relative_x = 0
        self.relative_y = 0
        self.back_up = False

    def relative_cone_callback(self, msg):
        self.relative_x = msg.x_pos
        self.relative_y = msg.y_pos
        drive_cmd = AckermannDriveStamped()
        #################################

        # YOUR CODE HERE
        # Use relative position and your control law to set drive_cmd
        angle_divider = 1 #0.75
        
        steering_angle = np.sign(self.relative_y)*np.arctan(abs(self.relative_y/self.relative_x))
        self.curr_dist = np.sqrt(self.relative_y**2 + self.relative_x**2)
        distance_error = self.curr_dist - self.parking_distance
        speed = 1.5*(self.relative_x - self.parking_distance)
	

        use_backup = False 
        if use_backup:
            if self.back_up:
                #    stopping backing up if far enough to finish turning
                if distance_error > 0.25*self.parking_distance:
                    self.back_up = False
                else:
                    speed = -0.3

            if not self.back_up:

                #    if too close, back up
                if abs(steering_angle) > 0.05 and abs(distance_error) < 0.05:
		    print("start backing up")
                    self.back_up = True
                    speed = -0.3


            # limit speed to 1 m/s
        speed = np.sign(speed)*min(1, abs(speed))
	
	# if speed too small, car doesn't move
	if speed > -0.19 and speed < -0.03:
		speed = -0.19
	if speed > 0.03 and speed < 0.19:
		speed = 0.19
	
	steering_angle *= np.sign(speed)
	print("speed: ", speed, " angle: ", steering_angle)
        drive_cmd.drive.steering_angle = steering_angle
        drive_cmd.drive.speed = speed

        #################################
        self.drive_pub.publish(drive_cmd)
        self.error_publisher()

    def error_publisher(self):
        """
        Publish the error between the car and the cone. We will view this
        with rqt_plot to plot the success of the controller
        """
        error_msg = ParkingError()

        #################################

        # YOUR CODE HERE
        # Populate error_msg with relative_x, relative_y, sqrt(x^2+y^2)
        error_msg.x_error = (self.parking_distance - self.relative_x)
        error_msg.y_error = (0 - self.relative_y)
        error_msg.distance_error = (self.parking_distance - self.curr_dist)

        #################################

        self.error_pub.publish(error_msg)

if __name__ == '__main__':
    try:
        rospy.init_node('ParkingController', anonymous=True)
        ParkingController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
