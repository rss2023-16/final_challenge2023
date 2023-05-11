#!/usr/bin/env python

import rospy
import numpy as np

from final_challenge2023.msg import ConeLocation, ParkingError
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
        
        self.relative_x = 0
        self.relative_y = 0

        # PID Control Params
        self.last_time = rospy.get_time()
        self.speed_last_error = 0.0
        self.angle_last_error = 0.0
        self.last_speed = 0.0
        self.last_angle = 0.0
        self.last_orange = 0 #in nanosecs   
        
        self.update_params()

    def update_params(self):
        self.angleKp = rospy.get_param("~angleKp")
        self.angleKd = rospy.get_param("~angleKd")
        self.speedKp = rospy.get_param("~speedKp")
        self.speedKd = rospy.get_param("~speedKd")
        self.constantSpeed = rospy.get_param("~constantSpeed")
        self.cornerSpeed = rospy.get_param("~cornerSpeed")
        self.parking_distance = rospy.get_param("~parkingDistance")
        self.timeout = rospy.get_param("~timeout")
        self.lookahead = rospy.get_param("~lookahead", 1.0)
        self.wheelbase_length = rospy.get_param("~wheelbase_length", 0.325)

    def relative_cone_callback(self, msg):
        self.relative_x = msg.x_pos
        self.relative_y = msg.y_pos
        drive_cmd = AckermannDriveStamped()
        #################################

        # YOUR CODE HERE
        # Use relative position and your control law to set drive_cmd
        self.update_params()
        if self.relative_x != 0:
            angle_error = np.sign(self.relative_y)*np.arctan(abs(self.relative_y/self.relative_x))
        else:
            angle_error = 0
        self.curr_dist = np.sqrt(self.relative_y**2 + self.relative_x**2)
        distance_error = self.relative_x - self.parking_distance
        speed, steering_angle = self.pid_control(distance_error, angle_error)
        insideArcTan = 2*self.wheelbase_length*np.sin(steering_angle)/self.lookahead
        steering_angle = np.arctan(insideArcTan)
        # limit speed to 1 m/s
        speed = np.sign(speed)*min(1, abs(speed))

        # check if we should use constant speed:
        if self.constantSpeed > 0.0:
            speed = self.constantSpeed
    
            # if above max steering angle, slow down
            if abs(steering_angle) > 0.34: 
                speed = self.cornerSpeed

        # if speed too small, car doesn't move
        if speed > -0.19 and speed < -0.03:
            speed = -0.19
        if speed > 0.03 and speed < 0.19:
            speed = 0.19
    
        # speed should always be positive
        #insideArcTan = 2*self.wheelbase_length*np.sin(steering_angle)/self.lookahead

        steering_angle *= np.sign(speed)
        print("speed: ", speed, " angle: ", steering_angle)
        if self.relative_x == 0 and self.relative_y == 0:
            speed = -1

        if speed > 0:
            drive_cmd.drive.steering_angle = steering_angle
            drive_cmd.drive.speed = speed
            self.last_speed, self.last_angle = speed, steering_angle
            self.last_orange = rospy.get_rostime().nsecs # in nanoseconds
            self.drive_pub.publish(drive_cmd)
            self.error_publisher()
        else:
            if rospy.get_rostime().nsecs - self.last_orange < self.timeout:
                print("using last drive command!!!!!!!!!!!!")
                drive_cmd.drive.steering_angle = self.last_angle
                drive_cmd.drive.speed = self.last_speed

                self.drive_pub.publish(drive_cmd)



    def pid_control(self, speed_error, angle_error):
        # update PID gains from rosparams

        time = rospy.get_time()
        dt = time - self.last_time

        speed_output = self.speedKp * speed_error + self.speedKd*(speed_error - self.speed_last_error)/dt
        angle_output = self.angleKp * angle_error + self.angleKd*(angle_error - self.angle_last_error)/dt

        # update for next iteration
        self.speed_last_error = speed_error
        self.angle_last_error = angle_error
        self.last_time = time

        return speed_output, angle_output
    

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
