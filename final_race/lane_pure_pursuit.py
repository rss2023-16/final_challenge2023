#!/usr/bin/env python

import rospy
import numpy as np
import time
import utils
import tf

import homography_transformer as homography

from geometry_msgs.msg import PoseArray, PoseStamped, PointStamped, Point
from std_msgs.msg import Float32
from visualization_msgs.msg import Marker
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
from h
import math


class PurePursuit(object):
    """ Implements Pure Pursuit trajectory tracking with a fixed lookahead and speed.
    """
    def __init__(self):
        self.lane_lines = rospy.Subscriber("/laneLines", PoseArray, self.lane_callback, queue_size = 1 )
        
        self.drive_pub = rospy.Publisher("/drive", AckermannDriveStamped, queue_size=1)
        self.error = rospy.Publisher("/error", Float32, queue_size=1)

        #############
        #############
        #############
        #############
        #############
        # self.odom_topic       = rospy.get_param("~odom_topic")
        # self.wheelbase_length = 1.0 # FILL IN #
        # self.trajectory  = utils.LineTrajectory("/followed_trajectory")
        # self.followed_trajectory  = utils.LineTrajectory("/followed_trajectory", color = (1.0, 0.0, 0.0))
        # self.traj_sub = rospy.Subscriber("/trajectory/current", PoseArray, self.trajectory_callback, queue_size=1)
        # self.drive_pub = rospy.Publisher("/drive", AckermannDriveStamped, queue_size=1)
        # self.current_pose = rospy.Subscriber(self.odom_topic, Odometry, self.pose_callback, queue_size = 1 )
        # self.traj = None
        # # Creating some visualization publishers to debug the best point and turning
        # self.two_closest_points_pub = rospy.Publisher("/pure_pursuit/two_closest_points", PointStamped, queue_size=1)
        # self.closest_point_pub = rospy.Publisher("/pure_pursuit/closest_point", PointStamped, queue_size=1)
        # self.point_wrt_car_pub = rospy.Publisher("/pure_pursuit/point_wrt_car", PointStamped, queue_size=1)
        # self.header_stamp = None
        #############
        #############
        #############
        #############
        #############


        # PID Control Params
        self.last_time = rospy.get_time()
        
        self.proportional = 0.0
        self.integral = 0.0
        self.integral_max = 0.5
        self.derivative = 0.0
        self.last_error = 0.0
        self.last_2_error = 0.0

        self.last_angle = 0.0

        # Get transform from map to baselink
        self.listener = tf.TransformListener()

        self.update_params()



    def update_params(self):
        self.kp = rospy.get_param("~kp", 0.4)
        self.ki = rospy.get_param("~ki", 0.0)
        self.kd = rospy.get_param("~kd", 0.0)
        self.lookahead        = rospy.get_param("~lookahead", 1.0) #0.5 # FILL IN #
        self.speed            = rospy.get_param("~speed", 2.0) # FILL IN #
        self.lookaheadclose = rospy.get_param("~lookaheadclose", .3)
        self.lookaheadfarther = rospy.get_param("~lookaheadfarther", .6)
        

    def lane_callback(self, laneArray): 
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
        #convert each laneArray pixel to car frame coordinate. 
        leftLane = laneArray[0, :]
        rightLane = laneArray[1, :]

        leftLaneCarFrame = np.zeros(leftLane.shape)
        rightLaneCarFrame = np.zeros(leftLane.shape)

        for i in leftLane.shape[0]:
            u = leftLane[i,0]
            v = leftLane[i,1]
            x,y = homography.transformUvToXy(u,v)
            leftLaneCarFrame[i] = np.array([x,y])
        for i in rightLane.shape[0]:
            u = rightLane[i,0]
            v = rightLane[i,1]
            x,y = homography.transformUvToXy(u,v)
            rightLaneCarFrame[i] = np.array([x,y])

        #Form of leftLaneCarFrame 
        # [
        #     [x,y],
        #     [x,y],
        #     [x,y],
        # ]
        #Get an array of the middle line between both lanes
        xMidLine = (leftLaneCarFrame[:,0] + rightLaneCarFrame[:,0])/2
        yMidLine = (leftLaneCarFrame[:,1] + rightLaneCarFrame[:,1])/2
        midLine = np.zeros(leftLane.shape)
        imin, distmin = self.find_closest_segment(self, 0, 0, midLine): 





    def frombefore(self, currentPose):
        # rospy.logerr("pose_callback")
        self.header_stamp = currentPose.header.stamp
        
        #update lookahead: 
        self.update_params()

        #Turns trajectory PoseArray into an  np.array of x,y coords.
        try:
            if self.traj:
                # rospy.logerr("self.traj is set!!")
                trajPoses = self.traj
                trajXYCoords = list()
                xcurrent = currentPose.pose.pose.position.x
                ycurrent = currentPose.pose.pose.position.y
                # update followed trajectory
                current_point = Point()
                current_point.x = xcurrent
                current_point.y = ycurrent
                self.followed_trajectory.addPoint(current_point)
                self.followed_trajectory.publish_trajectory()

                angle_current = self.quaternion_to_angle(currentPose.pose.pose.orientation)
                for pose in trajPoses.poses: 
                    trajXYCoords.append([pose.position.x, pose.position.y])
                trajXYCoords = np.array(trajXYCoords)
                imin, distMin = self.find_closest_segment(currentPose.pose.pose.position.x, currentPose.pose.pose.position.y, trajXYCoords)

                speed = self.speed
                angle = None
                lookahead_point, lookahead_angle = self.find_lookahead_point(trajXYCoords, imin, xcurrent, ycurrent, angle_current) #get lookahead point

                if distMin > self.lookahead: 
                    rospy.logerr("min dist > lookahead: min dist is " + str(distMin))
                    #just drive directly to the line 
                    # for now, do nothing if too far


                    drive_cmd = AckermannDriveStamped()
                    drive_cmd.drive.steering_angle = self.last_angle
                    drive_cmd.drive.speed = 0.2
                    self.drive_pub.publish(drive_cmd)

 
                else: 
                    # rospy.logerr("finding lookahead point...")
                    angle = self.pid_control(lookahead_angle)
                    rospy.logerr("Drive Command: Angle: " + str(angle))

                    # create drive command and publish
                    drive_cmd = AckermannDriveStamped()
                    drive_cmd.drive.steering_angle = angle
                    self.last_angle = angle
                    drive_cmd.drive.speed = speed
                    self.drive_pub.publish(drive_cmd)
                
            else:
                rospy.logerr("No traj yet")
                
        except Exception as e:
            rospy.logerr(e)

    def find_closest_segment(self, xcurrent,ycurrent, trajXYCoords):
        '''
        Finds the closest point from the cars current position to the trajectory. 
        Inputs: 
            xcurrent: current x position of the car 
            ycurrent: current y position of the car
        Outputs: 
            A bunch of parameters defining the closest segment to the car. 
        '''
        rospy.logerr("find closest segment")

        A = xcurrent - trajXYCoords[0:trajXYCoords.shape[0]-1, 0]
        B = ycurrent - trajXYCoords[0:trajXYCoords.shape[0]-1, 1]
        C = trajXYCoords[1:trajXYCoords.shape[0], 0] - trajXYCoords[0:trajXYCoords.shape[0]-1, 0]
        D = trajXYCoords[1:trajXYCoords.shape[0], 1] - trajXYCoords[0:trajXYCoords.shape[0]-1, 1]
        
        vectorDot = A*C + B*D
        trajLenSquared = C*C +D*D

        #xx and yy are the points on the line closest to our current pose
        param = vectorDot/trajLenSquared

        xx = np.zeros(param.shape)
        yy = np.zeros(param.shape)

        for i in range(param.shape[0]):
            if param[i] < 0: 
                xx[i] = trajXYCoords[i,0] 
                yy[i] = trajXYCoords[i,1]
            elif param[i] > 1: 
                xx[i] = trajXYCoords[i+1,0]
                yy[i] = trajXYCoords[i+1,1]
            else: 
                xx[i] = trajXYCoords[i,0] + param[i] * C[i]
                yy[i] = trajXYCoords[i,1] + param[i] * D[i]

        dx = xcurrent - xx 
        dy = ycurrent - yy

        distances = (dx**2 + dy**2)**.5
        distMin = np.min(distances)
        imin = np.argmin(distances) #WILL BE THE FIRST x_i YOU STILL NEED x_i+1 for the line

        return imin, distMin

    def find_lookahead_point(self, trajXYCoords, imin, xcurrent, ycurrent, angle_current):
        '''
        @returns: a tuple of the lookahead point and angle as ((x, y), angle)
        '''
        rospy.logerr("find lookahead point")
        firstPoint = trajXYCoords[imin]
        secondPoint = trajXYCoords[imin+1]
        lookahead = self.lookahead

        closestFirstx = firstPoint[0]
        closestFirsty = firstPoint[1]
        closestSecondx = secondPoint[0]
        closestSecondy = secondPoint[1]

        xLongSegMinus = firstPoint[0] - 5 #lookahead*2
        xLongSegPlus = secondPoint[0] + lookahead*2

        m = (closestSecondy - closestFirsty)/(closestSecondx - closestFirstx)
        yLongSegMinus = m * xLongSegMinus + (closestSecondy - m*closestSecondx)
        yLongSegPlus = m * xLongSegPlus + (closestSecondy - m*closestSecondx)

        firstPointExtended = np.array([xLongSegMinus, yLongSegMinus])
        secondPointExtended = np.array([xLongSegPlus, yLongSegPlus])

        Q = np.array([xcurrent, ycurrent])
        p1 = firstPointExtended 
        V = secondPointExtended - p1

        a = np.dot(V, V)
        b = 2 * np.dot(V,p1 - Q)
        c = np.dot(p1,p1) + np.dot(Q,Q) - 2 * np.dot(p1 , Q) - lookahead**2

        disc = b**2 - 4 * a * c
        if disc < 0:
            drive_cmd = AckermannDriveStamped()
            drive_cmd.drive.steering_angle = 0.0
            drive_cmd.drive.speed = 0.0
            self.drive_pub.publish(drive_cmd)
            raise ValueError("There is no point where circle intersects")

        sqrt_disc = np.sqrt(disc)
        t1 = (-b +sqrt_disc) / (2*a)
        t2 = (-b -sqrt_disc) / (2*a)

        # TODO: when to use this?? currently this interrupts corners
        # if not (0 <= t1 <= 1 or 0 <= t2 <= 1):
        #     raise ValueError('You would hit if the line extended')
            
        #right now we return two points which is no bueno
        point1 = p1+t1*V
        point2 = p1+t2*V
        # t = max(0, min(1, -b /(2*a))) # returns closest point from extended line and point (which we already have)
        if (point1[0]<0) and (point2[0] <0):
            rospy.logerr('ERROR!!!!! BOTH POINTS ARE BEHIND CAR')
        elif point1[0] < 0: 
            lookaheadPoint = point2
        elif point2[0] < 0: 
            lookaheadPoint = point1
        else: 
            rospy.logerr('ERROR!!!!! BOTH POINTS ARE IN FRONT CAR')

        # calculate which of the points is closer for the car to TURN to
        angle = self.find_angle_from_car_to_point(xcurrent, ycurrent, angle_current, lookaheadPoint)

    
        # rospy.logerr("rel x, rel y: " + str(relative_x) + ", " + str(relative_y))

        steering_angle = min(0.34, np.arctan(abs(lookaheadPoint[1]/lookaheadPoint[0])))
        rospy.logerr("mag of steering angle: " + str(steering_angle))
        # apply direction
        steering_angle *= np.sign(lookaheadPoint[1])

        return lookaheadPoint, steering_angle
    
        # point_to_axis_angle = self.wrap_angle(self.find_angle_between_vectors([1, 0], best_point))
        # car_heading = self.wrap_angle(angle_current)
        # rospy.logerr("Point to axis angle: " +  str(point_to_axis_angle) + "  :  car heading angle: " + str(car_heading))
        # if car_heading > 0:
        #     if point_to_axis_angle > car_heading: # car should turn in + dir towards point
        #         rospy.logerr("turn +")
        #         return best_point, abs(best_angle)
        #     else: # car should turn in - dir towards point
        #         rospy.logerr("turn -")
        #         return best_point, -1.0*abs(best_angle)
        # else:
        #     if point_to_axis_angle > car_heading: # car should turn in + dir towards point
        #         rospy.logerr("turn -")
        #         return best_point, -1.0 * abs(best_angle)
        #     else: # car should turn in - dir towards point
        #         rospy.logerr("turn +")
        #         return best_point, abs(best_angle)

    def transform_point_to_car_frame(self, car_transform, point):
        (trans, rot) = car_transform # rot is a list of quaternion values
        
        roll, pitch, yaw = tf.transformations.euler_from_quaternion(rot)
        rospy.logerr("trans: " + str(trans) + ", rot: " + str(rot) + ", yaw: " + str(yaw))
        (x, y) = point
        inv_R = np.array([
                [np.cos(yaw), np.sin(yaw)],
                [-np.sin(yaw), np.cos(yaw)]
                ])

        x_diff = x - trans[0]
        y_diff = y - trans[1]
        
        new_x = np.cos(yaw)*x_diff + np.sin(yaw)*y_diff
        new_y = -np.sin(yaw)*x_diff + np.cos(yaw)*y_diff
        return new_x, new_y

    
    def find_angle_from_car_to_point(self, xcurrent, ycurrent, angle_current, point):
        ''' Given the car location as (xcurrent, ycurrent) and the car heading as angle_current, 
            calculate the angle between the car's heading and the point.
            Return this angle
        '''
        # shift points so that car is at origin
        shifted = [point[0] - xcurrent, point[1] - ycurrent]

        # find dot product between car heading and point
        car_point = [np.cos(angle_current), np.sin(angle_current)]

        return self.find_angle_between_vectors(shifted, car_point)

    def find_angle_between_vectors(self, v1, v2):
        '''
        Given two vectors starting from the origin, find the angle between them. 
        v1 dot v2 = |v1| |v2| cos(angle between vectors)
        We can use this formula to solve for the angle

        '''
        return np.arccos(np.dot(v1, v2)/np.linalg.norm(v1)/np.linalg.norm(v2))
    

    def quaternion_to_angle(self, q):
        """Convert a quaternion _message_ into an angle in radians.
        The angle represents the yaw.
        This is not just the z component of the quaternion."""
        x, y, z, w = q.x, q.y, q.z, q.w
        roll, pitch, yaw = tf.transformations.euler_from_quaternion((x, y, z, w))
        return yaw
    
    def wrap_angle(self, angle):
        '''
        Keep angle in range [-pi, pi]
        '''
        return (angle + np.pi) % (2 * np.pi) - np.pi
    
    def pid_control(self, err):
        # update PID gains from rosparams
        rospy.logerr("YOU ENTER ERROR MY GUY")
        # error = PointStamped() 
        # error.point.x = err 
        # error.header.stamp = rospy.Time.now() 
        self.error.publish(err) 
        return err

        # TODO: use pid stuff
        self.kp = rospy.get_param("~kp")
        self.ki = rospy.get_param("~ki")
        self.kd = rospy.get_param("~kd")

        time = rospy.get_time()
        dt = time - self.last_time
        
        self.proportional = self.kp * error
        if self.integral > self.integral_max:
            self.integral = self.integral_max
        if self.integral < -self.integral_max:
            self.integral = -self.integral_max
        self.integral += self.ki * error * float(dt)
        self.derivative = self.kd * (error-self.last_error)/dt
    
        self.last_error = error
        self.last_2_error = self.last_error
        self.last_time = time

        output = self.proportional + self.integral + self.derivative

        return output



if __name__=="__main__":
    rospy.init_node("pure_pursuit", log_level = rospy.DEBUG)
    rospy.logerr("init pure pursuit")
    pf = PurePursuit()
    rospy.spin()
    