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


class PurePursuit(object):
    """ Implements Pure Pursuit trajectory tracking with a fixed lookahead and speed.
    """
    def __init__(self):
        self.point = rospy.Subscriber("/laneLines", PoseArray, self.point_callback, queue_size = 1 )
        self.drive_pub = rospy.Publisher("/drive", AckermannDriveStamped, queue_size=1)
        self.error_pub = rospy.Publisher("/error", Float32, queue_size=1)

        self.speed = rospy.get_param("~speed", 1.0)
        self.lookahead = rospy.get_param("~lookahead", 1.0) #0.5 # FILL IN #

        self.update_params()



    def update_params(self):
        self.speed = rospy.get_param("~speed", 0.4)
        self.lookahead = rospy.get_param("~lookahead", 1.0) #0.5 # FILL IN #

        

    def point_callback(self, point): 
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
        realPointx, realPointy = homography.transformUvtoXy(point[0],point[1])
        #convert each laneArray pixel to car frame  xy coordinate

        steering_angle = min(0.34, np.arctan(abs(realPointy/realPointx)))
        rospy.logerr("mag of steering angle: " + str(steering_angle))

        # apply direction
        steering_angle *= np.sign(realPointy)
        
        drive_cmd = AckermannDriveStamped()
        drive_cmd.drive.steering_angle = steering_angle
        drive_cmd.drive.speed = self.speed
        self.drive_pub.publish(drive_cmd)






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
            trajXYCoords: np.array for [[x,y],[x,y]...] coordinates of trajectory
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
        ################ Math to find the lookahead point. ################
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
        ###################################################################
        # Two intersections are found
        point1 = p1+t1*V
        point2 = p1+t2*V

        # We see which points is in front of the car and pick that one since thats the direction we're driving in//
        if (point1[0]<0) and (point2[0] <0):
            rospy.logerr('ERROR!!!!! BOTH POINTS ARE BEHIND CAR')
        elif (point1[0]>0) and (point2[0] >0): 
            rospy.logerr('ERROR!!!!! BOTH POINTS ARE IN FRONT CAR')
        elif point1[0] < 0: 
            lookaheadPoint = point2
        else: # point2[0] < 0: 
            lookaheadPoint = point1


        steering_angle = min(0.34, np.arctan(abs(lookaheadPoint[1]/lookaheadPoint[0])))
        rospy.logerr("mag of steering angle: " + str(steering_angle))

        # apply direction
        steering_angle *= np.sign(lookaheadPoint[1])

        return lookaheadPoint, steering_angle
    
    def error_pub(self, err):
        # update PID gains from rosparams
        rospy.logerr("YOU ENTER ERROR MY GUY")
        # error = PointStamped() 
        # error.point.x = err 
        # error.header.stamp = rospy.Time.now() 
        self.error_pub.publish(err) 
        return err


if __name__=="__main__":
    rospy.init_node("pure_pursuit", log_level = rospy.DEBUG)
    rospy.logerr("init pure pursuit")
    pf = PurePursuit()
    rospy.spin()
    