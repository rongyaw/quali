#!/usr/bin/env python

import rospy
from nav_msgs.msg import Path, Odometry
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import PoseStamped, PoseArray, PoseWithCovarianceStamped
from sensor_msgs.msg import LaserScan
import math
import numpy as np
from numpy import linalg as LA
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import csv
import os

class following_path:
    def __init__(self):
        self.current_pose = rospy.Subscriber('/odom', Odometry, self.callback_read_current_position, queue_size=2)
        self.odometry_subscriber = rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.callback_read_amcl_position, queue_size = 1)
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback, queue_size=1)
        self.Pose = []
        self.yaw_sum = 0
        self.amcl_pose = []
        self.path_pose = rospy.Subscriber('/move_base/GlobalPlanner/plan', Path, self.callback_read_path, queue_size=2)
        self.path_info = []
        self.Goal = []
        self.navigation_input = rospy.Publisher('/drive', AckermannDriveStamped, queue_size=1)
        self.MAX_VELOCITY = 8.5
        self.MIN_VELOCITY = 2.5
        self.max_angle = 0.42
        self.LOOKAHEAD_DISTANCE = 1.5
        self.Low_Speed_Mode = False
    
    def scan_callback(self, scan_msg):
        laser_range = scan_msg.ranges
        if min(laser_range) < 0.4:
            self.Low_Speed_Mode = True
        else:
            self.Low_Speed_Mode = False

    def callback_read_path(self, data):
        # Organize the pose message and only ask for (x,y) and orientation
        # Read the Real time pose message and load them into path_info
        self.path_info = []
        self.yaw_sum = 0
        yaw_data = []
        path_array = data.poses
        for path_pose in path_array:
            path_x = path_pose.pose.position.x
            path_y = path_pose.pose.position.y
            path_qx = path_pose.pose.orientation.x
            path_qy = path_pose.pose.orientation.y
            path_qz = path_pose.pose.orientation.z
            path_qw = path_pose.pose.orientation.w
            path_quaternion = (path_qx, path_qy, path_qz, path_qw)
            path_euler = euler_from_quaternion(path_quaternion)
            path_yaw = path_euler[2]
            yaw_data.append(path_yaw)
            self.path_info.append([float(path_x), float(path_y), float(path_yaw)])
        self.Goal = list(self.path_info[-1]) # Set the last pose of the global path as goal location
        yaw_data = np.array(yaw_data)
        for i in range(len(yaw_data)-1):
            self.yaw_sum = self.yaw_sum + abs(yaw_data[i+1] - yaw_data[i])

    def callback_read_current_position(self, data):
        if not len(self.path_info) == 0:
            # Read the path information to path_point list
            path_points_x = [float(point[0]) for point in self.path_info]
            path_points_y = [float(point[1]) for point in self.path_info]
            path_points_w = [float(point[2]) for point in self.path_info]

            # Read the current pose of the car from particle filter
            x = data.pose.pose.position.x
            y = data.pose.pose.position.y
            qx = data.pose.pose.orientation.x
            qy = data.pose.pose.orientation.y
            qz = data.pose.pose.orientation.z
            qw = data.pose.pose.orientation.w

            # Convert the quaternion angle to eular angle
            quaternion = (qx,qy,qz,qw)
            euler = euler_from_quaternion(quaternion)
            yaw = euler[2]
            self.Pose = np.array([float(x), float(y), float(yaw)])

            if np.linalg.norm(self.amcl_pose - self.Pose) > 0.02:
                self.Pose = self.amcl_pose

            # 2. Find the path point closest to the vehichle tat is >= 1 lookahead distance from vehicle's current location.
            dist_array = np.zeros(len(path_points_x))

            for i in range(len(path_points_x)):
                dist_array[i] = self.dist((path_points_x[i], path_points_y[i]), self.Pose)
            
            goal = np.argmin(dist_array) # Assume the closet point as the goal point at first
            goal_array = np.where((dist_array < (self.LOOKAHEAD_DISTANCE + 0.1)) & (dist_array > (self.LOOKAHEAD_DISTANCE - 0.1)))[0]
            for id in goal_array:
                v1 = [path_points_x[id] - x, path_points_y[id] - y]
                v2 = [math.cos(yaw), math.sin(yaw)]
                diff_angle = self.find_angle(v1,v2)
                if abs(diff_angle) < np.pi/6: # Check if the one that is the cloest to the lookahead direction
                    goal = id
                    break

            L = dist_array[goal]
            # 3. Transform the goal point to vehicle coordinates. 
            glob_x = path_points_x[goal] - x
            glob_y = path_points_y[goal] - y
            goal_x_veh_coord = glob_x*np.cos(yaw) + glob_y*np.sin(yaw)
            goal_y_veh_coord = glob_y*np.cos(yaw) - glob_x*np.sin(yaw)

            # 4. Calculate the curvature = 1/r = 2x/l^2
            # The curvature is transformed into steering wheel angle by the vehicle on board controller.
            # Hint: You may need to flip to negative because for the VESC a right steering angle has a negative value.
            
            diff_angle = path_points_w[goal] - yaw # Find the turning angle
            r = L/(2*math.sin(diff_angle)) # Calculate the turning radius
            angle = math.atan(0.3/r) # Find the wheel turning radius

            angle = np.clip(angle, -self.max_angle, self.max_angle)
            #angle = (0 if abs(angle) < 0.05 else angle)
            VELOCITY = self.speed_control(angle)
            self.lookahead_distance_control()

            # Write the Velocity and angle data into the ackermann message
            ackermann_control = AckermannDriveStamped()
            ackermann_control.drive.speed = VELOCITY
            ackermann_control.drive.acceleration = 9.51
            ackermann_control.drive.steering_angle_velocity = 3.2
            ackermann_control.drive.steering_angle = angle
        else:
            ackermann_control = AckermannDriveStamped()
            ackermann_control.drive.speed = 0.0
            ackermann_control.drive.acceleration = 0.0
            ackermann_control.drive.steering_angle = 0.0
            ackermann_control.drive.steering_angle_velocity = 0.0
        
        self.navigation_input.publish(ackermann_control)
    
    def callback_read_amcl_position(self, data):
        amcl_x = data.pose.pose.position.x
        amcl_y = data.pose.pose.position.y
        amcl_ori_x = data.pose.pose.orientation.x
        amcl_ori_y = data.pose.pose.orientation.y
        amcl_ori_z = data.pose.pose.orientation.z
        amcl_ori_w = data.pose.pose.orientation.w
        quaternion = (amcl_ori_x, amcl_ori_y, amcl_ori_z, amcl_ori_w)
        euler = euler_from_quaternion(quaternion)
        yaw = euler[2]
        self.amcl_pose = np.array([amcl_x, amcl_y, yaw])

    # Computes the Euclidean distance between two 2D points p1 and p2
    def dist(self, p1, p2):
        return np.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2)

    # Compute the angle between car direction and goal direction
    def find_angle(self, v1, v2):
        cos_ang = np.dot(v1, v2)
        sin_ang = LA.norm(np.cross(v1, v2))
        return np.arctan2(sin_ang, cos_ang)

    # Control the speed of the car within the speed limit
    def speed_control(self, angle):
        # Assume the speed change linearly with respect to yaw angle
        if self.Low_Speed_Mode:
            Velocity = 2.5
            print('Low Speed on.')
        else:
            k = (self.MAX_VELOCITY - self.MIN_VELOCITY - 1.25)/self.max_angle
            Velocity = -k*abs(angle) + self.MAX_VELOCITY
        return Velocity
        print('Look ahead distance is ' + str(self.LOOKAHEAD_DISTANCE) + ' m with speed of ' + str(Velocity) + ' m/s.')
        
    def lookahead_distance_control(self):
        self.LOOKAHEAD_DISTANCE = 1.5 - 0.85*math.atan(self.yaw_sum)/(math.pi/2)
        
if __name__ == "__main__":
    rospy.init_node("pursuit_path")
    following_path()
    rospy.spin()