#!/usr/bin/env python

import rospy
import csv
import os
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid, Path
import numpy as np
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import math

def draw_racing_line(filename):
    with open(filename) as f:
        path_points = [tuple(line) for line in csv.reader(f)]
    path_points = [(float(point[0]), float(point[1])) for point in path_points]
    path_points_pos_x = [float(point[0]) for point in path_points]
    path_points_pos_y = [float(point[1]) for point in path_points]
    path_yaw = []
    for i in range(len(path_points_pos_x)-1):
        vec = [path_points_pos_x[i+1] - path_points_pos_x[i], path_points_pos_y[i+1] - path_points_pos_y[i]]
        if vec[0] > 0:
            if vec[1] > 0:
                yaw = math.atan(vec[1]/vec[0])
            else:
                yaw = math.atan(vec[1]/vec[0])
        elif vec[0] < 0:
            if vec[1] > 0:
                yaw = math.pi + math.atan(vec[1]/vec[0])
            else:
                yaw = -math.pi + math.atan(vec[1]/vec[0])
        else:
            if vec[1] > 0:
                yaw = 0.5*math.pi
            else:
                yaw = -0.5*math.pi
        path_yaw.append(yaw)

    vec = [path_points_pos_x[-1] - path_points_pos_x[0], path_points_pos_y[-1] - path_points_pos_y[0]]
    if vec[0] > 0:
        if vec[1] > 0:
            yaw = math.atan(vec[1]/vec[0])
        else:
            yaw = math.atan(vec[1]/vec[0])
    else:
        if vec[1] > 0:
            yaw = math.pi + math.atan(vec[1]/vec[0])
        else:
            yaw = -math.pi + math.atan(vec[1]/vec[1])
    path_yaw.append(yaw)
    path_info = [path_points_pos_x, path_points_pos_y, path_yaw]
    return path_info

class path_generation():
    def __init__(self, racing_line):
        self.odometry_subscriber = rospy.Subscriber('/odom', Odometry, self.callback_read_position, queue_size = 1)
        self.costmap_subscriber = rospy.Subscriber('/move_base/global_costmap/costmap', OccupancyGrid, self.callback_map_info, queue_size = 1)
        self.goal_publisher = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size = 1)
        self.path_pose = rospy.Subscriber('/move_base/GlobalPlanner/plan', Path, self.callback_read_path, queue_size=3)
        self.costmap_info = []
        self.path_info = []
        self.temp_goal = []
        self.odom_pose = []
        self.local_goal = 0
        self.racing_line = racing_line
        self.frame_id = 'map'
        self.map_resolution = [];
        self.map_width = [];
        self.map_height = [];
        self.map_origin_x = [];
        self.map_origin_y = [];
    
    def callback_read_path(self, data):
        self.path_info = []
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
            self.path_info.append([float(path_x), float(path_y), float(path_yaw)])

    def callback_read_position(self, data):
        x = data.pose.pose.position.x
        y = data.pose.pose.position.y
        qx = data.pose.pose.orientation.x
        qy = data.pose.pose.orientation.y
        qz = data.pose.pose.orientation.z
        qw = data.pose.pose.orientation.w

        quaternion = (qx, qy, qz, qw)
        euler = euler_from_quaternion(quaternion)
        yaw = euler[2]
        robot_pose = np.array([x,y,yaw])

        local_goal = self.find_local_goal(robot_pose, ahead_distance=4.0)
        
        if not local_goal == []:
            self.local_goal = local_goal

        goal_x = self.racing_line[0][self.local_goal]
        goal_y = self.racing_line[1][self.local_goal]
        goal_yaw = self.racing_line[2][self.local_goal]
        cell_info = self.obstacle_check(goal_x, goal_y, goal_yaw)
        print(cell_info)

        while cell_info > 5:
            self.local_goal = self.local_goal + 5
            goal_x = self.racing_line[0][self.local_goal]
            goal_y = self.racing_line[1][self.local_goal]
            goal_yaw = self.racing_line[2][self.local_goal]
            cell_info = self.obstacle_check(goal_x, goal_y, goal_yaw)
        path_quaternion = quaternion_from_euler(0.0, 0.0, goal_yaw)
        
        if len(self.path_info) == 0:
            goal_pose = PoseStamped()
            goal_pose.header.frame_id = self.frame_id
            goal_pose.pose.position.x = goal_x
            goal_pose.pose.position.y = goal_y
            goal_pose.pose.orientation.x = path_quaternion[0]
            goal_pose.pose.orientation.y = path_quaternion[1]
            goal_pose.pose.orientation.z = path_quaternion[2]
            goal_pose.pose.orientation.w = path_quaternion[3]
            self.goal_publisher.publish(goal_pose)
            self.temp_goal = [goal_x, goal_y, goal_yaw]

        # Publish local goal to move_base_simple/goal to give information to local planner.
        if ((self.temp_goal[0] - robot_pose[0])**2 + (self.temp_goal[1] - robot_pose[1])**2)**0.5 < 3.7:
            goal_pose = PoseStamped()
            goal_pose.header.frame_id = self.frame_id
            goal_pose.pose.position.x = goal_x
            goal_pose.pose.position.y = goal_y
            goal_pose.pose.orientation.x = path_quaternion[0]
            goal_pose.pose.orientation.y = path_quaternion[1]
            goal_pose.pose.orientation.z = path_quaternion[2]
            goal_pose.pose.orientation.w = path_quaternion[3]
            self.goal_publisher.publish(goal_pose)
            rospy.sleep(0.15)
            self.temp_goal = [goal_x, goal_y, goal_yaw]

        #print('x: ' + str(self.temp_goal[0]) + ' y: ' + str(self.temp_goal[1]) + ' yaw: ' +  str(self.temp_goal[2]))

    def find_local_goal(self, current_pose, ahead_distance = 2.5):
        path_pose_x = self.racing_line[0]
        path_pose_y = self.racing_line[1]
        
        # Compute the distance between current_pose and path point within a certain distance.
        dist_array = np.zeros(len(path_pose_x))
        for i in range(len(path_pose_x)):
            dist_array[i] = ((path_pose_x[i] - current_pose[0])**2 + (path_pose_y[i] - current_pose[1])**2)**0.5
        goal_array = np.where((dist_array < (ahead_distance + 0.4)) & (dist_array > (ahead_distance - 0.2)))[0]
        
        # Compute the angle between candidate pose and current pose
        angle_array = np.zeros(len(goal_array))
        for i in range(len(angle_array)):
            v1 = [float(path_pose_x[goal_array[i]] - current_pose[0]), float(path_pose_y[goal_array[i]] - current_pose[1])]
            v2 = [math.cos(current_pose[2]), math.sin(current_pose[2])]
            angle = self.find_angle(v1,v2)
            angle_array[i] = abs(angle)
        
        # Choose local goal from angle_array
        if len(angle_array) == 0:
            return []
        else:
            goal_id = np.argmin(angle_array)
            goal = goal_array[goal_id]
            return goal
    
    def obstacle_check(self, goal_x, goal_y, goal_yaw):
        # Collision check of goal location
        if len(self.costmap_info) == 0:
            return 0
        else: 
            cell_id_x = np.clip(int((goal_x - self.map_origin_x)/self.map_resolution), 0, int(self.map_width/self.map_resolution))
            cell_id_y = np.clip(int((goal_y - self.map_origin_y)/self.map_resolution), 0, int(self.map_height/self.map_resolution))
            cell_id = cell_id_y * self.map_width + cell_id_x
            return self.costmap_info[cell_id]

    def callback_map_info(self, data):
        self.map_resolution = data.info.resolution
        self.map_width = data.info.width
        self.map_height = data.info.height
        self.map_origin_x = data.info.origin.position.x
        self.map_origin_y = data.info.origin.position.y
        self.costmap_info = data.data

    def find_angle(self, v1, v2):
        cos_ang = np.dot(v1,v2)
        sin_ang = np.linalg.norm(np.cross(v1,v2))
        return np.arctan2(sin_ang, cos_ang)

if __name__ == '__main__':
    rospy.init_node('path_generation')
    dirname = os.path.dirname(__file__)
    filename = os.path.join(dirname,'berlin_fast_2.csv')
    path_points = draw_racing_line(filename)
    path_generation(path_points)
    rospy.spin()
