#!/usr/bin/env python
import rospy
from nav_msgs.msg import Path, Odometry
from sensor_msgs.msg import Joy
from ackermann_msgs.msg import AckermannDriveStamped
import math
import numpy as np

class joystick_control():
    def __init__(self):
        self.speed_input = 0.0
        self.steering_input = 0.0
        self.max_acc = 3.0
        self.max_steering = 0.4189
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.callback_read_current_position, queue_size=2)
        self.joy_sub = rospy.Subscriber('/joy', Joy, self.callback_joy)
        self.control_input = rospy.Publisher('/drive', AckermannDriveStamped, queue_size=1)

    def callback_read_current_position(self, data):
        current_speed = data.twist.twist.linear.x
        # Convert joystick input to velocity/steering angle input
        if current_speed > 0:
            velocity = min((self.speed_input*self.max_acc + current_speed), 5.0)
        else:
            velocity = self.speed_input*2.0
        steering_angle = self.steering_input*self.max_steering
        ack_msg = AckermannDriveStamped()
        ack_msg.drive.speed = velocity
        ack_msg.drive.steering_angle = steering_angle
        self.control_input.publish(ack_msg)
        
    def callback_joy(self, data):
        self.speed_input = data.axes[1]
        self.steering_input = data.axes[2]
        

if __name__ == "__main__":
    rospy.init_node('joystick_control')
    joystick_control()
    rospy.spin()
