#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Float64
import random
import math
import tf
import numpy as np

class TurtleBot3LineOscillator:
    def __init__(self):
        rospy.init_node('line1')

        # Parameters
        self.velocity_scaling = rospy.get_param('~velocity_scaling', 1.0)

        self.offset_x = 3.5
        self.offset_y = 3.5
        self.height = 2.25

        # Publishers and Subscribers
        self.cmd_pub = rospy.Publisher('/hydrone_aerial_underwater2/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('/hydrone_aerial_underwater2/odometry_sensor1/odometry', Odometry, self.odom_callback)
        rospy.Subscriber('/hydrone_aerial_underwater2/uncertainty_reset', Bool, self.update_uncertainty)
        self.unc_pub = rospy.Publisher('/hydrone_aerial_underwater2/uncertainty', Float64, queue_size=10)

        # Timer for control loop
        self.control_timer = rospy.Timer(rospy.Duration(0.05), self.controller_callback)

        # Robot state
        self.x = self.offset_x
        self.y = self.offset_y
        self.z = 0.0
        self.theta = 0.0

        # Robot pose
        self.x = 0.0
        self.theta = 0.0

        # Motion
        self.max_speed = 0.22  # m/s
        self.start_x = None
        self.direction = 1  # 1 for forward, -1 for backward
        self.travel_distance = 3.0  # meters to travel before reversing
        self.epsilon = 0.05  # distance threshold to reverse

        rospy.loginfo("TurtleBot3 Line Oscillator Initialized")

        self.uncertainty = np.random.uniform(0.5, 5)

    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.z = msg.pose.pose.position.z

        # Get yaw from quaternion
        q = msg.pose.pose.orientation
        orientation_list = [q.x, q.y, q.z, q.w]
        _, _, self.theta = tf.transformations.euler_from_quaternion(orientation_list)

        if self.start_x is None:
            self.start_x = self.x  # Set reference start position at first odom read


    def update_uncertainty(self, msg):
        self.uncertainty = 0.5*self.uncertainty

        rospy.loginfo("Uncertainty updated" + str(self.uncertainty))

    def controller_callback(self, event):
        # Update parameter dynamically
        self.velocity_scaling = rospy.get_param('~velocity_scaling', 1.0)
        scaling = max(0.0, min(1.0, self.velocity_scaling))
        v = 0.22 * np.random.uniform(0.5, 1) * scaling
        if v < 1e-3:
            v = 0.0

        # Determine current target based on direction
        target_x = self.start_x + self.travel_distance * self.direction
        dx = self.x - target_x

        # Check if we are close to the target
        if abs(dx) < self.epsilon:
            self.direction *= -1
            rospy.loginfo("Reached target. Reversing direction. Now heading %s", "forward" if self.direction == 1 else "backward")

        # Move toward current direction
        twist = Twist()
        twist.linear.x = v * self.direction
        twist.angular.z = 0.0
        
        if self.z >= self.height + 0.05:
            twist.linear.z = -0.05
        elif self.z <= self.height - 0.05:
            twist.linear.z = 0.05
        else:
            twist.linear.z = 0.0

        self.cmd_pub.publish(twist)

        rospy.loginfo("x=%.2f | dx=%.2f | direction=%s", self.x, dx, "forward" if self.direction == 1 else "backward")

        self.uncertainty = self.uncertainty + 0.01

        self.unc_pub.publish(self.uncertainty)

        rospy.loginfo("Uncertainty" + str(self.uncertainty))

if __name__ == '__main__':
    try:
        TurtleBot3LineOscillator()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
