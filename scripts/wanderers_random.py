#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Float64
import random
import math
import tf
import numpy as np

class LissajousFollower:
    def __init__(self):
        rospy.init_node('lissajous_follower')

        # Parameters
        self.velocity_scaling = rospy.get_param('~velocity_scaling', 1.0)

        self.offset_x = 0.0
        self.offset_y = 2.5
        self.height = 2.5

        # Publishers and Subscribers
        self.cmd_pub = rospy.Publisher('/hydrone_aerial_underwater3/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('/hydrone_aerial_underwater3/odometry_sensor1/odometry', Odometry, self.odom_callback)
        rospy.Subscriber('/hydrone_aerial_underwater3/uncertainty_reset', Bool, self.update_uncertainty)
        self.unc_pub = rospy.Publisher('/hydrone_aerial_underwater3/uncertainty', Float64, queue_size=10)

        self.control_timer = rospy.Timer(rospy.Duration(0.05), self.controller_callback)

        # Robot state
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.theta = 0.0

        # Lissajous parameters
        self.A = 2.5 # x amplitude
        self.B = 1.0  # y amplitude
        self.m = 2    # frequency on x
        self.n = 3    # frequency on y
        self.num_points = 300
        self.threshold = 0.1

        # Generate waypoints
        self.path_points = []
        for i in range(self.num_points):
            s = 2 * math.pi * i / self.num_points
            x = self.offset_x + self.A * math.sin(self.m * s)
            y = self.offset_y + self.B * math.sin(self.n * s)
            self.path_points.append((x, y))

        self.current_waypoint_index = 0

        rospy.loginfo("Lissajous Follower (ROS1) Initialized.")

        self.uncertainty = np.random.uniform(0.5, 5)

    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.z = msg.pose.pose.position.z

        q = msg.pose.pose.orientation
        orientation_list = [q.x, q.y, q.z, q.w]
        _, _, self.theta = tf.transformations.euler_from_quaternion(orientation_list)

    def update_uncertainty(self, msg):
        self.uncertainty = 0.1

        rospy.loginfo("Uncertainty updated" + str(self.uncertainty))

    def controller_callback(self, event):
        # Update velocity scaling dynamically
        self.velocity_scaling = rospy.get_param('~velocity_scaling', 1.0)
        scaling = max(0.0, min(1.0, self.velocity_scaling))
        v = 0.22 * np.random.uniform(0.5, 1) * scaling

        # Get current target waypoint
        tx, ty = self.path_points[self.current_waypoint_index]
        distance = math.hypot(tx - self.x, ty - self.y)

        # Advance to next waypoint
        if distance < self.threshold:
            self.current_waypoint_index = (self.current_waypoint_index + 1) % self.num_points
            tx, ty = self.path_points[self.current_waypoint_index]

        # Compute heading and error
        angle_to_target = math.atan2(ty - self.y, tx - self.x)
        alpha = self.normalize_angle(angle_to_target - self.theta)

        omega = 2.0 * alpha  # proportional gain

        twist = Twist()
        twist.linear.x = v if v > 1e-3 else 0.0
        twist.angular.z = omega

        if self.z >= self.height + 0.05:
            twist.linear.z = -0.05
        elif self.z <= self.height - 0.05:
            twist.linear.z = 0.05
        else:
            twist.linear.z = 0.0

        self.cmd_pub.publish(twist)

        rospy.loginfo(f"[Lissajous] Index {self.current_waypoint_index} | x={self.x:.2f}, y={self.y:.2f} | alpha={alpha:.2f}")

        self.uncertainty = self.uncertainty + 0.01

        self.unc_pub.publish(self.uncertainty)

        rospy.loginfo("Uncertainty" + str(self.uncertainty))

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

if __name__ == '__main__':
    try:
        LissajousFollower()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
