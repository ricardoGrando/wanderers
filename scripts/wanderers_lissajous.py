#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import random
import math
import tf

class LissajousFollower:
    def __init__(self):
        rospy.init_node('lissajous_follower')

        # Parameters
        self.velocity_scaling = rospy.get_param('~velocity_scaling', 1.0)

        self.offset_x = 0.0
        self.offset_y = -2.5
        self.height = 2.5

        # Publishers and Subscribers
        self.cmd_pub = rospy.Publisher('/hydrone_aerial_underwater2/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('/hydrone_aerial_underwater2/odometry_sensor1/odometry', Odometry, self.odom_callback)

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

        rospy.loginfo("TurtleBot3 Lissajous Follower (ROS1) Initialized.")

    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.z = msg.pose.pose.position.z

        q = msg.pose.pose.orientation
        orientation_list = [q.x, q.y, q.z, q.w]
        _, _, self.theta = tf.transformations.euler_from_quaternion(orientation_list)

    def controller_callback(self, event):
        # Update velocity scaling dynamically
        self.velocity_scaling = rospy.get_param('~velocity_scaling', 1.0)
        scaling = max(0.0, min(1.0, self.velocity_scaling))
        v = 0.22 * scaling

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

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

class SmartRandomWander:
    def __init__(self):
        rospy.init_node('smart_random_wander_node')

        self.cmd_pub = rospy.Publisher('/hydrone_aerial_underwater2/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('/hydrone_aerial_underwater2/odometry_sensor1/odometry', Odometry, self.odom_callback)
        self.timer = rospy.Timer(rospy.Duration(0.1), self.update_motion)

        # Robot state
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.yaw = 0.0

        # Motion parameters
        self.linear_speed = 0.15
        self.angular_velocity = 0.0
        self.max_angular = 1.0
        self.angular_change_rate = 0.05

        # Boundaries
        self.limit = 5.0
        self.in_boundary_mode = False  # Whether we are correcting at boundary
        self.turning_to_center = False

        rospy.loginfo("SmartRandomWander node started.")

    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.z = msg.pose.pose.position.z
        q = msg.pose.pose.orientation
        _, _, self.yaw = tf.transformations.euler_from_quaternion(
            [q.x, q.y, q.z, q.w]
        )

    def update_motion(self, event):
        twist = Twist()

        # Distance from center and direction to center
        distance = math.hypot(self.x, self.y)
        center_angle = math.atan2(-self.y, -self.x)  # angle pointing to (0,0)
        angle_diff = self.normalize_angle(center_angle - self.yaw)

        # Check if outside boundary
        if abs(self.x) > self.limit or abs(self.y) > self.limit:
            self.turning_to_center = True
            self.in_boundary_mode = True
            rospy.logwarn("Outside limit. Turning to face center.")

        # If turning to face center
        if self.turning_to_center:
            # Check if we are now facing toward center
            if abs(angle_diff) > 0.2:
                # Still need to turn
                twist.angular.z = 1.0 * angle_diff
                twist.linear.x = 0.0
            else:
                # Done turning
                self.turning_to_center = False
                rospy.loginfo("Turn complete. Resuming forward motion.")
        else:
            self.in_boundary_mode = False

            # Normal wandering mode
            delta = random.uniform(-self.angular_change_rate, self.angular_change_rate)
            self.angular_velocity += delta
            self.angular_velocity = max(-self.max_angular, min(self.max_angular, self.angular_velocity))

            twist.linear.x = self.linear_speed
            twist.angular.z = self.angular_velocity

        if self.z >= 2.6:
            twist.linear.z = -0.05
        elif self.z <= 2.4:
            twist.linear.z = 0.05
        else:
            twist.linear.z = 0.0

        self.cmd_pub.publish(twist)

        rospy.loginfo("x=%.2f y=%.2f | yaw=%.2f | angle_diff=%.2f | mode=%s",
                      self.x, self.y, self.yaw, angle_diff,
                      "turning" if self.turning_to_center else "wandering")

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
