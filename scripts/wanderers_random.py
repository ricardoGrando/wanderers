#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import random
import math
import tf

class SmartRandomWander:
    def __init__(self):
        rospy.init_node('smart_random_wander_node')

        self.cmd_pub = rospy.Publisher('/hydrone_aerial_underwater3/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('/hydrone_aerial_underwater3/odometry_sensor1/odometry', Odometry, self.odom_callback)
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

        if self.z >= 2.8:
            twist.linear.z = -0.05
        elif self.z <= 2.7:
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
        SmartRandomWander()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
