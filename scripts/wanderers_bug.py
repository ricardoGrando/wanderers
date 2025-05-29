#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist, Pose, Point, Quaternion
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Float64, Float32
import random
import math
import tf
import numpy as np
from sensor_msgs.msg import LaserScan

# epsilon for testing whether a number is close to zero
_EPS = np.finfo(float).eps * 4.0

# axis sequences for Euler angles
_NEXT_AXIS = [1, 2, 0, 1]

# map axes strings to/from tuples of inner axis, parity, repetition, frame
_AXES2TUPLE = {
    'sxyz': (0, 0, 0, 0), 'sxyx': (0, 0, 1, 0), 'sxzy': (0, 1, 0, 0),
    'sxzx': (0, 1, 1, 0), 'syzx': (1, 0, 0, 0), 'syzy': (1, 0, 1, 0),
    'syxz': (1, 1, 0, 0), 'syxy': (1, 1, 1, 0), 'szxy': (2, 0, 0, 0),
    'szxz': (2, 0, 1, 0), 'szyx': (2, 1, 0, 0), 'szyz': (2, 1, 1, 0),
    'rzyx': (0, 0, 0, 1), 'rxyx': (0, 0, 1, 1), 'ryzx': (0, 1, 0, 1),
    'rxzx': (0, 1, 1, 1), 'rxzy': (1, 0, 0, 1), 'ryzy': (1, 0, 1, 1),
    'rzxy': (1, 1, 0, 1), 'ryxy': (1, 1, 1, 1), 'ryxz': (2, 0, 0, 1),
    'rzxz': (2, 0, 1, 1), 'rxyz': (2, 1, 0, 1), 'rzyz': (2, 1, 1, 1)}

_TUPLE2AXES = dict((v, k) for k, v in _AXES2TUPLE.items())

def euler_from_matrix(matrix, axes='sxyz'):
    """Return Euler angles from rotation matrix for specified axis sequence.

    axes : One of 24 axis sequences as string or encoded tuple

    Note that many Euler angle triplets can describe one matrix.

    >>> R0 = euler_matrix(1, 2, 3, 'syxz')
    >>> al, be, ga = euler_from_matrix(R0, 'syxz')
    >>> R1 = euler_matrix(al, be, ga, 'syxz')
    >>> numpy.allclose(R0, R1)
    True
    >>> angles = (4.0*math.pi) * (numpy.random.random(3) - 0.5)
    >>> for axes in _AXES2TUPLE.keys():
    ...    R0 = euler_matrix(axes=axes, *angles)
    ...    R1 = euler_matrix(axes=axes, *euler_from_matrix(R0, axes))
    ...    if not numpy.allclose(R0, R1): print axes, "failed"

    """
    try:
        firstaxis, parity, repetition, frame = _AXES2TUPLE[axes.lower()]
    except (AttributeError, KeyError):
        _ = _TUPLE2AXES[axes]
        firstaxis, parity, repetition, frame = axes

    i = firstaxis
    j = _NEXT_AXIS[i+parity]
    k = _NEXT_AXIS[i-parity+1]

    M = np.array(matrix, dtype=np.float64, copy=False)[:3, :3]
    if repetition:
        sy = math.sqrt(M[i, j]*M[i, j] + M[i, k]*M[i, k])
        if sy > _EPS:
            ax = math.atan2( M[i, j],  M[i, k])
            ay = math.atan2( sy,       M[i, i])
            az = math.atan2( M[j, i], -M[k, i])
        else:
            ax = math.atan2(-M[j, k],  M[j, j])
            ay = math.atan2( sy,       M[i, i])
            az = 0.0
    else:
        cy = math.sqrt(M[i, i]*M[i, i] + M[j, i]*M[j, i])
        if cy > _EPS:
            ax = math.atan2( M[k, j],  M[k, k])
            ay = math.atan2(-M[k, i],  cy)
            az = math.atan2( M[j, i],  M[i, i])
        else:
            ax = math.atan2(-M[j, k],  M[j, j])
            ay = math.atan2(-M[k, i],  cy)
            az = 0.0

    if parity:
        ax, ay, az = -ax, -ay, -az
    if frame:
        ax, az = az, ax
    return ax, ay, az

def euler_from_quaternion(quaternion, axes='sxyz'):
    """Return Euler angles from quaternion for specified axis sequence.

    >>> angles = euler_from_quaternion([0.06146124, 0, 0, 0.99810947])
    >>> np.allclose(angles, [0.123, 0, 0])
    True

    """
    return euler_from_matrix(quaternion_matrix(quaternion), axes)


def quaternion_matrix(quaternion):
    """Return homogeneous rotation matrix from quaternion.

    >>> R = quaternion_matrix([0.06146124, 0, 0, 0.99810947])
    >>> np.allclose(R, rotation_matrix(0.123, (1, 0, 0)))
    True

    """
    q = np.array(quaternion[:4], dtype=np.float64, copy=True)
    nq = np.dot(q, q)
    if nq < _EPS:
        return np.identity(4)
    q *= math.sqrt(2.0 / nq)
    q = np.outer(q, q)
    return np.array((
        (1.0-q[1, 1]-q[2, 2],     q[0, 1]-q[2, 3],     q[0, 2]+q[1, 3], 0.0),
        (    q[0, 1]+q[2, 3], 1.0-q[0, 0]-q[2, 2],     q[1, 2]-q[0, 3], 0.0),
        (    q[0, 2]-q[1, 3],     q[1, 2]+q[0, 3], 1.0-q[0, 0]-q[1, 1], 0.0),
        (                0.0,                 0.0,                 0.0, 1.0)
        ), dtype=np.float64)

class BUG2:
    def __init__(self):
        rospy.init_node('BUG2')

        # Parameters
        self.velocity_scaling = rospy.get_param('~velocity_scaling', 1.0)

        self.offset_x = 2.5
        self.offset_y = 2.5
        self.height = 3.0

        # Publishers and Subscribers
        self.cmd_pub = rospy.Publisher('/hydrone_aerial_underwater0/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('/hydrone_aerial_underwater0/odometry_sensor1/odometry', Odometry, self.odom0_callback)
        self.unc_reset = rospy.Publisher('/uncertainty_dealer/uncertainty_set', Bool, queue_size=10)
        rospy.Subscriber('/uncertainty_dealer/target_pose', Pose, self.target_pose_callback)

        rospy.Subscriber('/hydrone_aerial_underwater0/scan', LaserScan, self.laser_callback)

        # Timer for control loop
        self.control_timer = rospy.Timer(rospy.Duration(0.05), self.controller_callback)

        # Robot state
        self.position = Point()
        self.orientation = Quaternion()
        self.target_pos = Point()
        self.theta = 0.0   
        self.target_x = 0
        self.target_y = 0
        self.target_z = 0
        self.heading = [0,0]  

        self.underwater = False  

        self.lidar_ranges = []

        self.state = "GO_TO_GOAL"
        self.start_pos = Point(self.position.x, self.position.y, self.position.z)
        self.hit_point = None

        rospy.loginfo("BUG (ROS1) initialized.")

    def odom0_callback(self, msg):
        self.position = msg.pose.pose.position 
        self.orientation = msg.pose.pose.orientation 
   
    def target_pose_callback(self, msg):        
        self.target_pos = msg.position
        target_orientation = msg.orientation
        orientation_list = [self.orientation.x, self.orientation.y, self.orientation.z, self.orientation.w]
        roll, pitch, yaw = euler_from_quaternion(orientation_list)
        yaw_angle = math.atan2(self.target_pos.y - self.position.y, self.target_pos.x - self.position.x)
        pitch_angle = math.atan2(self.target_pos.z - self.position.z, self.target_pos.x - self.position.x)

        heading = np.array([0., 0.])
        heading[0] = yaw_angle - yaw
        heading[1] = pitch_angle - pitch
        for i in range(2):
            if heading[i] > math.pi:
                heading[i] -= 2 * math.pi

            elif heading[i] < -math.pi:
                heading[i] += 2 * math.pi

        self.heading = heading

    def laser_callback(self, msg):
        self.lidar_ranges = msg.ranges
        # rospy.loginfo(len(self.lidar_ranges))

    def is_on_mline(self, tolerance=0.1):
        # Vector from start to goal
        dx_goal = self.target_pos.x - self.start_pos.x
        dy_goal = self.target_pos.y - self.start_pos.y

        # Vector from current to start
        dx_curr = self.position.x - self.start_pos.x
        dy_curr = self.position.y - self.start_pos.y

        # Cross product should be ~0 if on line
        cross = dx_goal * dy_curr - dy_goal * dx_curr
        return abs(cross) < tolerance

    def is_closer_to_goal_than_hit(self):
        curr_dist = math.hypot(self.target_pos.x - self.position.x, self.target_pos.y - self.position.y)
        hit_dist = math.hypot(self.target_pos.x - self.hit_point[0], self.target_pos.y - self.hit_point[1])
        return curr_dist < hit_dist

    def controller_callback(self, event):
        # Publish command
        twist = Twist()

        # print(self.position)
        # print(self.target_pos)

        # # Heading control
        # yaw_error = self.heading[0]  # Difference in yaw (target - current)

        # if self.position.z < self.height:
        #     twist.linear.z = 0.25
        # else: 
        #     twist.linear.z = -0.25

        # if self.underwater:
        #     twist.linear.z = -0.105
        # else:
        #     twist.linear.z = 0

        # if abs(self.heading[0]) < 0.15:
        #     twist.linear.x = 0.25
        #     twist.angular.z = 0.0
        # else:            
        #     twist.angular.z = 0.25 if yaw_error > 0 else -0.25
                 

        distance = math.sqrt((self.target_pos.x - self.position.x)**2 + (self.target_pos.y - self.position.y)**2)

        front_range = 10  # center range for ~5°
        min_front_distance = 0
        yaw_error = self.heading[0]

        try:
            # Check if path is blocked in front (e.g., within ±10°)
            front_range = self.lidar_ranges[360:720]  # center range for ~5°
            min_front_distance = min(front_range)
        
            obstacle_threshold = 0.8  # meters

            if self.state == "GO_TO_GOAL":
                if min_front_distance < obstacle_threshold:
                    # Hit an obstacle, save hit point
                    self.state = "FOLLOW_OBSTACLE"
                    self.hit_point = (self.position.x, self.position.y)
                    rospy.loginfo("Switched to FOLLOW_OBSTACLE")
                else:
                    # Move toward goal
                    if abs(yaw_error) < 0.15:
                        twist.linear.x = 0.3
                        twist.angular.z = 0.0
                    else:
                        twist.linear.x = 0.0
                        twist.angular.z = 0.3 if yaw_error > 0 else -0.3

            elif self.state == "FOLLOW_OBSTACLE":
                # Basic left wall-following: check left side (270° to 180°)
                left_range = self.lidar_ranges[800:880]
                min_left_distance = min(left_range)

                if min_front_distance < obstacle_threshold:
                    # Still obstacle ahead: turn right
                    twist.linear.x = 0.0
                    twist.angular.z = -0.4
                elif min_left_distance > obstacle_threshold:
                    # Free on the left: turn slightly left and move
                    twist.linear.x = 0.1
                    twist.angular.z = 0.3
                else:
                    # Follow wall
                    twist.linear.x = 0.2
                    twist.angular.z = 0.0

                # Check if back on M-line and closer to goal than hit point
                if self.is_on_mline() and self.is_closer_to_goal_than_hit():
                    self.state = "GO_TO_GOAL"
                    rospy.loginfo("Back to GO_TO_GOAL")

            self.cmd_pub.publish(twist)

            if distance < 0.85:
                self.unc_reset.publish(True)
                rospy.loginfo("Uncerttain")

            rospy.loginfo("Distance" + str(distance) + ", " + str(self.heading[0]))
        except:
            rospy.loginfo("No lidar samples")

if __name__ == '__main__':
    try:
        BUG2()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
