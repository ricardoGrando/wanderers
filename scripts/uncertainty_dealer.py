#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Float64
import random
import math
import tf
import numpy as np

class UncertaintyDealer:
    def __init__(self):
        rospy.init_node('uncertainty_dealer')

        # Publishers and Subscribers
        self.pose_pub = rospy.Publisher('/uncertainty_dealer/target_pose', Pose, queue_size=10)
        rospy.Subscriber('/hydrone_aerial_underwater3/uncertainty', Float64, self.third_callback)
        rospy.Subscriber('/hydrone_aerial_underwater2/uncertainty', Float64, self.second_callback)
        rospy.Subscriber('/hydrone_aerial_underwater1/uncertainty', Float64, self.first_callback)

        rospy.Subscriber('/hydrone_aerial_underwater3/odometry_sensor1/pose', Pose, self.third_pose_callback)
        rospy.Subscriber('/hydrone_aerial_underwater2/odometry_sensor1/pose', Pose, self.second_pose_callback)
        rospy.Subscriber('/hydrone_aerial_underwater1/odometry_sensor1/pose', Pose, self.first_pose_callback)

        self.control_timer = rospy.Timer(rospy.Duration(0.05), self.controller_callback)

        self.uncertainty_first = 0
        self.uncertainty_second = 0
        self.uncertainty_third = 0

        self.first_pose = Pose()
        self.second_pose = Pose()
        self.first_pose = Pose()

        rospy.loginfo("Uncertainty Dealer Initialized.")

    def first_callback(self, msg):
        self.uncertainty_first = msg.data

    def second_callback(self, msg):
        self.uncertainty_second = msg.data

    def third_callback(self, msg):
        self.uncertainty_third = msg.data

    def first_pose_callback(self, msg):
        self.first_pose = msg

    def second_pose_callback(self, msg):
        self.second_pose = msg

    def third_pose_callback(self, msg):
        self.third_pose = msg

    def controller_callback(self, event):

        if self.uncertainty_first > self.uncertainty_second:
            if self.uncertainty_first > self.uncertainty_third:
                self.pose_pub.publish(self.first_pose)
                rospy.loginfo("Target pose first")

        if self.uncertainty_second > self.uncertainty_first:
            if self.uncertainty_second > self.uncertainty_third:
                self.pose_pub.publish(self.second_pose)
                rospy.loginfo("Target pose second")

        if self.uncertainty_third > self.uncertainty_first:
            if self.uncertainty_third > self.uncertainty_second:
                self.pose_pub.publish(self.second_pose)
                rospy.loginfo("Target pose third")

if __name__ == '__main__':
    try:
        UncertaintyDealer()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
