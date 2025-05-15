#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Float64
import random
import math
import tf
import numpy as np
import csv
import os
from datetime import datetime

class CSVLogger:
    def __init__(self, directory="/home/ricardo/hydrone_ws"):
        # Create directory if it doesn't exist
        if not os.path.exists(directory):
            os.makedirs(directory)

        # Generate filename with start time
        start_time_str = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        self.filename = os.path.join(directory, f"log_{start_time_str}.csv")

        # Open the file and prepare the CSV writer
        self.file = open(self.filename, mode='w', newline='')
        self.writer = csv.writer(self.file)

    def log(self, value1, value2, value3):
        now_str = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        self.writer.writerow([value1, value2, value3])
        self.file.flush()  # Ensure data is written to disk

    def close(self):
        self.file.close()

class UncertaintyDealer:
    def __init__(self):
        rospy.init_node('uncertainty_dealer')

        # Publishers and Subscribers
        self.pose_pub = rospy.Publisher('/uncertainty_dealer/target_pose', Pose, queue_size=10)
        self.unc_1_pub = rospy.Publisher('/hydrone_aerial_underwater1/uncertainty_reset', Bool, queue_size=10)
        self.unc_2_pub = rospy.Publisher('/hydrone_aerial_underwater2/uncertainty_reset', Bool, queue_size=10)
        self.unc_3_pub = rospy.Publisher('/hydrone_aerial_underwater3/uncertainty_reset', Bool, queue_size=10)

        rospy.Subscriber('/uncertainty_dealer/uncertainty_set', Bool, self.uncertainty_set)
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

        self.flag = -1

        self.first_pose = Pose()
        self.second_pose = Pose()
        self.third_pose = Pose()

        self.last_update_time = rospy.Time(0)  # initialize to 0

        self.logger = CSVLogger()

        rospy.loginfo("Uncertainty Dealer Initialized.")

    def uncertainty_set(self, msg):
        current_time = rospy.Time.now()
        if (current_time - self.last_update_time).to_sec() >= 5.0:
            if self.flag == 1:
                self.unc_1_pub.publish(True)
            if self.flag == 2:
                self.unc_2_pub.publish(True)
            if self.flag == 3:
                self.unc_3_pub.publish(True)
            self.last_update_time = current_time
        else:
            rospy.loginfo("Update skipped: waiting for 5-second interval")

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
                self.flag = 1
                rospy.loginfo("Target pose first")

        if self.uncertainty_second > self.uncertainty_first:
            if self.uncertainty_second > self.uncertainty_third:
                self.pose_pub.publish(self.second_pose)
                self.flag = 2
                rospy.loginfo("Target pose second")

        if self.uncertainty_third > self.uncertainty_first:
            if self.uncertainty_third > self.uncertainty_second:
                self.pose_pub.publish(self.third_pose)
                self.flag = 3
                rospy.loginfo("Target pose third")

        self.logger.log(self.uncertainty_first, self.uncertainty_second, self.uncertainty_third)

if __name__ == '__main__':
    try:
        UncertaintyDealer()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
