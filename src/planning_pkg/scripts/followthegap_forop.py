#!/usr/bin/env python3

import rospy, sys
import numpy as np
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped

class FollowTheGap:
    def __init__(self, smoothing_window_size=3, max_distance=2.6, safe_distance = 1, bubble_radius=0.5, speed_range=(2.0, 5.5)):
        self.smoothing_window_size = int(smoothing_window_size)
        self.max_distance = max_distance
        self.bubble_radius = bubble_radius
        self.min_speed, self.max_speed = speed_range
        self.safe_distance = safe_distance
        self.active = True

        self.subscriber = rospy.Subscriber('/scan', LaserScan, self.lidar_callback)
        self.publisher = rospy.Publisher('/drive', AckermannDriveStamped, queue_size=1)

    def preprocess_lidar(self, ranges):
        if len(ranges) == 0:
            rospy.logwarn("Received empty lidar ranges array")
            return np.array(ranges)

        if self.smoothing_window_size < 1:
            rospy.logwarn("Invalid smoothing_window_size: {}".format(self.smoothing_window_size))
            self.smoothing_window_size = 1

        proc_ranges = np.array(ranges)
        proc_ranges = np.convolve(proc_ranges, np.ones(self.smoothing_window_size) / self.smoothing_window_size, mode='same')
        proc_ranges = np.where(proc_ranges > self.max_distance, self.max_distance, proc_ranges)
        return proc_ranges

    def find_max_gap(self, free_space):
        max_gap_start = 0
        max_gap_end = 0
        max_gap_size = 0
        current_start = 0
        in_gap = False

        for i in range(len(free_space)):
            if free_space[i] > self.safe_distance:
                if not in_gap:
                    in_gap = True
                    current_start = i
            else:
                if in_gap:
                    in_gap = False
                    current_size = i - current_start
                    if current_size > max_gap_size:
                        max_gap_size = current_size
                        max_gap_start = current_start
                        max_gap_end = i - 1

        if in_gap:
            current_size = len(free_space) - current_start
            if current_size > max_gap_size:
                max_gap_size = current_size
                max_gap_start = current_start
                max_gap_end = i - 1
        return max_gap_start, max_gap_end

    def find_best_point(self, start_i, end_i, ranges):
        best_point_index = (start_i + end_i) // 2

        return best_point_index

    def lidar_callback(self, data):
        if not self.active:
            return

        start_index = int((-np.pi / 2 - data.angle_min) / data.angle_increment)
        end_index = int((np.pi / 2 - data.angle_min) / data.angle_increment)
        ranges = data.ranges[start_index:end_index + 1]

        if len(ranges) == 0:
            rospy.logwarn("Received empty lidar ranges array in callback")
            return

        proc_ranges = self.preprocess_lidar(ranges)

        closest_index = np.argmin(proc_ranges)
        bubble_size = int(self.bubble_radius / data.angle_increment)

        min_index = max(0, closest_index - bubble_size)
        max_index = min(len(proc_ranges) - 1, closest_index + bubble_size)
        for i in range(min_index, max_index + 1):
            proc_ranges[i] = 0

        max_start, max_end = self.find_max_gap(proc_ranges)
        best_point_index = self.find_best_point(max_start, max_end, proc_ranges)

        angle_range = (np.pi / 2) - (-np.pi / 2)
        steering_angle = -np.pi / 2 + (best_point_index / len(proc_ranges)) * angle_range

        distance = proc_ranges[best_point_index]
        speed = min(self.max_speed, max(self.min_speed, distance / 0.5))

        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = rospy.Time.now()
        drive_msg.header.frame_id = 'lidar'
        drive_msg.drive.steering_angle = steering_angle
        drive_msg.drive.speed = speed

        if not rospy.is_shutdown() and self.active:
            self.publisher.publish(drive_msg)

    def stop(self):
        self.active = False

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    if len(sys.argv) == 5:
        smoothing_window_size = int(sys.argv[1])
        max_distance = float(sys.argv[2])
        bubble_radius = float(sys.argv[3])
        min_speed, max_speed = map(float, sys.argv[4].split(','))
    else:
        smoothing_window_size = 3
        max_distance = 2.6
        bubble_radius = 0.5
        min_speed, max_speed = 2.0, 3.5

    ftg = FollowTheGap(smoothing_window_size, max_distance, bubble_radius, (min_speed, max_speed))
    ftg.run()


