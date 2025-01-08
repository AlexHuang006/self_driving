#!/usr/bin/env python3
# for the middle of best point
import rospy 
import numpy as np
from sensor_msgs.msg import LaserScan 
from ackermann_msgs.msg import AckermannDriveStamped 

class followthegap(object):
    # (1)subscribe lidar topic and publish vehicle control msgs
    def __init__(self):

        # initialize node of follow the gap
        rospy.init_node('followthegap')

        # subscribe lidar msgs and implement callback function
        self.subscriber = rospy.Subscriber('/scan', LaserScan, self.lidar_callback)

        # publish vehicle control msgs
        self.publisher = rospy.Publisher('/drive', AckermannDriveStamped, queue_size = 1)

    # (2) preprocess lidar data
    def preprocess_lidar(self, ranges):

        # transfer lidar data to array
        proc_ranges = np.array(ranges)

        # using smoothing window averega to 
        # deal with lidar data for avoiding outliers
        proc_ranges = np.convolve(proc_ranges, np.ones(3) / 3, mode = 'same')
        
        proc_ranges = np.where(proc_ranges > 3, 3, proc_ranges)
        
        return proc_ranges

    # (3) according to lidar data for finding the max gap
    def find_max_gap(self, free_space):
        # 3.1 initial variable
        max_gap_start = 0 # index of max gap start
        max_gap_end = 0   # index of max gap end
        max_gap_size = 0  # size of max gap

        current_start = 0 # index of current gap start 
        in_gap = False    # determine if the current point is in free space range

        # 3.2 iterate all points
        for i in range(len(free_space)):

            # if distance of the current point is greater than 1, 
            # and in_gap is equal to False, 
            # and finaly this point is the start point of the free space.
            if free_space[i] > 1:
                if in_gap == False:
                    in_gap = True
                    current_start = i
            else:
                if in_gap == True:
                    in_gap = False
                    current_size = i - current_start
                    if current_size > max_gap_size:
                        max_gap_size = current_size
                        max_gap_start = current_start
                        max_gap_end = i - 1
        
        # 3.3 in order to find the gap if there are no any obstacles
        if in_gap == True:
            current_size = len(free_space) - current_start
            if current_size > max_gap_size:
                max_gap_size = current_size
                max_gap_start = current_start
                max_gap_end = i - 1
        return max_gap_start, max_gap_end
    
    # (4) in order to find the middle point within the max gap and go there
    def find_best_point(self, start_i, end_i, ranges):
        best_point_index = (start_i + end_i) // 2 # calculate the middle point

        return best_point_index
    
    # (5) callback function
    def lidar_callback(self, data):
        # 5.1 deal with data
        # only need -90 to 90 angle in front of vehicle
        start_index = int((-np.pi/2 - data.angle_min) / data.angle_increment)
        end_index = int((np.pi/2 - data.angle_min) / data.angle_increment)
        # only need -90 to 90 angle's point
        ranges = data.ranges[start_index:end_index + 1]
        # call preprocess_lidar function
        proc_ranges = self.preprocess_lidar(ranges) 
        
        # 5.2 set bubble
        # Find closest point for setting bubble
        closest_index = np.argmin(proc_ranges) 
        # closest_distance = proc_ranges[closest_index] 
        
        # Eliminate all points inside 'bubble' (set them to zero) 
        bubble_radius = 0.5# Bubble radius in meters 
        bubble_size = int(bubble_radius / data.angle_increment) 
        
        # figure out all points which are in bubble
        min_index = max(0, closest_index - bubble_size) 
        max_index = min(len(proc_ranges) - 1, closest_index + bubble_size) 
        # set all points which are in bubble be 0
        for i in range(min_index, max_index + 1):
            proc_ranges[i] = 0 

        # 5.3 figure out gap   
        # call "find max gap" function for finding max gap 
        max_start, max_end = self.find_max_gap(proc_ranges) 
        
        # 5.4 Find the best point in the gap 
        best_point_index = self.find_best_point(max_start, max_end, proc_ranges) 
        
        # 5.5 Convert index to steering angle 
        angle_range = (np.pi/2) - (-np.pi/2)
        steering_angle = -np.pi/2 + (best_point_index / len(proc_ranges)) * angle_range
        
        # 5.6 according to the distance to decide the speed
        # high speed for far from the obstacle, slow down for closer to obstacle
        disance = proc_ranges[best_point_index]
        speed = min(3, max(1, disance / 0.5))

        # 5.7 Publish Drive message 
        drive_msg = AckermannDriveStamped() 
        drive_msg.header.stamp = rospy.Time.now() 
        drive_msg.header.frame_id = 'lidar' 
        drive_msg.drive.steering_angle = steering_angle 
        drive_msg.drive.speed = speed
        
        self.publisher.publish(drive_msg)

if __name__ == "__main__":
    try:
        follow_the_gap = followthegap()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass