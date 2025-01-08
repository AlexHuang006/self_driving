#!/usr/bin/env python3

import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDrive, AckermannDriveStamped
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped, Pose
import heapq

KP=1.000
KD=0.005
prev_error=0
FILTER_VALUE = 10.0
DESIRED_DISTANCE_RIGHT = 1.0

class AStarPlanner:
    def __init__(self, map_data):
        self.map_data = map_data
        self.map_width = map_data.info.width
        self.map_height = map_data.info.height
        self.map_resolution = map_data.info.resolution
        self.map_origin = map_data.info.origin

    def is_valid(self, x, y):
        if x < 0 or y < 0 or x >= self.map_width or y >= self.map_height:
            return False
        return self.map_data.data[y * self.map_width + x] == 0

    def heuristic(self, a, b):
        return abs(a[0] - b[0]) + abs(a[1] - b[1])

    def astar(self, start, goal):
        open_list = []
        heapq.heappush(open_list, (0, start))
        came_from = {}
        g_score = {start: 0}
        f_score = {start: self.heuristic(start, goal)}

        while open_list:
            _, current = heapq.heappop(open_list)

            if current == goal:
                path = []
                while current in came_from:
                    path.append(current)
                    current = came_from[current]
                path.reverse()
                return path

            for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
                neighbor = (current[0] + dx, current[1] + dy)
                tentative_g_score = g_score[current] + 1

                if self.is_valid(neighbor[0], neighbor[1]) and (neighbor not in g_score or tentative_g_score < g_score[neighbor]):
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = tentative_g_score + self.heuristic(neighbor, goal)
                    heapq.heappush(open_list, (f_score[neighbor], neighbor))

        return []

def get_range(data, angle, deg=True):
    if deg:
        angle = np.deg2rad(angle)
    dis = data.ranges[int((angle - data.angle_min) / data.angle_increment)]
    if dis < data.range_min or dis > data.range_max:
        dis = FILTER_VALUE
    return dis

def wall_following_callback(data):
    THETA = np.pi / 180 * 45
    a = get_range(data, 45)
    b = get_range(data, 45 + np.rad2deg(THETA))
    alpha = np.arctan((a * np.cos(THETA) - b) / (a * np.sin(THETA)))
    AB = b * np.cos(alpha)

    projected_dis = AB + 1.00 * np.sin(alpha)
    error = DESIRED_DISTANCE_RIGHT - projected_dis
    #print("projected_dis=",projected_dis)
    #print("Error=",error)

    tmoment = rospy.Time.now().to_sec()
    prev_tmoment = 0.0
    del_time = tmoment - prev_tmoment
    steering_angle = -(KP*error + KD*(error - prev_error)/del_time)
    prev_tmoment = tmoment

    if np.abs(steering_angle) > np.pi / 180 * 20.0:
        speed = 1.5
    elif np.abs(steering_angle) > np.pi / 180 * 10.0:
        speed = 2.5
    else:
        speed = 3.5

    drive_msg = AckermannDriveStamped()
    drive_msg.drive.steering_angle = steering_angle
    drive_msg.drive.speed = speed
    drive_pub.publish(drive_msg)

def publish_path(path, path_pub, map_origin, map_resolution):
    path_msg = Path()
    path_msg.header.frame_id = "map"
    path_msg.header.stamp = rospy.Time.now()

    for point in path:
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = rospy.Time.now()
        pose.pose.position.x = point[0] * map_resolution + map_origin.position.x
        pose.pose.position.y = point[1] * map_resolution + map_origin.position.y
        pose.pose.position.z = 0
        path_msg.poses.append(pose)

    path_pub.publish(path_msg)

class PurePursuit:
    def __init__(self, lookahead_distance):
        self.lookahead_distance = lookahead_distance
        self.current_path = []

    def update_path(self, path):
        self.current_path = path

    def compute_steering_angle(self, current_pose):
        if not self.current_path:
            return 0.0

        min_distance = float('inf')
        lookahead_point = None

        for point in self.current_path:
            distance = np.sqrt((point[0] - current_pose[0])**2 + (point[1] - current_pose[1])**2)
            if distance >= self.lookahead_distance and distance < min_distance:
                min_distance = distance
                lookahead_point = point

        if lookahead_point is None:
            lookahead_point = self.current_path[-1]

        angle_to_goal = np.arctan2(lookahead_point[1] - current_pose[1], lookahead_point[0] - current_pose[0])
        return angle_to_goal

# if __name__ == '__main__': 
#     try:
#         rospy.init_node("path_planning")
        
#         map_data = rospy.wait_for_message('/map', OccupancyGrid)
#         astar_planner = AStarPlanner(map_data)

#         start = (10, 10)
#         goal = (50, 50)
#         path = astar_planner.astar(start, goal)

#         # pure_pursuit = PurePursuit(lookahead_distance=1.0)
#         # pure_pursuit.update_path(path)

#         scan_sub = rospy.Subscriber('/scan', LaserScan, wall_following_callback)
#         drive_pub = rospy.Publisher('/drive', AckermannDriveStamped, queue_size=1)
        
#         rospy.spin()
#     except rospy.ROSInterruptException:
#         pass

if __name__ == '__main__':
    try:
        rospy.init_node("path_planning")
       
        map_data = rospy.wait_for_message('/map', OccupancyGrid)
        astar_planner = AStarPlanner(map_data)

        start = (10, 10)
        goal = (50, 50)
        path = astar_planner.astar(start, goal)

        path_pub = rospy.Publisher('/planned_path', Path, queue_size=1)
        if path:
            publish_path(path, path_pub, astar_planner.map_origin, astar_planner.map_resolution)

        scan_sub = rospy.Subscriber('/scan', LaserScan, wall_following_callback)
        drive_pub = rospy.Publisher('/drive', AckermannDriveStamped, queue_size=1)
       
        rospy.spin()
    except rospy.ROSInterruptException:
        pass