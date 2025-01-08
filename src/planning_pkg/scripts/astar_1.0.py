#!/usr/bin/env python3

import rospy
import numpy as np
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from geometry_msgs.msg import PoseStamped
import heapq

class AStarPlanner:
    def __init__(self, map_data):
        self.map_data = map_data
        self.map_width = map_data.info.width
        self.map_height = map_data.info.height
        self.map_resolution = map_data.info.resolution
        self.map_origin = map_data.info.origin

    def is_valid(self, x, y):
        if x < 0 or y < 0 or x >= self.map_width or y >= self.map_height:
            rospy.loginfo("Point (%d, %d) is out of map bounds", x, y)
            return False
        if self.map_data.data[y * self.map_width + x] != 0:
            rospy.loginfo("Point (%d, %d) is an obstacle", x, y)
            return False
        return True

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

def publish_path(path, path_pub, map_origin, map_resolution):
    path_msg = Path()
    path_msg.header.frame_id = "map"
    path_msg.header.stamp = rospy.Time.now()

    for point in path:
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = rospy.Time.now()
        # Correct the coordinate transformation here
        pose.pose.position.x = point[0] * map_resolution + map_origin.position.x
        pose.pose.position.y = point[1] * map_resolution + map_origin.position.y
        pose.pose.position.z = 0
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = 0.0
        pose.pose.orientation.w = 1.0
        rospy.loginfo("Path point: (%f, %f)", pose.pose.position.x, pose.pose.position.y)
        path_msg.poses.append(pose)

    rospy.loginfo("Publishing Path with %d points", len(path_msg.poses))
    path_pub.publish(path_msg)

def odom_callback(msg):
    global start, goal
    current_position = (msg.pose.pose.position.x, msg.pose.pose.position.y)
    if start is None:
        start = current_position
        goal = (current_position[0] + 1.0, current_position[1] + 1.0)  # 设置一个稍微偏离的终点
        rospy.loginfo("Initial position set: %s", current_position)

if __name__ == '__main__':
    try:
        rospy.init_node("path_planning")

        start = None
        goal = None

        # 初始化路径发布器和路径消息
        path_pub = rospy.Publisher('/planned_path', Path, queue_size=1)

        # 订阅里程计数据
        rospy.Subscriber('/odom', Odometry, odom_callback)

        # 订阅地图数据并等待消息
        rospy.loginfo("Waiting for map data...")
        map_data = rospy.wait_for_message('/map', OccupancyGrid)
        astar_planner = AStarPlanner(map_data)
        rospy.loginfo("Map data received")

        # 等待获取初始位置
        while start is None or goal is None:
            rospy.loginfo("Waiting for initial position...")
            rospy.sleep(0.1)

        # 执行路径规划
        start_grid = (int((start[0] - astar_planner.map_origin.position.x) / astar_planner.map_resolution),
                      int((start[1] - astar_planner.map_origin.position.y) / astar_planner.map_resolution))
        goal_grid = (int((goal[0] - astar_planner.map_origin.position.x) / astar_planner.map_resolution),
                     int((goal[1] - astar_planner.map_origin.position.y) / astar_planner.map_resolution))

        rospy.loginfo("Planning path from %s to %s", start_grid, goal_grid)

        if not astar_planner.is_valid(start_grid[0], start_grid[1]):
            rospy.logwarn("Start position is invalid!")
        if not astar_planner.is_valid(goal_grid[0], goal_grid[1]):
            rospy.logwarn("Goal position is invalid!")

        path = astar_planner.astar(start_grid, goal_grid)

        if path:
            rospy.loginfo("Path generated: %s", path)  # 打印生成的路径
            publish_path(path, path_pub, astar_planner.map_origin, astar_planner.map_resolution)
        else:
            rospy.logwarn("No path found")

        rospy.spin()
    except rospy.ROSInterruptException:
        pass