#!/usr/bin/env python3
import math
import rospy
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from ackermann_msgs.msg import AckermannDriveStamped
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import PoseStamped
import numpy as np

PI = 3.1415927

class AStarPlanner:

    def __init__(self, ox, oy, resolution, rr):
        self.resolution = resolution
        self.rr = rr
        self.min_x, self.min_y = 0, 0
        self.max_x, self.max_y = 0, 0
        self.obstacle_map = None
        self.x_width, self.y_width = 0, 0
        self.motion = self.get_motion_model()
        self.calc_obstacle_map(ox, oy)

    class Node:
        def __init__(self, x, y, cost, parent_index):
            self.x = x
            self.y = y
            self.cost = cost
            self.parent_index = parent_index

        def __str__(self):
            return str(self.x) + "," + str(self.y) + "," + str(
                self.cost) + "," + str(self.parent_index)

    def planning(self, sx, sy, gx, gy):
        start_node = self.Node(self.calc_xy_index(sx, self.min_x),
                               self.calc_xy_index(sy, self.min_y), 0.0, -1)
        goal_node = self.Node(self.calc_xy_index(gx, self.min_x),
                              self.calc_xy_index(gy, self.min_y), 0.0, -1)

        open_set, closed_set = dict(), dict()
        open_set[self.calc_grid_index(start_node)] = start_node

        while 1:
            if len(open_set) == 0:
                print("Open set is empty..")
                break

            c_id = min(
                open_set,
                key=lambda o: open_set[o].cost + self.calc_heuristic(goal_node, open_set[o]))
            current = open_set[c_id]

            if current.x == goal_node.x and current.y == goal_node.y:
                print("Find goal")
                goal_node.parent_index = current.parent_index
                goal_node.cost = current.cost
                break

            del open_set[c_id]
            closed_set[c_id] = current

            for i, _ in enumerate(self.motion):
                node = self.Node(current.x + self.motion[i][0],
                                 current.y + self.motion[i][1],
                                 current.cost + self.motion[i][2], c_id)
                n_id = self.calc_grid_index(node)

                if not self.verify_node(node):
                    continue

                if n_id in closed_set:
                    continue

                if n_id not in open_set:
                    open_set[n_id] = node
                else:
                    if open_set[n_id].cost > node.cost:
                        open_set[n_id] = node

        rx, ry = self.calc_final_path(goal_node, closed_set)

        return rx, ry

    def calc_final_path(self, goal_node, closed_set):
        rx, ry = [self.calc_grid_position(goal_node.x, self.min_x)], [
            self.calc_grid_position(goal_node.y, self.min_y)]
        parent_index = goal_node.parent_index
        while parent_index != -1:
            n = closed_set[parent_index]
            rx.append(self.calc_grid_position(n.x, self.min_x))
            ry.append(self.calc_grid_position(n.y, self.min_y))
            parent_index = n.parent_index

        return rx, ry

    @staticmethod
    def calc_heuristic(n1, n2):
        w = 1.0
        d = w * math.hypot(n1.x - n2.x, n1.y - n2.y)
        return d

    def calc_grid_position(self, index, min_position):
        pos = index * self.resolution + min_position
        return pos

    def calc_xy_index(self, position, min_pos):
        return round((position - min_pos) / self.resolution)

    def calc_grid_index(self, node):
        return (node.y - self.min_y) * self.x_width + (node.x - self.min_x)

    def verify_node(self, node):
        px = self.calc_grid_position(node.x, self.min_x)
        py = self.calc_grid_position(node.y, self.min_y)

        if px < self.min_x:
            return False
        elif py < self.min_y:
            return False
        elif px >= self.max_x:
            return False
        elif py >= self.max_y:
            return False

        if self.obstacle_map[node.x][node.y]:
            return False

        return True

    def calc_obstacle_map(self, ox, oy):
        self.min_x = round(min(ox))
        self.min_y = round(min(oy))
        self.max_x = round(max(ox))
        self.max_y = round(max(oy))
        print("min_x:", self.min_x)
        print("min_y:", self.min_y)
        print("max_x:", self.max_x)
        print("max_y:", self.max_y)

        self.x_width = round((self.max_x - self.min_x) / self.resolution)
        self.y_width = round((self.max_y - self.min_y) / self.resolution)
        print("x_width:", self.x_width)
        print("y_width:", self.y_width)

        self.obstacle_map = [[False for _ in range(self.y_width)]
                             for _ in range(self.x_width)]
        for ix in range(self.x_width):
            x = self.calc_grid_position(ix, self.min_x)
            for iy in range(self.y_width):
                y = self.calc_grid_position(iy, self.min_y)
                for iox, ioy in zip(ox, oy):
                    d = math.hypot(iox - x, ioy - y)
                    if d <= self.rr:
                        self.obstacle_map[ix][iy] = True
                        break

    @staticmethod
    def get_motion_model():
        motion = [[1, 0, 1],
                  [0, 1, 1],
                  [-1, 0, 1],
                  [0, -1, 1],
                  [-1, -1, math.sqrt(2)],
                  [-1, 1, math.sqrt(2)],
                  [1, -1, math.sqrt(2)],
                  [1, 1, math.sqrt(2)]]
        return motion

class PurePursuitController:
    def __init__(self):
        self.lookahead_distance = 1.5  # 预瞄距离，可以根据需要调整
        self.max_steering_angle = np.pi / 4  # 最大转向角
        self.max_steering_angle_rate = np.pi / 18  # 最大转向角变化速率，防止剧烈摆动
        self.vel_pub = rospy.Publisher('/drive', AckermannDriveStamped, queue_size=1)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)

        self.current_pose = None
        self.path = []

    def odom_callback(self, msg):
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        _, _, yaw = euler_from_quaternion(orientation_list)
        self.current_pose = (msg.pose.pose.position.x, msg.pose.pose.position.y, yaw)

    def set_path(self, path):
        self.path = path

    def get_distance(self, point1, point2):
        return np.hypot(point2[0] - point1[0], point2[1] - point1[1])

    def find_target_point(self):
        if self.current_pose is None or not self.path:
            return None

        x, y, _ = self.current_pose
        min_dist = float('inf')
        target_point = None

        for point in self.path:
            dist = self.get_distance((x, y), point)
            if dist < min_dist and dist > self.lookahead_distance:
                min_dist = dist
                target_point = point

        return target_point

    def compute_control(self):
        if self.current_pose is None or not self.path:
            return

        target_point = self.find_target_point()
        if target_point is None:
            return

        x, y, theta = self.current_pose
        target_x, target_y = target_point

        # 计算目标点相对于车辆坐标系的坐标
        dx = target_x - x
        dy = target_y - y
        local_x = np.cos(theta) * dx + np.sin(theta) * dy
        local_y = -np.sin(theta) * dx + np.cos(theta) * dy

        # 计算转向角
        if local_x == 0:
            steering_angle = 0
        else:
            steering_angle = np.arctan2(2 * self.lookahead_distance * local_y, local_x)

        # 限制转向角变化速率
        current_steering_angle = 0  # 假设初始转向角为0，可以根据实际情况获取当前转向角
        steering_angle_change = np.clip(steering_angle - current_steering_angle,
                                        -self.max_steering_angle_rate,
                                        self.max_steering_angle_rate)
        steering_angle = current_steering_angle + steering_angle_change

        # 限制转向角
        steering_angle = max(min(steering_angle, self.max_steering_angle), -self.max_steering_angle)

        # 打印调试信息
        rospy.loginfo(f"Current Pose: x={x}, y={y}, theta={theta}")
        rospy.loginfo(f"Target Point: x={target_x}, y={target_y}")
        rospy.loginfo(f"Steering Angle: {steering_angle}")

        # 创建 AckermannDriveStamped 消息
        ackermann_cmd = AckermannDriveStamped()
        ackermann_cmd.drive.speed = 1.5  # 固定速度
        ackermann_cmd.drive.steering_angle = steering_angle

        rospy.loginfo(f"Control Output (Speed): {ackermann_cmd.drive.speed}")
        rospy.loginfo(f"Control Output (Steering Angle): {ackermann_cmd.drive.steering_angle}")
        print()

        self.vel_pub.publish(ackermann_cmd)

def map_callback(msg):
    global ox, oy, resolution
    resolution = msg.info.resolution
    width = msg.info.width
    height = msg.info.height
    origin_x = msg.info.origin.position.x
    origin_y = msg.info.origin.position.y

    ox, oy = [], []
    for y in range(height):
        for x in range(width):
            index = x + y * width
            if msg.data[index] > 50:
                ox.append(x * resolution + origin_x)
                oy.append(y * resolution + origin_y)

def publish_path(rx, ry, path_pub):
    path = Path()
    path.header.frame_id = "map"
    path.header.stamp = rospy.Time.now()

    for x, y in zip(rx, ry):
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = rospy.Time.now()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0
        pose.pose.orientation.w = 1.0
        path.poses.append(pose)

    path_pub.publish(path)

def main():
    print(__file__ + " start!!")

    rospy.init_node('a_star_planner')
    rospy.Subscriber('/map', OccupancyGrid, map_callback)

    rospy.wait_for_message('/map', OccupancyGrid)

    path_pub = rospy.Publisher('/planned_path', Path, queue_size=1)

    # for levine_blocked map
    sx = 0
    sy = 0
    wx1 = 7.5
    wy1 = 8.5
    wx2 = -13.5
    wy2 = 7
    gx = 0
    gy = 0

    robot_radius = 0.5

    planning_resolution = 0.5
    a_star = AStarPlanner(ox, oy, planning_resolution, robot_radius)
    rx1, ry1 = a_star.planning(sx, sy, wx1, wy1)
    rx2, ry2 = a_star.planning(wx1, wy1, wx2, wy2)
    rx3, ry3 = a_star.planning(wx2, wy2, gx, gy)

    rx = rx3 + rx2[1:] + rx1[1:]
    ry = ry3 + ry2[1:] + ry1[1:]
   
    rx = rx[::-1]
    ry = ry[::-1]

    print("rx:", rx)
    print("ry:", ry)

    publish_path(rx, ry, path_pub)

    controller = PurePursuitController()
    path = list(zip(rx, ry))
    controller.set_path(path)

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        controller.compute_control()
        rate.sleep()

if __name__ == '__main__':
    main()

