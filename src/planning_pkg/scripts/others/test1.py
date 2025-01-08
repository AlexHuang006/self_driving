#!/usr/bin/env python3
import math
import rospy
from nav_msgs.msg import OccupancyGrid
import matplotlib.pyplot as plt

show_animation = True


class AStarPlanner:

    def __init__(self, ox, oy, resolution, rr):
        """
        Initialize grid map for a star planning

        ox: x position list of Obstacles [m]
        oy: y position list of Obstacles [m]
        resolution: grid resolution [m],地图的像素
        rr: robot radius[m]
        """

        self.resolution = resolution
        self.rr = rr
        self.min_x, self.min_y = 0, 0
        self.max_x, self.max_y = 0, 0
        self.obstacle_map = None
        self.x_width, self.y_width = 0, 0
        self.motion = self.get_motion_model()
        self.calc_obstacle_map(ox, oy)

    class Node:
        """定义搜索区域节点类,每个Node都包含坐标x和y, 移动代价cost和父节点索引。
        """
        def __init__(self, x, y, cost, parent_index):
            self.x = x  # index of grid
            self.y = y  # index of grid
            self.cost = cost
            self.parent_index = parent_index

        # def __str__(self):
        #     return str(self.x) + "," + str(self.y) + "," + str(
        #         self.cost) + "," + str(self.parent_index)

    def planning(self, sx, sy, gx, gy):
        """
        A star path search
        输入起始点和目标点的坐标(sx,sy)和(gx,gy)，
        最终输出的结果是路径包含的点的坐标集合rx和ry。
        input:
            s_x: start x position [m]
            s_y: start y position [m]
            gx: goal x position [m]
            gy: goal y position [m]

        output:
            rx: x position list of the final path
            ry: y position list of the final path
        """

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

            # show graph
            if show_animation:  # pragma: no cover
                plt.plot(self.calc_grid_position(current.x, self.min_x),
                         self.calc_grid_position(current.y, self.min_y), "xc")
                # for stopping simulation with the esc key.
                plt.gcf().canvas.mpl_connect('key_release_event', lambda event: [exit(0) if event.key == 'escape' else None])
                if len(closed_set.keys()) % 10 == 0:
                    plt.pause(0.001)

            # 通过追踪当前位置current.x和current.y来动态展示路径寻找
            if current.x == goal_node.x and current.y == goal_node.y:
                print("Find goal")
                goal_node.parent_index = current.parent_index
                goal_node.cost = current.cost
                break

            # Remove the item from the open set
            del open_set[c_id]

            # Add it to the closed set
            closed_set[c_id] = current

            # expand_grid search grid based on motion model
            for i, _ in enumerate(self.motion):
                node = self.Node(current.x + self.motion[i][0],
                                 current.y + self.motion[i][1],
                                 current.cost + self.motion[i][2], c_id)
                n_id = self.calc_grid_index(node)

                # If the node is not safe, do nothing
                if not self.verify_node(node):
                    continue

                if n_id in closed_set:
                    continue

                if n_id not in open_set:
                    open_set[n_id] = node  # discovered a new node
                else:
                    if open_set[n_id].cost > node.cost:
                        # This path is the best until now. record it
                        open_set[n_id] = node

        rx, ry = self.calc_final_path(goal_node, closed_set)

        return rx, ry

    def calc_final_path(self, goal_node, closed_set):
        # generate final course
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
        """计算启发函数

        Args:
            n1 (_type_): _description_
            n2 (_type_): _description_

        Returns:
            _type_: _description_
        """
        w = 1.0  # weight of heuristic
        d = w * math.hypot(n1.x - n2.x, n1.y - n2.y)
        return d

    def calc_grid_position(self, index, min_position):
        """
        calc grid position

        :param index:
        :param min_position:
        :return:
        """
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

        # collision check
        if self.obstacle_map[node.x][node.y]:
            return False

        return True

    def calc_obstacle_map(self, ox, oy):
       
        # 确定地图的边界，x和y轴上的最小和最大值
        self.min_x = round(min(ox))
        self.min_y = round(min(oy))
        self.max_x = round(max(ox))
        self.max_y = round(max(oy))
        print("min_x:", self.min_x)
        print("min_y:", self.min_y)
        print("max_x:", self.max_x)
        print("max_y:", self.max_y)

        # 计算网格地图的宽度和高度
        self.x_width = round((self.max_x - self.min_x) / self.resolution)
        self.y_width = round((self.max_y - self.min_y) / self.resolution)
        print("x_width:", self.x_width)
        print("y_width:", self.y_width)

        # obstacle map generation
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
        # dx, dy, cost
        motion = [[1, 0, 1],
                  [0, 1, 1],
                  [-1, 0, 1],
                  [0, -1, 1],
                  [-1, -1, math.sqrt(2)],
                  [-1, 1, math.sqrt(2)],
                  [1, -1, math.sqrt(2)],
                  [1, 1, math.sqrt(2)]]

        return motion


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
            if msg.data[index] > 50:  # 占据率大于50表示有障碍物
                ox.append(x * resolution + origin_x)
                oy.append(y * resolution + origin_y)


def main():
    print(__file__ + " start!!")

    rospy.init_node('a_star_planner')
    rospy.Subscriber('/map', OccupancyGrid, map_callback)

    # 等待地图消息
    rospy.wait_for_message('/map', OccupancyGrid)

    # start and goal position
    sx = 0  # [m]
    sy = 0  # [m]
    wx1 = 9.5
    wy1 = 8.5
    wx2 = -13.5
    wy2 = 7
    gx = 0  # [m]
    gy = 0  # [m]
    robot_radius = 0.5  # [m]

    if show_animation:  # pragma: no cover
        plt.plot(ox, oy, ".k")
        plt.plot(sx, sy, "og")
        plt.plot(wx1, wy1, ".k")
        plt.plot(wx2, wy2, ".k")
        plt.plot(gx, gy, "xb")
        plt.grid(True)
        plt.axis("equal")

    a_star = AStarPlanner(ox, oy, resolution, robot_radius)
    # rx, ry = a_star.planning(sx, sy, gx, gy)
    rx1, ry1 = a_star.planning(sx, sy, wx1, wy1)
    rx2, ry2 = a_star.planning(wx1, wy1, wx2, wy2)
    rx3, ry3 = a_star.planning(wx2, wy2, gx, gy)

    # rx = rx1 + rx2[1:]
    # ry = ry1 + ry2[1:]
    # print(rx, ry)

    if show_animation:  # pragma: no cover
        # plt.plot(rx, ry, "-r")
        plt.plot(rx1, ry1, "-r")
        plt.plot(rx2, ry2, "-r")
        plt.plot(rx3, ry3, "-r")
        plt.pause(0.001)
        plt.show()


if __name__ == '__main__':
    main()
    
# import rospy
# from nav_msgs.msg import OccupancyGrid
# from geometry_msgs.msg import PoseStamped, Point
# from ackermann_msgs.msg import AckermannDriveStamped
# from std_msgs.msg import Header
# import math
# import heapq

# class AStarPlanner:
#     def __init__(self, ox, oy, resolution, vehicle_radius):
#         self.ox = ox
#         self.oy = oy
#         self.resolution = resolution
#         self.vehicle_radius = vehicle_radius
#         self.motion = [[1, 0, 1], [-1, 0, 1], [0, 1, 1], [0, -1, 1]]
#         self.motion.extend([[-1, -1, math.sqrt(2)], [-1, 1, math.sqrt(2)], [1, -1, math.sqrt(2)], [1, 1, math.sqrt(2)]])
#         self.motion_cost = [c for _, _, c in self.motion]

#     def planning(self, start, goal):
#         start_node = Node(*self.to_grid(start), 0, None)
#         goal_node = Node(*self.to_grid(goal), 0, None)
#         open_set = {start_node}
#         came_from = {start_node: None}
#         g_cost = {start_node: 0}
#         f_cost = {start_node: self.heuristic(start_node, goal_node)}

#         while open_set:
#             current_node = self.get_lowest_f_cost(open_set)
#             if current_node == goal_node:
#                 return self.reconstruct_path(came_from, current_node)

#             open_set.remove(current_node)
#             for i, (motion_dx, motion_dy, motion_cost) in enumerate(self.motion):
#                 next_node = Node(
#                     current_node.x + motion_dx,
#                     current_node.y + motion_dy,
#                     current_node.g_cost + motion_cost,
#                     current_node
#                 )

#                 if not self.is_node_valid(next_node):
#                     continue

#                 tentative_g_cost = current_node.g_cost + motion_cost
#                 if next_node not in g_cost or tentative_g_cost < g_cost[next_node]:
#                     next_node.h_cost = self.heuristic(next_node, goal_node)
#                     next_node.g_cost = tentative_g_cost
#                     next_node.f_cost = next_node.g_cost + next_node.h_cost
#                     came_from[next_node] = current_node
#                     if next_node not in open_set:
#                         open_set.add(next_node)

#         return None

#     def get_lowest_f_cost(self, nodes):
#         return min(nodes, key=lambda n: n.f_cost)

#     def to_grid(self, position):
#         x, y = position
#         return round((x - self.ox) / self.resolution), round((y - self.oy) / self.resolution)

#     def to_world(self, grid_position):
#         x, y = grid_position
#         return x * self.resolution + self.ox, y * self.resolution + self.oy

#     def is_node_valid(self, node):
#         x, y = self.to_world(node.x, node.y)
#         if not (0 <= node.x < self.width and 0 <= node.y < self.height):
#             return False
#         if self.is_collision(x, y):
#             return False
#         return True

#     def is_collision(self, x, y):
#         return math.hypot(x - self.ox, y - self.oy) <= self.vehicle_radius

#     def heuristic(self, a, b):
#         return math.sqrt((a.x - b.x) ** 2 + (a.y - b.y) ** 2)

#     def reconstruct_path(self, came_from, current):
#         path = [self.to_world(current.x, current.y)]
#         while current in came_from:
#             current = came_from[current]
#             path.append(self.to_world(current.x, current.y))
#         return path[::-1]

# class Node:
#     def __init__(self, x, y, g_cost, parent):
#         self.x = x
#         self.y = y
#         self.g_cost = g_cost
#         self.h_cost = 0
#         self.f_cost = 0
#         self.parent = parent

#     def __lt__(self, other):
#         return self.f_cost < other.f_cost

# def main():
#     rospy.init_node('a_star_planner')
#     # Assume ox and oy are the center of the map for simplicity
#     ox, oy = 0, 0
#     resolution = 0.1  # 10 cm resolution
#     vehicle_radius = 0.25  # Half of the vehicle width
#     a_star = AStarPlanner(ox, oy, resolution, vehicle_radius)

#     # Assume we subscribe to the map and get the map size
#     width = 100  # 100 grid cells wide
#     height = 100  # 100 grid cells high

#     a_star.width = width
#     a_star.height = height

#     # Start and goal positions in world coordinates
#     start = (0, 0)
#     goal = (95 * resolution, 95 * resolution)  # 5 meters from the start

#     path = a_star.planning(start, goal)
#     if path:
#         for point in path:
#             pub = rospy.Publisher('/planned_path', PoseStamped, queue_size=1)
#             pose = PoseStamped()
#             pose.header.stamp = rospy.Time.now()
#             pose.header.frame_id = 'map'
#             pose.pose.position = Point(x=point[0], y=point[1], z=0)
#             pub.publish(pose)

# class PIDController:
#     def __init__(self, K_P, K_I, K_D, target_speed, max_angle):
#         self.K_P = K_P
#         self.K_I = K_I
#         self.K_D = K_D
#         self.target_speed = target_speed
#         self.max_angle = max_angle
#         self.integral = 0.0
#         self.previous_error = 0.0

#     def update(self, current_pose, target_pose):
#         error = math.atan2(target_pose.y - current_pose.y, target_pose.x - current_pose.x)
#         derivative = error - self.previous_error
#         self.integral += error
#         steering_angle = (self.K_P * error + self.K_I * self.integral + self.K_D * derivative) * self.target_speed

#         # Limit the steering angle between -max_angle and max_angle
#         steering_angle = min(max(steering_angle, -self.max_angle), self.max_angle)

#         self.previous_error = error
#         return self.target_speed, steering_angle

# class AutonomousVehicle:
#     def __init__(self):
#         rospy.init_node('astar_navigation', anonymous=True)
#         self.current_pose = PoseStamped()
#         self.current_pose.header.frame_id = 'map'
#         self.K_P = 1.0
#         self.K_I = 0.1
#         self.K_D = 0.0
#         self.target_speed = 1.0  # m/s
#         self.max_steering_angle = math.radians(30)  # Maximum steering angle in radians
#         self.pid_controller = PIDController(self.K_P, self.K_I, self.K_D, self.target_speed, self.max_steering_angle)
#         self.a_star_planner = None
#         self.map_received = False
#         self.vehicle_pose_recerived = False
#         self.map_sub = rospy.Subscriber('/map', OccupancyGrid, self.map_callback)
#         self.vehicle_pose_sub = rospy.SubscribeListener('/initialpose', PoseStamped, self.vehicle_pose_callback)
#         self.path_pub = rospy.Publisher('/planned_path', AckermannDriveStamped, queue_size=10)

#     def map_callback(self, msg):
#         rospy.loginfo("Map received")
#         self.map_received = True
#         self.a_star_planner = AStarPlanner(msg.data, msg.info.width, msg.info.height, msg.info.resolution)
#         self.ox, self.oy = msg.info.origin.position.x, msg.info.origin.position.y
#         self.width, self.height = msg.info.width, msg.info.height
#         self.vehicle_radius = 0.25  # Vehicle radius is half of the vehicle width

#     def plan_and_control(self):
#         if not self.map_received:
#             rospy.loginfo("Waiting for map")
#             return

#         start = (0, 0)  # Starting at the bottom-left corner of the map
#         goal = (self.width * self.a_star_planner.resolution,
#                  self.height * self.a_star_planner.resolution)  # Goal at the top-right corner

#         path = self.a_star_planner.plan(start, goal)
#         if not path:
#             rospy.logwarn("No path found")
#             return

#         # Start following the path
#         for point in path:
#             pose = PoseStamped()
#             pose.header.stamp = rospy.Time.now()
#             pose.header.frame_id = 'map'
#             pose.pose.position = Point(x=point[0], y=point[1], z=0)
#             drive_stamped = AckermannDriveStamped()
#             drive_stamped.header.stamp = rospy.Time.now()
#             drive_stamped.header.frame_id = 'map'
#             speed, steering_angle = self.pid_controller.update(self.current_pose, pose)
#             drive_stamped.drive.speed = speed
#             drive_stamped.drive.steering_angle = steering_angle
#             self.path_pub.publish(drive_stamped)
#             self.current_pose = pose  # Update current pose

#     def run(self):
#         rate = rospy.Rate(10)  # 10 Hz
#         while not rospy.is_shutdown():
#             if self.map_received:
#                 self.plan_and_control()
#             rate.sleep()

# if __name__ == '__main__':
#     try:
#         av = AutonomousVehicle()
#         av.run()
#     except rospy.ROSInterruptException:
#         pass