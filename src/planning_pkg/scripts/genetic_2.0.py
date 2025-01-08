#!/usr/bin/env python3

import rospy
import threading
import time
import random
import math
from followthegap_forop import FollowTheGap
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Log
import reset_car_position  # 导入重置小车位置的模块

# 定义待优化的参数范围
PARAMETER_RANGES = {
    "smoothing_window_size": (1, 8),
    "max_distance": (1.0, 8.0),
    "speed_range": (1.0, 8.0),
    "safe_distance": (1.0, 4.0),
    "bubble_radius": (0.3, 1.0)
}

# 定义适应度函数
def fitness_function(time_taken, average_speed, distance_traveled, collision_occurred):
    """计算适应度值。如果发生碰撞，适应度值设为负无穷大"""
    if collision_occurred:
        return float('-inf')
    alpha = 0.6
    beta = 0.7
    gamma = 0.3

    fitness_time = 1.0 / (time_taken + 0.1)
    fitness_speed = average_speed
    fitness_distance = 1.0 / (distance_traveled + 0.1)

    fitness = alpha * fitness_time + beta * fitness_speed + gamma * fitness_distance
    return fitness

# 性能监控类
class PerformanceMonitor:
    def __init__(self):
        """初始化性能监控器，订阅 /odom 和 /rosout 话题"""
        self.start_time = None
        self.end_time = None
        self.collisions = 0
        self.collision_detected = False
        self.is_moving = False
        self.odom_subscriber = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.log_subscriber = rospy.Subscriber('/rosout', Log, self.log_callback)
        self.total_distance = 0.0
        self.total_speed = 0.0
        self.num_steps = 0
        self.starting_point = None
        self.last_position = None
        self.lap_count = 0
        self.lap_threshold = 0.2  # 设置合适的阈值
        self.lap_cooldown = 5.0  # 圈数检测的冷却时间
        self.last_lap_time = None

    def odom_callback(self, msg):
        """处理 /odom 话题中的消息，记录车辆开始和停止的时间"""
        speed = msg.twist.twist.linear.x
        current_position = (msg.pose.pose.position.x, msg.pose.pose.position.y)

        if speed > 0 and not self.is_moving:
            self.start_time = rospy.Time.now()
            self.is_moving = True
            self.starting_point = current_position
            self.last_lap_time = rospy.Time.now()

        if speed > 0:
            self.total_distance += speed * 0.1  # 0.1秒的时间步长
            self.total_speed += speed
            self.num_steps += 1

            if self.starting_point is not None and self.is_moving:
                time_since_last_lap = (rospy.Time.now() - self.last_lap_time).to_sec()
                distance_from_start = math.sqrt(
                    (current_position[0] - self.starting_point[0])**2 +
                    (current_position[1] - self.starting_point[1])**2
                )
                if distance_from_start < self.lap_threshold and time_since_last_lap > self.lap_cooldown:
                    self.lap_count += 1
                    self.last_lap_time = rospy.Time.now()
                    # rospy.loginfo(f"Lap {self.lap_count} completed.")

        if speed == 0 and self.is_moving:
            self.end_time = rospy.Time.now()
            self.is_moving = False

    def log_callback(self, msg):
        """处理 /rosout 话题中的日志消息，检测碰撞"""
        if "Collision detected" in msg.msg and not self.collision_detected:
            self.collisions += 1
            self.collision_detected = True
            rospy.loginfo("Collision detected based on log message!")

    def get_performance_data(self):
        """返回行驶时间和碰撞次数"""
        time_taken = (self.end_time - self.start_time).to_sec() if self.start_time and self.end_time else float('inf')
        average_speed = self.total_speed / self.num_steps if self.num_steps > 0 else 0
        return time_taken, average_speed, self.total_distance, self.collisions

# 生成个体
def generate_individual():
    """生成一个个体，个体包含平滑窗口大小、最大距离和速度范围"""
    individual = {}
    individual["smoothing_window_size"] = random.randint(*PARAMETER_RANGES["smoothing_window_size"])
    individual["safe_distance"] = random.uniform(*PARAMETER_RANGES["safe_distance"])
    individual["bubble_radius"] = random.uniform(*PARAMETER_RANGES["bubble_radius"])
    individual["max_distance"] = random.uniform(*PARAMETER_RANGES["max_distance"])
    individual["speed_range"] = (random.uniform(*PARAMETER_RANGES["speed_range"]), random.uniform(*PARAMETER_RANGES["speed_range"]))
    return individual

# 生成种群
def generate_population(size):
    """生成初始种群"""
    population = []
    for _ in range(size):
        population.append(generate_individual())
    return population

def evaluate_individual(individual):
    """评估一个个体的适应度值"""
    reset_car_position.publish_initial_pose()

    # 1. 拿到个体的参数
    smoothing_window_size = individual["smoothing_window_size"]
    max_distance = individual["max_distance"]
    safe_distance = individual["safe_distance"]
    bubble_radius = individual["bubble_radius"]
    min_speed, max_speed = individual["speed_range"]

    # 2. 将个体参数给到路径规划算法，启动算法
    rospy.loginfo("2. Starting FollowTheGap for evaluate fitness")
    ftg = FollowTheGap(smoothing_window_size, max_distance, safe_distance, bubble_radius, (min_speed, max_speed))
    ftg_thread = threading.Thread(target=ftg.run)
    ftg_thread.start()

    # 3. 收集性能数据
    monitor = PerformanceMonitor()

    # 4. 当小车完成3圈或检测到碰撞后，立即计算适应度值
    while monitor.lap_count < 3 and not monitor.collision_detected:
        time.sleep(0.1)

    time_taken, average_speed, distance_traveled, collisions = monitor.get_performance_data()
    fitness = fitness_function(time_taken, average_speed, distance_traveled, collisions > 0)

    # 5. 立即停止路径规划算法
    rospy.loginfo("3. Stopping FollowTheGap")
    ftg.stop()
    ftg_thread.join(1)
    if ftg_thread.is_alive():
        rospy.logwarn("FollowTheGap thread did not terminate in time.")

    # 6. 重置小车位置
    rospy.loginfo("4. Resetting car position...")
    reset_car_position.publish_initial_pose()
    rospy.loginfo("5. Waiting for car position to reset...")
    # rospy.sleep(2)  # 等待足够的时间以确保位置重置成功
    rospy.loginfo("6. Car position reset complete.")

    # 7. 返回适应度值
    return fitness


# select parents
def select_parents(population, fitnesses):
    """choose the highest fitness individual as the parents"""
    sorted_population = sorted(zip(population, fitnesses), key=lambda x: x[1], reverse=True)
    parents = [sorted_population[0][0], sorted_population[1][0]]
    return parents

# crossover
def crossover(parent1, parent2):
    """random choose from the parents, generate two childs"""
    child1 = {
        "smoothing_window_size": random.choice([parent1["smoothing_window_size"], parent2["smoothing_window_size"]]),
        "max_distance": random.choice([parent1["max_distance"], parent2["max_distance"]]),
        "safe_distance": random.choice([parent1["safe_distance"], parent2["safe_distance"]]),
        "bubble_radius": random.choice([parent1["bubble_radius"], parent2["bubble_radius"]]),
        "speed_range": (
            random.choice([parent1["speed_range"][0], parent2["speed_range"][0]]),
            random.choice([parent1["speed_range"][1], parent2["speed_range"][1]])
        )
    }
    child2 = {
        "smoothing_window_size": random.choice([parent1["smoothing_window_size"], parent2["smoothing_window_size"]]),
        "max_distance": random.choice([parent1["max_distance"], parent2["max_distance"]]),
        "safe_distance": random.choice([parent1["safe_distance"], parent2["safe_distance"]]),
        "bubble_radius": random.choice([parent1["bubble_radius"], parent2["bubble_radius"]]),
        "speed_range": (
            random.choice([parent1["speed_range"][0], parent2["speed_range"][0]]),
            random.choice([parent1["speed_range"][1], parent2["speed_range"][1]])
        )
    }
    return child1, child2

# mutation
def mutate(individual):
    mutation_prob = 0.1  # mutation rate
    if random.random() < mutation_prob:
        individual["smoothing_window_size"] = random.randint(*PARAMETER_RANGES["smoothing_window_size"])
    if random.random() < mutation_prob:
        individual["max_distance"] = random.uniform(*PARAMETER_RANGES["max_distance"])
    if random.random() < mutation_prob:
        individual["safe_distance"] = random.uniform(*PARAMETER_RANGES["safe_distance"])
    if random.random() < mutation_prob:
        individual["bubble_radius"] = random.uniform(*PARAMETER_RANGES["bubble_radius"])
    if random.random() < mutation_prob:
        individual["speed_range"] = (random.uniform(*PARAMETER_RANGES["speed_range"]), random.uniform(*PARAMETER_RANGES["speed_range"]))

# iterate genetic algorithm
def genetic_algorithm(population_size, generations):
    population = generate_population(population_size)
    for generation in range(generations):
        # rospy.loginfo(f"0. generation {generation + 1}")
        fitnesses = []
        for idx, individual in enumerate(population):
            # rospy.loginfo(f"1. Evaluating individual {idx + 1}")
            fitness = evaluate_individual(individual)
            fitnesses.append(fitness)

        parents = select_parents(population, fitnesses)
        next_population = []
        while len(next_population) < population_size:
            child1, child2 = crossover(parents[0], parents[1])
            fitness_child1 = evaluate_individual(child1)
            fitness_child2 = evaluate_individual(child2)
            fitness_parents1 = evaluate_individual(parents[0])
            fitness_parents2 = evaluate_individual(parents[1])
            if fitness_child1 > fitness_parents1:
                next_population.extend([child1])
            else:
                mutate(child1)
                next_population.extend([child1])
            if fitness_child2 > fitness_parents2:
                next_population.extend([child2])
            else:
                mutate(child2)
                next_population.extend([child2])
        population = next_population[:population_size]

    best_individual = min(population, key=lambda x: evaluate_individual(x))
    return best_individual

if __name__ == '__main__':
    rospy.init_node('reset_simulation_node', anonymous=True)
    best_individual = genetic_algorithm(population_size=30, generations=20)
    print("Best individual is:", best_individual)