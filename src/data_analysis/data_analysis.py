import rosbag
import math
import numpy as np
# import matplotlib.pyplot as plt

def get_positions_and_velocities_from_bag(bag_file, odom_topic):
    positions = []
    velocities = []
    timestamps = []
    with rosbag.Bag(bag_file, 'r') as bag:
        for topic, msg, t in bag.read_messages(topics=[odom_topic]):
            if topic == odom_topic:
                x = msg.pose.pose.position.x
                y = msg.pose.pose.position.y
                linear_x = msg.twist.twist.linear.x
                positions.append((x, y))
                velocities.append(linear_x)
                timestamps.append(t.to_sec())
    return positions, velocities, timestamps

def calculate_incremental_path_length(positions, start_index, end_index):
    length = 0.0
    for i in range(start_index + 1, end_index + 1):
        dx = positions[i][0] - positions[i-1][0]
        dy = positions[i][1] - positions[i-1][1]
        length += math.sqrt(dx**2 + dy**2)
    return length

def is_near_start(x_current, y_current, x_start, y_start, epsilon):
    return math.sqrt((x_current - x_start) ** 2 + (y_current - y_start) ** 2) < epsilon

def main(bag_file, odom_topic, epsilon=0.3, min_distance=4.0, min_time=3.0):
    positions, velocities, timestamps = get_positions_and_velocities_from_bag(bag_file, odom_topic)

    if not positions:
        print("No position data found in the bag file.")
        return

    x_start, y_start = positions[0]
    start_time = None
    total_path_length = 0.0
    lap_times = []
    lap_distances = []
    laps_completed = 0

    for i in range(1, len(positions)):
        x_current, y_current = positions[i]
        linear_x = velocities[i]

        if start_time is None and linear_x != 0.0:
            start_time = timestamps[i]

        if start_time is not None:
            elapsed_time = timestamps[i] - start_time
            total_path_length += calculate_incremental_path_length(positions, i-1, i)

            if elapsed_time > min_time and total_path_length > min_distance and is_near_start(x_current, y_current, x_start, y_start, epsilon):
                laps_completed += 1
                lap_times.append(elapsed_time)
                lap_distances.append(total_path_length)
                total_path_length = 0.0  # Reset for next lap
                start_time = timestamps[i]  # Reset for next lap

    total_distance_traveled = sum(lap_distances)
    total_time = timestamps[-1] - timestamps[0] if start_time is not None else 0
    average_lap_length = np.mean(lap_distances) if lap_distances else 0
    average_lap_time = np.mean(lap_times) if lap_times else "N/A"
    average_speed = total_distance_traveled / total_time if total_time > 0 else 0

    print(f"Total laps completed: {laps_completed if laps_completed > 0 else '未完成一圈'}")
    print(f"Total distance traveled: {total_distance_traveled:.2f} meters")
    print(f"Average lap length: {average_lap_length:.2f} meters")
    print(f"Total time traveled: {total_time:.2f} seconds")
    print(f"Average lap time: {average_lap_time} seconds")
    print(f"Average speed: {average_speed:.2f} meters/second")

    # 可视化路径
    # positions_array = np.array(positions)
    # plt.figure(figsize=(12, 6))

    # # 绘制路径
    # plt.plot(positions_array[:, 0], positions_array[:, 1], marker='o', linestyle='-')
    # plt.title('Robot Path')
    # plt.xlabel('X Position')
    # plt.ylabel('Y Position')
    # plt.grid(True)

    # plt.tight_layout()
    # plt.show()

if __name__ == "__main__":
    # bag_file = "follow3.0 for levine.bag"
    # bag_file = "follow3.0 for berlin.bag"
    # bag_file = "follow3.0 for torino.bag"

    bag_file = "astar3.0 for levine.bag"
    # bag_file = "astar3.0 for berlin.bag"
    # bag_file = "astar3.0 for torino.bag"

    # bag_file = "pure3.0 for levine.bag"
    # bag_file = "pure3.0 for berlin.bag"
    # bag_file = "pure3.0 for torino.bag"

    # bag_file = "levine2.0optimise.bag"
    # bag_file = "berlin3testoptimise.bag"
    # bag_file = "torino2.0optimise.bag"
    odom_topic = "/odom"
    main(bag_file, odom_topic)

