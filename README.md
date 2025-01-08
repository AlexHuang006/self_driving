1.2 Scope and Objectives
The most critical steps in achieving autonomous driving in cars contain perception, planning and control. Specifically, the car senses the surrounding environment through various sensors on the car, then processes the collected information and generates the optimal path or the next driving direction, and finally the control module commands the car to follow the results of the planning part [5]. The algorithms that have been studied more are Follow the gap, pure pursuit and A* algorithm [6]. Each of these algorithms also has its own advantages and disad- vantages in different working environments that are complex and dynamically changing. For example, the A* algorithm, which is a heuristic search algorithm, which is centred on finding a shortest path. However, it still has some obvious defects, such as the generated paths are not smooth enough and the size of the racing car itself is not considered [7]. Follow the gap on the other hand is a locally planned geometric obstacle avoidance algorithm, but it sometimes leads to paths being unnecessarily lengthened [8]. In addition, pure pursuit is also a very popular and simple algorithm, but it also has some weaknesses because of the poor performance of the algorithm if the lookahead distance parameter is not properly selected [9].
However, in most cases, the default parameter configuration of a path planning algorithm may be a reasonable setting for the general case, however, the performance of the algorithm with this parameter configuration may not be optimal [10]. The algorithm itself has many different types of configuration parameters, and the large number of parameter combinations implies a large search space, and finding the optimal parameter configuration is obviously tedious and laborious [10]. But actually this problem is a typical optimization problem [11]. And genetic algorithm (GA) has become a popular method to solve complex combinatorial problems [12].
Therefore, the objectives of this study:
i. Evaluate and compare the above three algorithms using the F1TENTH simulator [6].
ii. Finding the best combination of parameters for the path planning algorithm through genetic algorithm to enhance the performance of the path planning algorithm.

based on ubuntu system
1. construct work space(catkin_ws)

2. install F1TENTH simulator from git link

3. Compile code using catkin_make

4. Configure the environment in setup.bash

5. run ROS, roslaunch f1tenth_simulator simulator.luanch

6. implement node, rosrun planning_pkg xxxx.py
