#!/usr/bin/env python3

# import rospy
# from geometry_msgs.msg import PoseWithCovarianceStamped

# def publish_initial_pose():
#     # 初始化ROS节点
#     rospy.init_node('initial_pose_publisher')

#     # 创建一个发布者，发布到/initialpose话题
#     pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=1)

#     # 等待直到发布者准备好
#     rospy.sleep(1)

#     # 创建一个PoseWithCovarianceStamped消息
#     msg = PoseWithCovarianceStamped()

#     # 设置时间戳和坐标系
#     msg.header.stamp = rospy.Time.now()
#     msg.header.frame_id = "map"

#     # 设置位置
#     msg.pose.pose.position.x = 0.0  # x坐标
#     msg.pose.pose.position.y = 0.0  # y坐标
#     msg.pose.pose.position.z = 0.0  # z坐标

#     # 设置姿态
#     msg.pose.pose.orientation.x = 0.0  # 绕x轴旋转
#     msg.pose.pose.orientation.y = 0.0  # 绕y轴旋转
#     msg.pose.pose.orientation.z = 0.0  # 绕z轴旋转
#     msg.pose.pose.orientation.w = 1.0  # 旋转的余弦值

#     # 发布消息
#     pub.publish(msg)

#     # 保持节点运行，直到被关闭
#     rospy.spin()

# if __name__ == '__main__':
#     try:
#         publish_initial_pose()
#     except rospy.ROSInterruptException:
#         pass

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped

def publish_initial_pose():
    # 创建一个发布者，发布到/initialpose话题
    pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=1)

    # 等待直到发布者准备好
    rospy.sleep(1)

    # 创建一个PoseWithCovarianceStamped消息
    msg = PoseWithCovarianceStamped()

    # 设置时间戳和坐标系
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = "map"

    # 设置位置
    msg.pose.pose.position.x = 0.0  # x坐标
    msg.pose.pose.position.y = 0.0  # y坐标
    msg.pose.pose.position.z = 0.0  # z坐标

    # 设置姿态
    msg.pose.pose.orientation.x = 0.0  # 绕x轴旋转
    msg.pose.pose.orientation.y = 0.0  # 绕y轴旋转
    msg.pose.pose.orientation.z = 0.0  # 绕z轴旋转
    msg.pose.pose.orientation.w = 1.0  # 旋转的余弦值

    # 发布消息
    pub.publish(msg)
    # rospy.loginfo("Initial pose published.")

if __name__ == '__main__':
    rospy.init_node('initial_pose_publisher')
    publish_initial_pose()
    rospy.spin()