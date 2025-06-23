#!/usr/bin/env python3
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Quaternion
from tf.transformations import quaternion_from_euler
import sys

def send_goal(x, y, yaw_deg):
    rospy.init_node('simple_goal_sender')

    # Connect to move_base action server
    client = actionlib.SimpleActionClient('robot1/move_base', MoveBaseAction)
    rospy.loginfo("Waiting for move_base action server...")
    client.wait_for_server()
    rospy.loginfo("Connected to move_base!")

    # Create goal message
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"  # Use "map" for global nav
    goal.target_pose.header.stamp = rospy.Time.now()

    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y

    # Convert yaw (degrees) to quaternion
    yaw_rad = yaw_deg * 3.14159 / 180.0
    q = quaternion_from_euler(0, 0, yaw_rad)
    goal.target_pose.pose.orientation = Quaternion(*q)

    rospy.loginfo(f"Sending goal: x={x}, y={y}, yaw={yaw_deg}°")
    client.send_goal(goal)
    client.wait_for_result()

    result = client.get_result()
    rospy.loginfo("Goal reached!" if result else "Failed to reach goal.")

if __name__ == '__main__':
    if len(sys.argv) < 4:
        print("Usage: rosrun your_package send_goal.py x y yaw_deg")
    else:
        x = float(sys.argv[1])
        y = float(sys.argv[2])
        yaw = float(sys.argv[3])
        send_goal(x, y, yaw)
