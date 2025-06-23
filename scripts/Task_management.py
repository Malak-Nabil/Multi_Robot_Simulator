#!/usr/bin/env python3

import rospy
import json
import math
from std_msgs.msg import String
from geometry_msgs.msg import PoseWithCovarianceStamped
from grad.msg import RobotState
from grad.msg import TaskCmd

class TaskAssigner:
    def __init__(self, json_file):
        rospy.init_node("task_assigner")

        # Load shelf locations
        with open(json_file, 'r') as f:
            self.shelf_poses = json.load(f)

        self.robot_states = {"robot1": "UNKNOWN", "robot2": "UNKNOWN"}
        self.robot_positions = {"robot1": None, "robot2": None}
        self.task_queue = []
        self.pending_pickup = None
        self.pending_dropoff = None

        # Publishers for task assignment
        self.task_publishers = {
            "robot1": rospy.Publisher("/robot1/task_cmd", TaskCmd, queue_size=10, latch=True),
            "robot2": rospy.Publisher("/robot2/task_cmd", TaskCmd, queue_size=10, latch=True)
        }
        self.assigned_task_pub = rospy.Publisher("/assigned_task", TaskCmd, queue_size=10, latch=True)

        # Subscribers
        rospy.Subscriber("/robot1/state", RobotState, self.state_callback, callback_args="robot1")
        rospy.Subscriber("/robot2/state", RobotState, self.state_callback, callback_args="robot2")
        rospy.Subscriber("/robot1/amcl_pose", PoseWithCovarianceStamped, self.pose_callback, callback_args="robot1")
        rospy.Subscriber("/robot2/amcl_pose", PoseWithCovarianceStamped, self.pose_callback, callback_args="robot2")
        rospy.Subscriber("/task_queue", TaskCmd, self.task_queue_callback)
        rospy.Subscriber("/pick_up", String, self.pickup_callback)
        rospy.Subscriber("/drop_off", String, self.dropoff_callback)
        
        self.task_complete_sub = rospy.Subscriber("/task_complete", String, self.task_complete_callback)
        self.state_update_pub = rospy.Publisher("/state_update", RobotState, queue_size=10)

    def state_callback(self, msg, robot_id):
        self.robot_states[robot_id] = msg.state
        rospy.loginfo(f"[{robot_id}] State updated: {msg.state}")
        if msg.state == "IDLE":
            self.assign_tasks_from_queue()

    def pose_callback(self, msg, robot_id):
        self.robot_positions[robot_id] = msg.pose.pose
        rospy.loginfo(f"[{robot_id}] Position updated: x={msg.pose.pose.position.x:.2f}, y={msg.pose.pose.position.y:.2f}")

    def pickup_callback(self, msg):
        self.pending_pickup = msg.data.strip()
        rospy.loginfo(f"Received pickup: {self.pending_pickup}")
        self.try_assign_direct_task()

    def dropoff_callback(self, msg):
        self.pending_dropoff = msg.data.strip()
        rospy.loginfo(f"Received dropoff: {self.pending_dropoff}")
        self.try_assign_direct_task()

    def try_assign_direct_task(self):
        if self.pending_pickup and self.pending_dropoff:
            self.task_queue.append((self.pending_pickup, self.pending_dropoff))
            self.pending_pickup = None
            self.pending_dropoff = None
            self.assign_tasks_from_queue()

    def task_queue_callback(self, msg):
        rospy.loginfo(f"Received queued task externally: {msg.pickup} → {msg.dropoff}")
        self.task_queue.append((msg.pickup, msg.dropoff))
        self.assign_tasks_from_queue()

    def assign_tasks_from_queue(self):
        while self.task_queue:
            idle_robots = [r for r, s in self.robot_states.items() if s == "IDLE"]
            if not idle_robots:
                rospy.loginfo("No idle robots available.")
                break

            pickup_letter, dropoff_zone = self.task_queue[0]  # Do not pop yet

            if pickup_letter not in self.shelf_poses:
                rospy.logwarn(f"Shelf '{pickup_letter}' not found in JSON.")
                self.task_queue.pop(0)  # Invalid task, discard
                continue

            pickup_pose = self.shelf_poses[pickup_letter]

            # Find closest idle robot
            best_robot = None
            min_dist = float('inf')
            for robot_id in idle_robots:
                dist = self.get_distance(self.robot_positions[robot_id], pickup_pose)
                if dist < min_dist:
                    min_dist = dist
                    best_robot = robot_id

            if best_robot:
                self.assign_task(best_robot, pickup_letter, dropoff_zone)
                self.robot_states[best_robot] = "BUSY"
                self.task_queue.pop(0)  # Only pop if assignment succeeded
                break  # Prevent assigning same task to multiple robots
            else:
                break

    def assign_task(self, robot_id, pickup_letter, dropoff_zone):
        task_id = f"{pickup_letter}_{dropoff_zone}"
        rospy.loginfo(f"Assigning task to {robot_id}: Pick {pickup_letter}, Drop {dropoff_zone}")

        task_msg = TaskCmd()
        task_msg.task_id = task_id

        rospy.sleep(0.1)
        self.task_publishers[robot_id].publish(task_msg)
        self.assigned_task_pub.publish(task_msg)

    def get_distance(self, pose, target):
        if pose is None:
            return float('inf')
        return math.hypot(pose.position.x - target['x'], pose.position.y - target['y'])
    
    def task_complete_callback(self, msg):
        """Handle task completion notifications"""
        robot_id = msg.data
        rospy.loginfo(f"Task completed by {robot_id}")
        
        # Create a state update message to set robot back to IDLE
        state_msg = RobotState()
        state_msg.robot_id = robot_id
        state_msg.state = "IDLE"
        state_msg.current_task = ""
        self.state_update_pub.publish(state_msg)


if __name__ == "__main__":
    json_file_path = "/home/mariam/multi_rob/src/grad/qr_locations_gazebo.json"
    TaskAssigner(json_file_path)
    rospy.spin()
