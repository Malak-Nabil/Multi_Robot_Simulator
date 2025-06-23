#!/usr/bin/env python3

import rospy
import json
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from grad.msg import TaskCmd
from std_msgs.msg import String

class RobotController:
    def __init__(self, robot_id, json_file):
        self.robot_id = robot_id
        self.expected_qr = None
        self.qr_detected = None
        self.at_goal_position = False
        self.current_task = None
        self.actuator_done = False
        
        # Load shelf locations from JSON file
        with open(json_file, 'r') as f:
            self.shelf_poses = json.load(f)
        
        # ROS communication
        self.task_sub = rospy.Subscriber(f"/{robot_id}/task_cmd", TaskCmd, self.task_callback)
        self.qr_sub = rospy.Subscriber(f"/{robot_id}/qr_detection", String, self.qr_callback)
        self.actuator_pub = rospy.Publisher(f"/{robot_id}/linear_actuator", String, queue_size=10, latch=True)
        self.actuator_status_sub = rospy.Subscriber(f"/{robot_id}/actuator_status", String, self.actuator_status_callback)
        
        self.task_complete_pub = rospy.Publisher("/task_complete", String, queue_size=10)
        
        # MoveBase action client
        self.move_base_client = actionlib.SimpleActionClient(f"/{robot_id}/move_base", MoveBaseAction)
        rospy.loginfo(f"[{robot_id}] Waiting for move_base server...")
        self.move_base_client.wait_for_server()
        rospy.loginfo(f"[{robot_id}] Connected to move_base server")

    def actuator_status_callback(self, msg):
        """Handle actuator status updates"""
        if msg.data == "done":
            self.actuator_done = True
            rospy.loginfo(f"[{self.robot_id}] Actuator operation completed")

    def qr_callback(self, msg):
        """Handle detected QR codes"""
        self.qr_detected = msg.data
        rospy.loginfo(f"[{self.robot_id}] Detected QR: {self.qr_detected}")
        
        # Check if we're at goal position and QR matches
        if self.at_goal_position and self.qr_detected == self.expected_qr:
            if self.current_task == "pickup":
                self.handle_successful_pickup()
            elif self.current_task == "dropoff":
                self.handle_successful_dropoff()

    def task_callback(self, msg):
        """Handle new task assignment"""
        if not msg.task_id:
            return
            
        task_parts = msg.task_id.split('_')
        if len(task_parts) != 2:
            rospy.logerr(f"[{self.robot_id}] Invalid task format")
            return
            
        pickup_shelf, dropoff_zone = task_parts
        self.expected_qr = pickup_shelf
        self.current_task = "pickup"
        self.dropoff_zone = dropoff_zone
        
        if pickup_shelf in self.shelf_poses:
            self.navigate_to_pickup(pickup_shelf)
        else:
            rospy.logerr(f"[{self.robot_id}] Unknown shelf: {pickup_shelf}")

    def navigate_to_pickup(self, shelf_id):
        """Navigate to shelf and wait for QR verification"""
        self.at_goal_position = False
        pickup_pose = self.shelf_poses[shelf_id]
        
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.pose.position.x = pickup_pose['x']
        goal.target_pose.pose.position.y = pickup_pose['y']
        goal.target_pose.pose.orientation.w = 1.0  # Neutral orientation
        
        rospy.loginfo(f"[{self.robot_id}] Navigating to {shelf_id} at ({pickup_pose['x']}, {pickup_pose['y']})")
        self.move_base_client.send_goal(goal)
        
        # Wait for navigation to complete
        if self.move_base_client.wait_for_result(rospy.Duration(3000)):
            self.at_goal_position = True
            rospy.loginfo(f"[{self.robot_id}] Reached shelf area, waiting for QR verification...")
        else:
            rospy.logwarn(f"[{self.robot_id}] Navigation timeout")
            self.move_base_client.cancel_goal()

    def navigate_to_dropoff(self):
        """Navigate to dropoff zone"""
        self.at_goal_position = False
        dropoff_pose = self.shelf_poses[self.dropoff_zone]
        
        # Set expected QR for dropoff (assuming dropoff zones start with Z)
        self.expected_qr = self.dropoff_zone  # e.g., "ZB1"
        
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.pose.position.x = dropoff_pose['x']
        goal.target_pose.pose.position.y = dropoff_pose['y']
        goal.target_pose.pose.orientation.w = 1.0  # Neutral orientation
        
        rospy.loginfo(f"[{self.robot_id}] Navigating to dropoff at ({dropoff_pose['x']}, {dropoff_pose['y']})")
        self.move_base_client.send_goal(goal)
        
        # Wait for navigation to complete
        if self.move_base_client.wait_for_result(rospy.Duration(3000)):
            self.at_goal_position = True
            self.current_task = "dropoff"
            rospy.loginfo(f"[{self.robot_id}] Reached dropoff area, waiting for QR verification...")
        else:
            rospy.logwarn(f"[{self.robot_id}] Navigation timeout")
            self.move_base_client.cancel_goal()

    def handle_successful_pickup(self):
        """Actions to perform when QR is verified at pickup"""
        rospy.loginfo(f"[{self.robot_id}] Verified QR {self.expected_qr} - activating linear actuator")
        
        # Actuate linear actuator
        self.actuator_done = False
        self.actuator_pub.publish("lift")
        
        # Wait for actuator to complete
        while not self.actuator_done and not rospy.is_shutdown():
            rospy.sleep(0.1)
        
        rospy.loginfo(f"[{self.robot_id}] Pickup completed successfully")
        
        # Reset state and navigate to dropoff
        self.at_goal_position = False
        self.expected_qr = None
        self.navigate_to_dropoff()

    def handle_successful_dropoff(self):
        """Actions to perform when QR is verified at dropoff"""
        rospy.loginfo(f"[{self.robot_id}] Verified at dropoff - activating linear actuator")
        
        # Actuate linear actuator
        self.actuator_done = False
        self.actuator_pub.publish("drop")
        
        # Wait for actuator to complete
        while not self.actuator_done and not rospy.is_shutdown():
            rospy.sleep(0.1)
        
        rospy.loginfo(f"[{self.robot_id}] Dropoff completed successfully")
        
        self.task_complete_pub.publish(self.robot_id)
            
        # Reset state
        self.at_goal_position = False
        self.expected_qr = None
        self.current_task = None

if __name__ == "__main__":
    rospy.init_node("robot_controller")
    robot_id = rospy.get_param("~robot_id")
    json_file = rospy.get_param("~json_file", "/home/mariam/multi_rob/src/grad/qr_locations_gazebo.json")
    RobotController(robot_id, json_file)
    rospy.spin()