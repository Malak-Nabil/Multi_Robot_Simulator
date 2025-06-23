#!/usr/bin/env python3

import rospy
from grad.msg import TaskCmd, RobotState

class RobotStateNode:
    def __init__(self, robot_id):
        self.robot_id = robot_id
        self.state_pub = rospy.Publisher(f"/{robot_id}/state", RobotState, queue_size=10,latch=True)
        self.task_sub = rospy.Subscriber(f"/{robot_id}/task_cmd", TaskCmd, self.task_callback)
        
        self.state_sub = rospy.Subscriber("/state_update", RobotState, self.state_update_callback)

        self.current_task = ""
        self.current_state = "IDLE"

        # Wait a short moment to ensure ROS connections are established
        rospy.sleep(0.5)
        self.publish_state()

    def task_callback(self, msg):
        """Callback function to handle task messages."""
        if msg.task_id:  # Task received → set state to BUSY
            self.current_state = "BUSY"
            self.current_task = msg.task_id
            rospy.loginfo(f"[{self.robot_id}] Received task {msg.task_id}")
        self.publish_state()

    def publish_state(self):
        """Publish the current state to the /robot_id/state topic."""
        msg = RobotState()
        msg.robot_id = self.robot_id
        msg.state = self.current_state
        msg.current_task = self.current_task
        self.state_pub.publish(msg)
        rospy.loginfo(f"[{self.robot_id}] State updated: {msg.state}")
        
    def state_update_callback(self, msg):
        """Callback for external state updates"""
        if msg.robot_id == self.robot_id:
            self.current_state = msg.state
            self.current_task = msg.current_task
            self.publish_state()

if __name__ == "__main__":
    rospy.init_node("robot_state")
    robot_id = rospy.get_param("~robot_id")
    RobotStateNode(robot_id)
    rospy.spin()
