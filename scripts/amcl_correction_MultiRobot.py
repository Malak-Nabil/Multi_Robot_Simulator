#!/usr/bin/env python3
import rospy
import json
import os
import time
import numpy as np
from sensor_msgs.msg import Image, Imu
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import String
from move_base_msgs.msg import MoveBaseActionResult
from pyzbar.pyzbar import decode
from cv_bridge import CvBridge
from tf.transformations import euler_from_quaternion, quaternion_from_euler

class QrAmclUpdater:
    def __init__(self, robot_ns):
        self.robot_ns = robot_ns
        self.seen_qrs = set()
        self.current_orientation_quat = None
        self.goal_reached = False
        self.last_reset_time = time.time()
        self.reset_interval = 10

        self.bridge = CvBridge()
        self.qr_locations = self.load_known_qr_locations()

        # Publishers
        self.amcl_pub = rospy.Publisher(f'/{robot_ns}/initialpose', PoseWithCovarianceStamped, queue_size=1)
        self.qr_pub = rospy.Publisher(f'/{robot_ns}/qr_detection', String, queue_size=1)

        # Subscribers
        rospy.Subscriber(f'/{robot_ns}/{robot_ns}/imu/filtered', Imu, self.imu_callback)
        rospy.Subscriber(f'/{robot_ns}/{robot_ns}/camera1/{robot_ns}/image_raw', Image, self.image_callback)
        rospy.Subscriber(f'/{robot_ns}/move_base/result', MoveBaseActionResult, self.goal_result_callback)

    def load_known_qr_locations(self):
        path = '/home/mariam/multi_rob/src/grad/qr_locations_gazebo.json'
        if os.path.exists(path):
            with open(path, 'r') as f:
                rospy.loginfo(f"[{self.robot_ns}] Loaded known QR positions.")
                return json.load(f)
        else:
            rospy.logwarn(f"[{self.robot_ns}] QR locations file not found: {path}")
            return {}

    def normalize_angle_rad(self, angle):
        """ Normalize angle to [-pi, pi] """
        return (angle + np.pi) % (2 * np.pi) - np.pi

    def extract_yaw_from_quaternion(self, q):
        """ Convert quaternion to yaw angle (radians) """
        _, _, yaw = euler_from_quaternion(q)
        return self.normalize_angle_rad(yaw)

    def quaternion_from_yaw(self, yaw):
        """ Convert yaw angle back to quaternion """
        return quaternion_from_euler(0, 0, yaw)

    def imu_callback(self, msg):
        q = msg.orientation
        self.current_orientation_quat = [q.x, q.y, q.z, q.w]

    def goal_result_callback(self, msg):
        status = msg.status.status
        if status == 3:  # SUCCEEDED
            rospy.loginfo(f"[{self.robot_ns}] MoveBase: Goal reached!")
            self.goal_reached = True
        else:
            self.goal_reached = False

    def publish_amcl_pose(self, qr_id, x, y):
        if self.current_orientation_quat is None:
            return

        yaw = self.extract_yaw_from_quaternion(self.current_orientation_quat)
        quaternion = self.quaternion_from_yaw(yaw)

        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.stamp = rospy.Time.now()
        pose_msg.header.frame_id = "map"
        pose_msg.pose.pose.position.x = x
        pose_msg.pose.pose.position.y = y
        pose_msg.pose.pose.position.z = 0.0
        pose_msg.pose.pose.orientation.x = quaternion[0]
        pose_msg.pose.pose.orientation.y = quaternion[1]
        pose_msg.pose.pose.orientation.z = quaternion[2]
        pose_msg.pose.pose.orientation.w = quaternion[3]

        pose_msg.pose.covariance = [
            0.25, 0, 0, 0, 0, 0,
            0, 0.25, 0, 0, 0, 0,
            0, 0, 0.0, 0, 0, 0,
            0, 0, 0, 0.0, 0, 0,
            0, 0, 0, 0, 0.0, 0,
            0, 0, 0, 0, 0, 0.0685
        ]

        rospy.loginfo(f"[{self.robot_ns}] [AMCL] QR '{qr_id}' detected → Updating pose to x={x:.2f}, y={y:.2f}, yaw={np.degrees(yaw):.1f}°")
        self.amcl_pub.publish(pose_msg)

    def image_callback(self, msg):
        if not self.goal_reached:
            return

        current_time = time.time()
        if current_time - self.last_reset_time > self.reset_interval:
            self.seen_qrs.clear()
            self.last_reset_time = current_time

        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        decoded_objects = decode(cv_image)

        for obj in decoded_objects:
            qr_id = obj.data.decode('utf-8').strip()
            if qr_id in self.seen_qrs:
                continue

            self.qr_pub.publish(qr_id)

            if qr_id in self.qr_locations:
                loc = self.qr_locations[qr_id]
                self.publish_amcl_pose(qr_id, loc["x"], loc["y"])
                self.seen_qrs.add(qr_id)
                self.goal_reached = False  # Wait for next goal
            else:
                rospy.logwarn(f"[{self.robot_ns}] Unknown QR code: '{qr_id}' — not in known list.")

def main():
    rospy.init_node('qr_amcl_updater_multi')
    robot_ns = rospy.get_param("~robot_namespace")
    QrAmclUpdater(robot_ns)
    rospy.loginfo("QR-AMCL multi-updater running...")
    rospy.spin()

if __name__ == '__main__':
    main()
