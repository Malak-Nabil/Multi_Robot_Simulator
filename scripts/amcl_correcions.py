#!/usr/bin/env python3
import rospy
import json
import os
import time
from sensor_msgs.msg import Image, Imu
from pyzbar.pyzbar import decode
from cv_bridge import CvBridge
import tf
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import PoseWithCovarianceStamped
import numpy as np

# Path to the known QR locations JSON file
known_qr_path = '/home/mariam/multi_rob/src/grad/qr_locations_gazebo.json'

bridge = CvBridge()
seen_qrs = set()
qr_locations = {}
amcl_pub = None
current_orientation_quat = None

# Reset interval for seen_qrs
reset_interval = 10
last_reset_time = time.time()

def normalize_quaternion(q):
    q = np.array(q)
    return q / np.linalg.norm(q)

def load_known_qr_locations():
    global qr_locations
    if os.path.exists(known_qr_path):
        with open(known_qr_path, 'r') as f:
            qr_locations = json.load(f)
        rospy.loginfo(f"Loaded {len(qr_locations)} known QR positions.")
    else:
        rospy.logwarn(f"QR locations file not found: {known_qr_path}")
        qr_locations = {}

def imu_callback(msg):
    global current_orientation_quat
    q = msg.orientation
    current_orientation_quat = normalize_quaternion([q.x, q.y, q.z, q.w])
    rospy.loginfo_once("IMU orientation received and stored.")

def publish_amcl_pose(qr_id, x, y):
    quaternion = current_orientation_quat

    if quaternion is None:
        rospy.logwarn(f"[AMCL] QR '{qr_id}' detected but IMU orientation not yet available. Skipping pose update.")
        return

    (roll, pitch, yaw) = euler_from_quaternion(quaternion)
    rospy.loginfo(f"Pre-publish check - Yaw: {yaw:.2f} radians ({np.degrees(yaw):.1f}°)")

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
        0.25, 0,    0, 0, 0, 0,
        0,    0.25, 0, 0, 0, 0,
        0,    0,    0.0, 0, 0, 0,
        0,    0,    0, 0.0, 0, 0,
        0,    0,    0, 0, 0.0, 0,
        0,    0,    0, 0, 0, 0.0685
    ]

    rospy.loginfo(f"[AMCL] QR '{qr_id}' detected → Updating pose to x={x:.2f}, y={y:.2f}, quaternion={quaternion}")
    amcl_pub.publish(pose_msg)

def image_callback(msg):
    global seen_qrs, last_reset_time

    # Reset seen_qrs based on time interval
    current_time = time.time()
    if current_time - last_reset_time > reset_interval:
        seen_qrs.clear()
        last_reset_time = current_time

    cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
    decoded_objects = decode(cv_image)

    for obj in decoded_objects:
        qr_id = obj.data.decode('utf-8').strip()

        if qr_id in seen_qrs:
            continue  # Already processed recently

        if qr_id in qr_locations:
            loc = qr_locations[qr_id]
            publish_amcl_pose(qr_id, loc["x"], loc["y"])
            seen_qrs.add(qr_id)
        else:
            rospy.logwarn(f"Detected unknown QR code: '{qr_id}' — not in known list.")

def main():
    global amcl_pub

    rospy.init_node('qr_amcl_updater_from_camera')
    load_known_qr_locations()

    rospy.Subscriber('/robot2/robot2/imu/filtered', Imu, imu_callback)
    amcl_pub = rospy.Publisher('/robot2/initialpose', PoseWithCovarianceStamped, queue_size=1)
    rospy.Subscriber('/robot2/robot2/camera1/robot2/image_raw', Image, image_callback)

    # Wait until IMU data is received
    rospy.loginfo("Waiting for IMU orientation before processing QR detections...")
    rate = rospy.Rate(10)
    while current_orientation_quat is None and not rospy.is_shutdown():
        rate.sleep()
    rospy.loginfo("IMU orientation acquired. QR detection and pose updates enabled.")

    rospy.loginfo("QR-AMCL updater running. Waiting for QR detections...")
    rospy.spin()

if __name__ == '__main__':
    main()
