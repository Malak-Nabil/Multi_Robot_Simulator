#!/usr/bin/env python3
import rospy
import json
import os
from sensor_msgs.msg import Image
from pyzbar.pyzbar import decode
from cv_bridge import CvBridge
import tf
from geometry_msgs.msg import Pose2D

# Global variables
robot_x, robot_y = 0.0, 0.0
qr_map = {}
bridge = CvBridge()
save_path = '/home/mariam/multi_rob/src/grad/trial.json'
listener = None  # TF listener (will be initialized in main)

def load_existing_qr_map():
    global qr_map
    try:
        if os.path.exists(save_path):
            if os.path.getsize(save_path) == 0:
                qr_map = {}
                with open(save_path, 'w') as f:
                    json.dump({}, f)
                rospy.loginfo("QR map file was empty, initialized with empty dict")
            else:
                with open(save_path, 'r') as f:
                    qr_map = json.load(f)
                rospy.loginfo(f"Loaded existing QR map with {len(qr_map)} entries.")
        else:
            qr_map = {}
            with open(save_path, 'w') as f:
                json.dump({}, f)
            rospy.loginfo("Created new QR map file with empty dict")
    except Exception as e:
        rospy.logerr(f"Error loading QR map: {str(e)}")
        qr_map = {}

def get_robot_pose():
    global robot_x, robot_y
    try:
        # Wait for the transform to be available
        listener.waitForTransform('map', 'robot1/base_footprint', rospy.Time(0), rospy.Duration(1.0))
        (trans, rot) = listener.lookupTransform('map', 'robot1/base_footprint', rospy.Time(0))
        robot_x = trans[0]
        robot_y = trans[1]
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        rospy.logwarn("Transform not available, keeping last known pose.")

def image_callback(msg):
    global qr_map
    get_robot_pose()  # Update the robot pose at the moment of QR detection

    cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
    decoded_objects = decode(cv_image)

    for obj in decoded_objects:
        data = obj.data.decode('utf-8')
        if data:
            if data not in qr_map:
                rospy.loginfo(f"New QR detected: {data} at ({robot_x:.2f}, {robot_y:.2f})")
                qr_map[data] = {"x": robot_x, "y": robot_y}
                save_to_file()
            else:
                rospy.loginfo(f"QR '{data}' already recorded, ignoring.")

def save_to_file():
    with open(save_path, 'w') as f:
        json.dump(qr_map, f, indent=4)
    rospy.loginfo(f"Saved QR locations to {save_path}")

def main():
    global listener
    rospy.init_node('qr_mapper_setup')

    load_existing_qr_map()

    # Initialize TF listener once
    listener = tf.TransformListener()

    # Subscribe to camera topic
    rospy.Subscriber('/robot1/robot1/camera1/robot1/image_raw', Image, image_callback)

    rospy.loginfo("QR Mapping setup node started. Move the robot to scan QR codes.")
    rospy.spin()

if __name__ == '__main__':
    main()
