#!/usr/bin/env python3

import rospy
from std_msgs.msg import String, Float64

def actuator_callback(msg):
    if msg.data == "lift":
        rospy.loginfo(f"[{robot_name}] Command received: lift")
        send_command(0.1)
    elif msg.data == "drop":
        rospy.loginfo(f"[{robot_name}] Command received: drop")
        send_command(0.0)
    else:
        rospy.logwarn(f"[{robot_name}] Unknown command received: {msg.data}")

def send_command(value):
    rate = rospy.Rate(10)  # 10 Hz
    start_time = rospy.Time.now()
    duration = rospy.Duration(3.0)  # 3 seconds
    
    # Publish status that we're starting
    status_pub.publish("working")
    
    while not rospy.is_shutdown() and rospy.Time.now() - start_time < duration:
        pub.publish(Float64(value))
        rate.sleep()
    
    # Publish status that we're done
    status_pub.publish("done")
    rospy.loginfo(f"[{robot_name}] Done sending command {value}")

if __name__ == '__main__':
    rospy.init_node('linear_actuator_controller', anonymous=True)

    robot_name = rospy.get_param("~robot_name")
    topic_name = f"/{robot_name}/joint1_position_controller/command"
    pub = rospy.Publisher(topic_name, Float64, queue_size=10)
    status_pub = rospy.Publisher(f"/{robot_name}/actuator_status", String, queue_size=10)

    rospy.Subscriber(f"/{robot_name}/linear_actuator", String, actuator_callback)

    rospy.loginfo(f"[{robot_name}] Subscribed to /{robot_name}/linear_actuator, publishing to {topic_name}")
    rospy.spin()