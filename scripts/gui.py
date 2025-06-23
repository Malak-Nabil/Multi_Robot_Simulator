#!/usr/bin/env python3
import rospy
import tkinter as tk
from tkinter import ttk
import time
from grad.msg import RobotState, TaskCmd


class RobotPanel:
    def __init__(self, parent, robot_id):
        self.robot_id = robot_id
        self.state = "IDLE"
        self.current_task = ""
        self.task_count = 0
        self.task_start_time = None
        self.total_task_time = 0

        self.frame = tk.LabelFrame(parent, text=f"{robot_id.upper()}", padx=10, pady=10)
        self.frame.pack(side=tk.LEFT, padx=10, pady=10)

        self.state_label = tk.Label(self.frame, text="State: IDLE")
        self.task_label = tk.Label(self.frame, text="Task: None")
        self.task_count_label = tk.Label(self.frame, text="Task Count: 0")
        self.time_taken_label = tk.Label(self.frame, text="Time Taken: 0s")

        self.state_led = tk.Canvas(self.frame, width=20, height=20)
        self.led = self.state_led.create_oval(5, 5, 15, 15, fill="green")

        self.state_label.pack()
        self.task_label.pack()
        self.task_count_label.pack()
        self.time_taken_label.pack()
        self.state_led.pack()

    def update(self, msg):
        prev_state = self.state
        self.state = msg.state.upper()
        self.current_task = msg.current_task

        self.state_label.config(text=f"State: {self.state}")
        self.task_label.config(text=f"Task: {self.current_task}")

        if self.state == "BUSY":
            if self.task_start_time is None:
                self.task_start_time = time.time()
            self.state_led.itemconfig(self.led, fill="red")
        elif self.state == "IDLE":
            if prev_state == "BUSY" and self.task_start_time is not None:
                task_time = int(time.time() - self.task_start_time)
                self.total_task_time += task_time
                self.task_count += 1
                self.task_start_time = None
            self.state_led.itemconfig(self.led, fill="green")

        self.task_count_label.config(text=f"Task Count: {self.task_count}")
        self.time_taken_label.config(text=f"Time Taken: {self.total_task_time}s")


class TaskGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("Multi-Robot Task GUI")

        # Robot Panels
        self.robot1_panel = RobotPanel(root, "robot1")
        self.robot2_panel = RobotPanel(root, "robot2")

        # Task Assignment Panel
        self.task_frame = tk.LabelFrame(root, text="Assign Task", padx=10, pady=10)
        self.task_frame.pack(side=tk.RIGHT, padx=10, pady=10)

        tk.Label(self.task_frame, text="Pick Up (Shelf):").pack()
        self.shelf_choice = tk.StringVar()
        self.shelf_menu = ttk.Combobox(self.task_frame, textvariable=self.shelf_choice)
        self.shelf_menu['values'] = ["A", "B", "C", "D"]
        self.shelf_menu.pack()

        tk.Label(self.task_frame, text="Drop Off (Zone):").pack()
        self.zone_choice = tk.StringVar()
        self.zone_menu = ttk.Combobox(self.task_frame, textvariable=self.zone_choice)
        self.zone_menu['values'] = ["ZA1", "ZB1", "ZC1", "ZD1"]
        self.zone_menu.pack()

        self.assign_button = tk.Button(self.task_frame, text="Assign Task", command=self.assign_task)
        self.assign_button.pack(pady=10)

        # Task Queue Panel
        self.queue_frame = tk.LabelFrame(root, text="Task Queue", padx=10, pady=10, width=200, height=150)
        self.queue_frame.pack(side=tk.LEFT, padx=10, pady=10)
        self.queue_frame.pack_propagate(False)

        self.task_queue = []
        self.queue_listbox = tk.Listbox(self.queue_frame, width=30, height=6)
        self.queue_listbox.pack()

        # ROS Setup
        rospy.Subscriber("/robot1/state", RobotState, self.robot1_callback)
        rospy.Subscriber("/robot2/state", RobotState, self.robot2_callback)
        rospy.Subscriber("/assigned_task", TaskCmd, self.task_assigned_callback)

        self.task_pub = rospy.Publisher("/task_queue", TaskCmd, queue_size=10, latch=True)

        self.root.after(100, self.tk_loop)

    def assign_task(self):
        pickup = self.shelf_choice.get()
        dropoff = self.zone_choice.get()

        if not pickup or not dropoff:
            rospy.logwarn("Both pickup and dropoff must be selected.")
            return

        task_str = f"{pickup} → {dropoff}"
        self.task_queue.append(task_str)
        self.queue_listbox.insert(tk.END, task_str)

        task_msg = TaskCmd()
        task_msg.pickup = pickup
        task_msg.dropoff = dropoff
        self.task_pub.publish(task_msg)

        rospy.loginfo(f"Published task to /task_queue: {task_str}")

    def task_assigned_callback(self, msg):
        task_str = msg.task_id.replace("_", " → ")
        if task_str in self.task_queue:
            index = self.task_queue.index(task_str)
            self.task_queue.pop(index)
            self.queue_listbox.delete(index)
            rospy.loginfo(f"Removed assigned task from queue: {task_str}")

    def robot1_callback(self, msg):
        self.robot1_panel.update(msg)

    def robot2_callback(self, msg):
        self.robot2_panel.update(msg)

    def tk_loop(self):
        self.root.update()
        self.root.after(100, self.tk_loop)


if __name__ == "__main__":
    rospy.init_node("task_gui")
    root = tk.Tk()
    app = TaskGUI(root)
    root.mainloop()
