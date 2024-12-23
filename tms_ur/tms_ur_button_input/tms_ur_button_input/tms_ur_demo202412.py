# Copyright 2023, IRVS Laboratory, Kyushu University, Japan.
# 
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
# 
#     http://www.apache.org/licenses/LICENSE-2.0
# 
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_msgs.msg import Bool
import tkinter as tk
from pymongo import MongoClient
import re
import threading
import time
from tms_msg_ur.msg import Demo202412

from rclpy.node import Node

MONGODB_IPADDRESS = '127.0.0.1'
MONGODB_PORTNUMBER = 27017


class GUI_button(Node):

    def __init__(self):
        super().__init__('button_input_bt')

        self.declare_parameter('task_id1', 0)
        self.declare_parameter('task_id2', 0)
        self.task_id1 = self.get_parameter("task_id1").get_parameter_value().integer_value
        self.task_id2 = self.get_parameter("task_id2").get_parameter_value().integer_value
        self.emergency_signal_publisher = self.create_publisher(Bool, '/emergency_signal', 10)
        self.publisher_ = self.create_publisher(Demo202412, '/task_sequence', 10)
        root = tk.Tk()
        root.title("CONTROL PANEL")
        root.geometry("800x200")
        self.emergency_button = tk.Button(root, text="EMERGENCY" + "\n" + "\n" + "Urgently stops a running task", command=self.emergency_button_click, width=50, height=8, bg="red")
        self.emergency_button.pack(side="right")
        self.ts_button = tk.Button(root, text="task_id :  " + str(self.task_id1) + "," + str(self.task_id2) +"\n" + "\n"+ "Execute the task corresponding to the specified task ID", command = self.button_click, width=50, height=8, bg="green")
        self.ts_button.pack(side="right")
        
        root.mainloop()

    def emergency_button_click(self):
        self.ts_button["state"] = "disabled"
        self.ts_button.configure(bg="black")
        self.emergency_thread = threading.Thread(target=self.pub_emergency_signal)
        self.emergency_thread.start()

    def pub_emergency_signal(self):
        while True:
            msg = Bool()
            msg.data = True
            self.emergency_signal_publisher.publish(msg)
            time.sleep(0.01)


    def button_click(self):
        self.arg_data = {}
        task_sequence1 = self.search_task(self.task_id1)
        task_sequence2 = self.search_task(self.task_id2)

        if task_sequence1 != Node and task_sequence2 != None:
            task_sequences = Demo202412()
            task_sequences.task_sequence1 = task_sequence1
            task_sequences.task_sequence2 = task_sequence2
            self.publisher_.publish(task_sequences)
        else:
            if task_sequence1 == None:
                self.get_logger().error(f"Stop the Task Scheduler because of the invalid task ID({self.task_id1})")
            else:
                self.get_logger().error(f"Stop the Task Scheduler because of the invalid task ID({self.task_id2})")

    
    def search_task(self, task_id):
        client = MongoClient(MONGODB_IPADDRESS, MONGODB_PORTNUMBER)
        db = client['rostmsdb']
        collection = db['task']
        task_info = collection.find_one({"task_id": task_id})
        if task_info != None:
            # self._is_valid_taskid = True
            task_sequence = task_info["task_sequence"]
            # self.set_parameters()
            self.get_logger().info(f"Execute the task corresponding to the specified task ID({task_id})")
            return task_sequence
        else:
            self.get_logger().info(f"The task corresponding to the specified task ID({task_id}) does not exist under the default collection in the rostmsdb database")
            # self._is_valid_taskid = False
            return None

    # def set_parameters(self):
    #     client = MongoClient(MONGODB_IPADDRESS, MONGODB_PORTNUMBER)
    #     db = client['rostmsdb']
    #     collection = db['parameter']
    #     subtask_raw_list = re.findall(r'\$\[.*\]', self.task_sequence)
    #     if(len(subtask_raw_list) >= 1):
    #         for subtask_raw in subtask_raw_list:
    #             parts_task_sequence = self.task_sequence.split(subtask_raw)
    #             subtask_raw = subtask_raw.replace("$", "")
    #             subtask_raw = subtask_raw.replace("[", "")
    #             subtask_raw = subtask_raw.replace("]", "")
    #             parameter_tag = subtask_raw.split(":")
    #             parameter_id_tag = int(parameter_tag[0])
    #             parameter_value_tag = str(parameter_tag[1])
    #             self.parameter_info = collection.find_one({"parameter_id": parameter_id_tag})
    #             self.parameter_value = self.parameter_info[parameter_value_tag]
    #             # self.get_logger().info(f"subtask_raw {subtask_raw}")
    #             # self.get_logger().info(f"parameter_value {self.parameter_value}")
    #             # self.get_logger().info(parts_task_sequence[0])
    #             # self.get_logger().info(parts_task_sequence[1])
    #             self.task_sequence = parts_task_sequence[0] + str(self.parameter_value) + parts_task_sequence[1]
    #             # self.get_logger().info("RESULT" + self.task_sequence)


def main(args=None):
    rclpy.init(args=args)
    button_input = GUI_button()
    rclpy.spin(button_input)
    button_input.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()