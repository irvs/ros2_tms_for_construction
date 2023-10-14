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
import tkinter as tk
from pymongo import MongoClient

from rclpy.node import Node

MONGODB_IPADDRESS = '127.0.0.1'
MONGODB_PORTNUMBER = 27017

task_id = 1

class GUI_button(Node):

    def __init__(self):
        super().__init__('button_input_bt')
        self.publisher_ = self.create_publisher(String, '/task_sequence', 10)
        root = tk.Tk()
        root.title("CONTROL PANEL")
        root.geometry("300x200")
        button = tk.Button(root, text="demo_2024_3", command = self.button_click, width=20, height=10)
        button.pack()
        root.mainloop()


    def button_click(self):
        print("Start the demo_2024_3 task")
        self.arg_data = {}
        self.search_task()
        if self._is_valid_taskid == True:
            msg = String()
            msg.data = self.task_sequence
            self.publisher_.publish(msg)
        else:
            self.get_logger().error(f"Stop the Task Scheduler because of the invalid task ID({task_id})")
    
    
    def search_task(self):
        client = MongoClient(MONGODB_IPADDRESS, MONGODB_PORTNUMBER)
        db = client['rostmsdb']
        collection = db['default']
        self.task_info = db.default.find_one({"task_id": task_id})
        if self.task_info != None:
            self._is_valid_taskid = True
            self.task_sequence = self.task_info["task_sequence"]
            self.get_logger().info(f"Execute the task corresponding to the specified task ID({task_id})")
        else:
            self.get_logger().info(f"The task corresponding to the specified task ID({task_id}) does not exist under the default collection in the rostmsdb database")
            self._is_valid_taskid = False


    # def search_parameters(self):
    #     # etcdataのタスク列に記載のあるパラメータ情報をもとにパラarg_dataータへの直接の検索を行うプログラムを作成する
    #     client = MongoClient(MONGODB_IPADDRESS, MONGODB_PORTNUMBER)
    #     db = client['rostmsdb']
    #     collection = db['default']
    #     task_sequence = self.task["etcdata"]
    #     subtask_raw_list = re.findall(r'[0-9]+\$\{.*?\}|[0-9]+|\+|\|', task_sequence)
    #     #self.get_logger().info(f"subtask_raw_list: {subtask_raw_list}")
    #     for subtask_raw in subtask_raw_list:
    #         subtask = subtask_raw.split("$")
    #         subtask_id = subtask[0]
    #         parameter_info = re.findall(r'\(.*?\)',str(subtask))[0].replace("(", "").replace(")", "")
    #         #self.get_logger().info(f"parameter {parameter}")
    #         parameter_info_components = parameter_info.split(".")
    #         self.get_logger().info(f"parameter_components {parameter_info_components}")
    #         parameters = db.default.find_one({"type": parameter_info_components[0]})
    #         del parameters["_id"]
    #         self.arg_data[parameters["type"]] = parameters


def main(args=None):
    rclpy.init(args=args)
    button_input = GUI_button()
    rclpy.spin(button_input)
    button_input.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()