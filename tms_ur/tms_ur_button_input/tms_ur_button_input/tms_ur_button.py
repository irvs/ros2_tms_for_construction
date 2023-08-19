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
from std_msgs.msg import Int64
import tkinter as tk
from pymongo import MongoClient
import json
import re

from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.action import ActionClient
from tms_msg_ts.action import TsReq

MONGODB_IPADDRESS = '127.0.0.1'
MONGODB_PORTNUMBER = 27017

task_id = 9001

class GUI_button(Node):

    def __init__(self):
        super().__init__('button_input')   
        root = tk.Tk()
        root.title("CONTROL PANEL")
        root.geometry("300x200")
        button = tk.Button(root, text="demo_2024_3", command = self.button_click, width=20, height=10)
        button.pack()
        root.mainloop()
        print("")


    def button_click(self):
        print("Start the Task Scheduler for demo_2024_3")
        self.arg_data = {}
        self.goal_handles = {}
        self.cb_group = ReentrantCallbackGroup()
        self.cli_ts_req = ActionClient(self, TsReq, 'tms_ts_master', callback_group=self.cb_group)
        while not self.cli_ts_req.wait_for_server(timeout_sec=1.0):
            self.get_logger().info('service "tms_ts_master" not available, waiting again...')
        self.search_task()
        if self._is_valid_taskid == True:
            self.search_parameters()
            req = TsReq.Goal()
            req.task_id = task_id
            req.arg_json = json.dumps(self.arg_data)
            #self.get_logger().info(f"self.arg_data: {self.arg_data}")
            goal_handle = self.cli_ts_req.send_goal_async(req)
            self.goal_handles[goal_handle.goal_id.uuid.tostring()] = goal_handle
            goal_handle.get_result_async().add_done_callback(self.done_callback)
            self.get_logger().info(f"Call task {task_id}")
        else:
            self.get_logger().error(f"Stop the Task Scheduler because of the invalid task ID({task_id})")
    
    
    def search_task(self):
        client = MongoClient(MONGODB_IPADDRESS, MONGODB_PORTNUMBER)
        db = client['rostmsdb']
        collection = db['default']
        self.task = db.default.find_one({"id": task_id})
        if self.task != None:
            self._is_valid_taskid = True
            self.get_logger().info(str(len(self.task)))
            self.get_logger().info(str(self.task))
            del self.task["_id"]
            self.get_logger().info(f"Execute the task corresponding to the specified task ID({task_id})")
        else:
            self.get_logger().info(f"The task corresponding to the specified task ID({task_id}) does not exist under the default collection in the rostmsdb database")
            self._is_valid_taskid = False


    def search_parameters(self):
        # etcdataのタスク列に記載のあるパラメータ情報をもとにパラarg_dataータへの直接の検索を行うプログラムを作成する
        client = MongoClient(MONGODB_IPADDRESS, MONGODB_PORTNUMBER)
        db = client['rostmsdb']
        collection = db['default']
        task_sequence = self.task["etcdata"]
        subtask_raw_list = re.findall(r'[0-9]+\$\{.*?\}|[0-9]+|\+|\|', task_sequence)
        #self.get_logger().info(f"subtask_raw_list: {subtask_raw_list}")
        for subtask_raw in subtask_raw_list:
            subtask = subtask_raw.split("$")
            subtask_id = subtask[0]
            parameter_info = re.findall(r'\(.*?\)',str(subtask))[0].replace("(", "").replace(")", "")
            #self.get_logger().info(f"parameter {parameter}")
            parameter_info_components = parameter_info.split(".")
            self.get_logger().info(f"parameter_components {parameter_info_components}")
            parameters = db.default.find_one({"type": parameter_info_components[0]})
            del parameters["_id"]
            self.arg_data[parameters["type"]] = parameters


def main(args=None):
    rclpy.init(args=args)
    button_input = GUI_button()
    rclpy.spin(button_input)
    button_input.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()