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
import re

from rclpy.node import Node

MONGODB_IPADDRESS = '127.0.0.1'
MONGODB_PORTNUMBER = 27017


class GUI_button(Node):

    def __init__(self):
        super().__init__('button_input_bt')

        self.declare_parameter('task_id', 3)
        self.task_id = self.get_parameter("task_id").get_parameter_value().integer_value

        self.publisher_ = self.create_publisher(String, '/task_sequence', 10)
        root = tk.Tk()
        root.title("CONTROL PANEL")
        root.geometry("500x200")
        button_txt = "task_id: " + str(self.task_id) + "\n" + "Execute the task corresponding to the specified task ID"
        button = tk.Button(root, text="task_id :  " + str(self.task_id)+"\n" + "\n"+ "Execute the task corresponding to the specified task ID", command = self.button_click, width=50, height=8)
        button.pack()
        root.mainloop()

    # GUIボタンが押されたときに実行される関数
    def button_click(self):
        self.arg_data = {}
        self.search_task()
        if self._is_valid_taskid == True:
            msg = String()
            msg.data = self.task_sequence
            self.publisher_.publish(msg)
        else:
            self.get_logger().error(f"Stop the Task Scheduler because of the invalid task ID({self.task_id})")
    
    # タスクIDに対応するタスクをTMS_DBから検索する関数
    def search_task(self):
        client = MongoClient(MONGODB_IPADDRESS, MONGODB_PORTNUMBER)
        db = client['rostmsdb']
        collection = db['task']
        self.task_info = db.default.find_one({"task_id": self.task_id})
        if self.task_info != None:
            self._is_valid_taskid = True
            self.task_sequence = self.task_info["task_sequence"]
            self.set_parameters()
            self.get_logger().info(f"Execute the task corresponding to the specified task ID({self.task_id})")
        else:
            self.get_logger().info(f"The task corresponding to the specified task ID({self.task_id}) does not exist under the default collection in the rostmsdb database")
            self._is_valid_taskid = False

    # タスク列に動的にパラメータを埋め込むための関数
    def set_parameters(self):
        # 任意のパラメータ判別用記号(例: ${parameter})などをtask_sequenceの中に入れて、この部分を置き換えるプログラムを作成する
        client = MongoClient(MONGODB_IPADDRESS, MONGODB_PORTNUMBER)
        db = client['rostmsdb']
        collection = db['parameter']
        subtask_raw_list = re.findall(r'\$\[.*\]', self.task_sequence)
        if(len(subtask_raw_list) >= 1):
            for subtask_raw in subtask_raw_list:
                parts_task_sequence = self.task_sequence.split(subtask_raw)
                subtask_raw = subtask_raw.replace("$", "")
                subtask_raw = subtask_raw.replace("[", "")
                subtask_raw = subtask_raw.replace("]", "")
                parameter_tag = subtask_raw.split(":")
                parameter_id_tag = int(parameter_tag[0])
                parameter_value_tag = str(parameter_tag[1])
                self.parameter_info = db.default.find_one({"parameter_id": parameter_id_tag})
                self.parameter_value = self.parameter_info[parameter_value_tag]
                # self.get_logger().info(f"subtask_raw {subtask_raw}")
                # self.get_logger().info(f"parameter_value {self.parameter_value}")
                # self.get_logger().info(parts_task_sequence[0])
                # self.get_logger().info(parts_task_sequence[1])
                self.task_sequence = parts_task_sequence[0] + str(self.parameter_value) + parts_task_sequence[1]
                # self.get_logger().info("RESULT" + self.task_sequence)


def main(args=None):
    rclpy.init(args=args)
    button_input = GUI_button()
    rclpy.spin(button_input)
    button_input.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()