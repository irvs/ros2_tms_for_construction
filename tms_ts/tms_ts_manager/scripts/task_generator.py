#!/usr/bin/env python3

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
import pymongo
from ament_index_python.packages import get_package_share_directory
import xml.etree.ElementTree as ET
import re

package_name = 'tms_ts_manager'

MONGODB_IPADDRESS = '127.0.0.1'
MONGODB_PORTNUMBER = 27017

# XML形式で記載されたBT treeを読み込んでmongodbのtaskコレクションに新たなタスクを生成し格納するノード

class TaskGenerator(Node):
    
    def __init__(self):
        super().__init__('task_generator')
        client = pymongo.MongoClient(MONGODB_IPADDRESS, MONGODB_PORTNUMBER)
        db = client['rostmsdb']
        self.collection = db['task']

        tms_ts_subtask_package_directory = get_package_share_directory(package_name)
        self.task_info = {"task_id": None, "type": None, "model_name": None, "description": None, "task_sequence": None}

        self.declare_parameter('bt_tree_xml_file_name', 'sample_construction_tree')
        self.declare_parameter("output_text_file_directory_path", tms_ts_subtask_package_directory + '/config')
        self.declare_parameter("output_file_name", 'task_sequence')
        self.declare_parameter("output_txt_file", False)
        self.declare_parameter("description", "No Commented ...")

        self.xml_file_name = self.get_parameter("bt_tree_xml_file_name").get_parameter_value().string_value
        output_text_file_directory = self.get_parameter("output_text_file_directory_path").get_parameter_value().string_value
        output_file_name = self.get_parameter("output_file_name").get_parameter_value().string_value
        self.output_txt_file = self.get_parameter("output_txt_file").get_parameter_value().bool_value

        self.unique_task_sequence = True

        self.output_file_path = output_text_file_directory + "/" + output_file_name + '.txt'
        self.xml_file_path = tms_ts_subtask_package_directory + '/config/' + self.xml_file_name + '.xml'
        self.get_logger().info(f"xml_file_path: {self.xml_file_path}")

        self.format_task_sequnece()
        if self.output_txt_file == True:
            self.create_text_file(self.output_file_path, self.task_sequence)
        
        duplicated_task_id = self.dupricate_task_sequence_checker()
        if(self.unique_task_sequence == True):
            self.task_info["task_id"] = self.search_recommend_task_id()
            self.task_info["type"] = "task"
            self.task_info["task_sequence"] = self.task_sequence
            self.task_info["model_name"] = self.model_name
            self.task_info["description"] = self.get_parameter("description").get_parameter_value().string_value
            self.set_task()
            self.get_logger().info(f"Completed Inseting task data into the rostmsdb database ! The task_id is {self.task_info['task_id']}.")
        else:
            self.get_logger().info(f"This task sequence is already registered in the database. The task_id of the task sequence is {duplicated_task_id}.")

        
    # XML形式で記載されたBT treeを読み込んで整形し、文字列に変換する関数
    def format_task_sequnece(self):
        tree = ET.parse(self.xml_file_path)
        root = tree.getroot()
        self.task_sequence = ET.tostring(root, encoding='unicode', method='xml')
        self.task_sequence = '\n'.join(line.strip() for line in self.task_sequence.split('\n'))
        self.task_sequence = re.sub(r'\n', '', self.task_sequence)
        keyword_pattern = re.compile(r'(zx120|zx200|ic120)', re.I)
        matches = keyword_pattern.findall(self.task_sequence)
        unique_keywords = set(match.lower() for match in matches)
        self.model_name = ",".join(unique_keywords)
        
    
    # 必要であればタスク列をテキストファイルとして出力する関数
    def create_text_file(self, file_path, content):
        try:
            with open(file_path, 'w') as file:
                file.write(content)
            print(f"File '{file_path}' created successfully.")
        except Exception as e:
            print(f"Error: {e}")
    
    # mongodbのrostmsdbデータベースのtaskコレクションに含まれるすべてのタスクの内、もっとも適したtask_idを返す関数
    def search_recommend_task_id(self):
        max_num = self.collection.find_one(sort=[("task_id", pymongo.DESCENDING)])["task_id"]
        existing_task_ids = set()
        reccomend_task_id = None
        for document in self.collection.find({}, {"task_id": 1}):
            existing_task_ids.add(document["task_id"])
        for i in range(1, max_num + 1):
            if i not in existing_task_ids:
                reccomend_task_id = i
                break
        if(reccomend_task_id != None):
            return reccomend_task_id
        else:
            return max_num + 1
    
    # mongodbのrostmsdbデータベースのtaskコレクションに新たなタスクを格納する関数
    def set_task(self):
        self.task_info["task_sequence"] = self.task_sequence
        self.collection.insert_one(self.task_info)
        self.get_logger().info(f"Completed Inseting task data into the rostmsdb database !")
    
    # mongodbのrostmsdbデータベースのtaskコレクションに新たなタスクを格納する前に、すでに同じタスク列を持つタスクが格納されていないか確認する関数
    def dupricate_task_sequence_checker(self):
        for document in self.collection.find({}):
             if "task_sequence" in document:
                task_sequence_value = document["task_sequence"]
                task_id = document["task_id"]
                if isinstance(task_sequence_value, str) and self.task_sequence in task_sequence_value:
                    self.unique_task_sequence = False
                    return task_id
    


def main(args=None):
    rclpy.init(args=args)
    task_generator = TaskGenerator()
    rclpy.spin_once(task_generator)
    task_generator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()