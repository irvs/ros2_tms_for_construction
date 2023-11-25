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

package_name = 'tms_ts_subtask'

MONGODB_IPADDRESS = '127.0.0.1'
MONGODB_PORTNUMBER = 27017

# XML形式で記載されたBT treeを読み込んでmongodbのtaskコレクションに新たなタスクを生成し格納するノード

class TaskRemover(Node):
    
    def __init__(self):
        super().__init__('task_remover')
        client = pymongo.MongoClient(MONGODB_IPADDRESS, MONGODB_PORTNUMBER)
        db = client['rostmsdb']
        collection = db['task']
        self.declare_parameter('task_id', 0)
        target_task_id = self.get_parameter("task_id").get_parameter_value().integer_value
        result = collection.delete_one({"task_id": target_task_id})
        if result.deleted_count == 1:
            print(f"Successfully deleted document with task_id: {target_task_id}")
        else:
            print(f"No task found with task_id: {target_task_id}.")


def main(args=None):
    rclpy.init(args=args)
    task_remover = TaskRemover()
    rclpy.spin_once(task_remover)
    task_remover.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()