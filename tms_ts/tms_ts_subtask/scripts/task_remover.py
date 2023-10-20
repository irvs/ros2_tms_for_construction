#!/usr/bin/env python3

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