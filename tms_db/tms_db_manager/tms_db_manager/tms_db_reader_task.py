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

import json
import pymongo

import rclpy
from rclpy.node import Node

import tms_db_manager.tms_db_util as db_util
from tms_msg_db.srv import TmsdbGetTask


class TmsDbReaderTask(Node):
    """Read task data from MongoDB."""

    def __init__(self):
        super().__init__('tms_db_reader_task')

        # Declare parameters
        self.declare_parameter('db_host', 'localhost')
        self.declare_parameter('db_port', 27017)

        # Get parameters
        self.db_host: str = self.get_parameter('db_host').get_parameter_value().string_value
        self.db_port: int = self.get_parameter('db_port').get_parameter_value().integer_value

        self.db: pymongo.database.Database  = db_util.connect_db('rostmsdb', self.db_host, self.db_port)
        self.srv = self.create_service(TmsdbGetTask, 'tms_db_reader_task', self.db_reader_srv_callback)


    def db_reader_srv_callback(self, request, response):
        collection: pymongo.collection.Collection = self.db["task"]
        task: str = self.get_task_data(request.task_id, collection)
        response.task = task
        return response
        
    
    def get_task_data(self, task_id, collection: pymongo.collection.Collection) -> str:
        doc = collection.find_one({"task_id": task_id})

        if doc == None:
            self.get_logger().info(f"The task ID({task_id}) does not exist under the task collection in the rostmsdb database")
            return ''
        
        self.get_logger().info(f"Successfully retrieved the task (ID: {task_id})")
        return str(doc["task_sequence"])


def main(args=None):
    rclpy.init(args=args)
    tms_db_reader_task = TmsDbReaderTask()
    rclpy.spin(tms_db_reader_task)
    tms_db_reader_task.destroy_node()
    rclpy.shutdown()


if __name__=='__main__':
    main()