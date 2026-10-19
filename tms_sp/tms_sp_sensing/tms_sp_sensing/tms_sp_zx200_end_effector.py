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

from datetime import datetime
import json
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from geometry_msgs.msg import Pose
from std_msgs.msg import String
from pymongo import MongoClient
from sensing_msgs.msg import Zx200EndEffector

import tms_db_manager.tms_db_util as db_util

MONGODB_IPADDRESS = '127.0.0.1'
MONGODB_PORTNUMBER = 27017

DATA_ID = 10000
DATA_TYPE = 'parameter' 
DATA_NAME = 'data_name'

class UpdateDbParameter(Node):
    def __init__(self):
        super().__init__("tms_sp_zx200_end_effector")
        self.subscription = self.create_subscription(
            Zx200EndEffector,
            '/zx200/end_effector',
            self.update_db_parameter,
            10) 

        self.get_logger().info("UpdateDbParameter node has been started.")
    
    def update_db_parameter(self, msg: Zx200EndEffector) -> None:
        client = MongoClient(MONGODB_IPADDRESS, MONGODB_PORTNUMBER)
        db = client['rostmsdb']
        collection = db['parameter']
        query = {"model_name": msg.model_name, "record_name": msg.record_name}
        parameter_info = collection.find_one(query)
        if parameter_info is None:
            self.get_logger().warn(f"No parameter document found for query: {query}")
            return

        # waypoints内の最初のpose型waypointのdataを更新する
        waypoints = parameter_info.get("waypoints") or []
        pose_index = next(
            (i for i, wp in enumerate(waypoints) if isinstance(wp, dict) and wp.get("type") == "pose"),
            None)
        if pose_index is None:
            self.get_logger().warn(f"No pose waypoint found in record: {msg.record_name}")
            return

        update_parameter_info = {
            f"waypoints.{pose_index}.data.{key}": float(getattr(msg, key))
            for key in ("x", "y", "z", "theta_w")
        }
        update_query = {"$set": update_parameter_info}
        collection.update_one(query, update_query)



def main(args=None):
    rclpy.init(args=args)
    update_db_parameter = UpdateDbParameter()
    rclpy.spin(update_db_parameter)
    update_db_parameter.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()