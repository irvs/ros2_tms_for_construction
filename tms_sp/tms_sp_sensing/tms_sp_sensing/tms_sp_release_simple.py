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

from geometry_msgs.msg import Pose
from std_msgs.msg import String
from pymongo import MongoClient
from sensing_msgs.msg import ReleaseSimple

import tms_db_manager.tms_db_util as db_util

MONGODB_IPADDRESS = '127.0.0.1'
MONGODB_PORTNUMBER = 27017

DATA_ID = 10000
DATA_TYPE = 'parameter' 
DATA_NAME = 'data_name'

class UpdateDB_Parameter(Node):
    def __init__(self):
        super().__init__("tms_sp_release_simple")
        self.subscription = self.create_subscription(
            ReleaseSimple,
            '/tms_sp_release_simple',
            self.update_db_parameter,
            10) 
    
    def update_db_parameter(self, msg: ReleaseSimple) -> None:
        client = MongoClient(MONGODB_IPADDRESS, MONGODB_PORTNUMBER)
        db = client['rostmsdb']
        collection = db['parameter']
        query = {"model_name": msg.model_name, "type" : "dynamic", "record_name": msg.record_name}
        parameter_info = collection.find_one(query)
        update_parameter_info = {}
        msg_fields_and_types = msg.get_fields_and_field_types()
        msg_fields = list(msg_fields_and_types.keys())

        for key, value in parameter_info.items():
           if key != "_id" and key!="model_name" and key != "type" and key != "record_name" and key != "description":
                if key.lower() in msg_fields:
                    lower_key = key.lower()
                    update_parameter_info[key] = eval(f"msg.{lower_key}")
                else:
                    update_parameter_info[key] = value 

        update_query = {"$set": update_parameter_info}
        collection.update_one(query, update_query)

def main(args=None):
    rclpy.init(args=args)
    update_db_parameter = UpdateDB_Parameter()
    rclpy.spin(update_db_parameter)
    update_db_parameter.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()