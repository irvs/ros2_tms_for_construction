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

from bson import ObjectId 

import tms_db_manager.tms_db_util as db_util
from tms_msg_db.msg import Tmsdb
from tms_msg_db.srv import TmsdbGetData


class TmsDbReaderParam(Node):
    """Read task data from MongoDB."""

    def __init__(self):
        super().__init__('tms_db_param_reader')

        # Declare parameters
        self.declare_parameter('db_host', 'localhost')
        self.declare_parameter('db_port', 27017)

        # Get parameters
        self.db_host: str = self.get_parameter('db_host').get_parameter_value().string_value
        self.db_port: int = self.get_parameter('db_port').get_parameter_value().integer_value

        self.db: pymongo.database.Database  = db_util.connect_db('rostmsdb', self.db_host, self.db_port)
        self.srv = self.create_service(TmsdbGetData, 'tms_db_param_reader', self.db_reader_srv_callback)


    def db_reader_srv_callback(self, request, response):
        collection = self.db[request.type]
        datatype = request.flgorparam
        data_result = self.get_latest_data(request, collection)

        if data_result is None:
            self.get_logger().warn("No data found for the given request.")
            return response

        if datatype == "param":
            if isinstance(data_result, list):
                for data in data_result:
                    response.tmsdbs.append(self.allocate_tmsdb(data))
            else:
                response.tmsdbs.append(self.allocate_tmsdb(data_result))

            return response
        
        elif datatype == "flg":
            response.tmsdbs.append(self.allocate_flg_tmsdb(data_result))
            return response        

    def get_latest_data(self, request, collection: pymongo.collection.Collection):
        if request.name != "":
            latest_data = collection.find_one({"record_name": request.name})
            if latest_data is None:
                self.get_logger().warn(f"No data found for {request.name} in {request.type}.")
            return latest_data

        elif len(request.recordnames) > 0:
            waypointlist = []
            for name in request.recordnames:
                data = collection.find_one({"record_name": name})
                if data:
                    waypointlist.append(data)
                else:
                    self.get_logger().warn(f"No data found for {name} in {request.type}.")
            return waypointlist

        return None
    

    def allocate_tmsdb(self, data: dict) -> Tmsdb:
        tmsdb = Tmsdb()
        tmsdb.type = data["type"]
        tmsdb.name = data["record_name"]
        pose_data = {
            "x": data.get("x", []),
            "y": data.get("y", []),
            "z": data.get("z", []),
            "qx": data.get("qx", []),
            "qy": data.get("qy", []),
            "qz": data.get("qz", []),
            "qw": data.get("qw", []),
            "Release_Count": data.get("Release_Count", []),
        }
        tmsdb.msg = json.dumps(pose_data)
        return tmsdb
    
    def convert_objectid(self, obj):
        if isinstance(obj, ObjectId):
            return str(obj)
        elif isinstance(obj, dict):
            return {k: self.convert_objectid(v) for k, v in obj.items()}
        elif isinstance(obj, list):
            return [self.convert_objectid(v) for v in obj]
        else:
            return obj

    def allocate_flg_tmsdb(self, data: dict) -> Tmsdb:
        tmsdb = Tmsdb()
        tmsdb.type = data["type"]
        tmsdb.name = data["record_name"]

        keys_to_exclude = ["type", "record_name"]
        flg_data_raw = {k: v for k, v in data.items() if k not in keys_to_exclude}

        # ObjectIdを文字列化してからJSON変換
        flg_data = self.convert_objectid(flg_data_raw)
        tmsdb.msg = json.dumps(flg_data)

        return tmsdb
    
def main(args=None):
    rclpy.init(args=args)
    tms_db_param_reader = TmsDbReaderParam()
    rclpy.spin(tms_db_param_reader)
    tms_db_param_reader.destroy_node()
    rclpy.shutdown()


if __name__=='__main__':
    main()