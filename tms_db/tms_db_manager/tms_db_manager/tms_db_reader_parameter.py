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
from tms_msg_db.srv import TmsdbGetParameter


class TmsDbReaderParameter(Node):

    def __init__(self):
        super().__init__('tms_db_reader_parameter')

        # Declare parameters
        self.declare_parameter('db_host', 'localhost')
        self.declare_parameter('db_port', 27017)

        # Get parameters
        self.db_host: str = self.get_parameter('db_host').get_parameter_value().string_value
        self.db_port: int = self.get_parameter('db_port').get_parameter_value().integer_value

        self.db: pymongo.database.Database  = db_util.connect_db('rostmsdb', self.db_host, self.db_port)
        self.srv = self.create_service(TmsdbGetParameter, 'tms_db_reader_parameter', self.db_reader_srv_callback)


    def db_reader_srv_callback(self, request, response):

        collection: pymongo.collection.Collection = self.db["parameter"]
        response = self.get_parameter_data(request.model_name, request.record_name, collection)
        self.get_logger().info(f"Response Data: keys={response.keys}, values={response.values}")
        return response
        
    
    def get_parameter_data(self, model_name, record_name, collection: pymongo.collection.Collection) -> str:

        filter_builder = {
            "model_name": model_name,
            "record_name": record_name
        }
        parameter = collection.find_one(filter_builder)

        if parameter == None:
            self.get_logger().info(f"The parameter ID(Model_name:{model_name} , Record name: {record_name}) does not exist under the default collection in the rostmsdb database")
            return ''
        
        for field in ["_id", "model_name", "type", "record_name"]:
            if field in parameter:
                del parameter[field]

        response = TmsdbGetParameter.Response()
        response.keys = []
        response.values = []

        for key, value in parameter.items():
            if isinstance(value, (list, tuple)):
                response.keys.extend([key] * len(value))
                response.values.extend(map(str, value))
            else:
                response.keys.append(key)
                response.values.append(str(value))

        self.get_logger().info(f"Successfully retrieved the parameter (Model_name:{model_name} , Record name: {record_name})")
        return response


def main(args=None):
    rclpy.init(args=args)
    tms_db_reader_parameter = TmsDbReaderParameter()
    rclpy.spin(tms_db_reader_parameter)
    tms_db_reader_parameter.destroy_node()
    rclpy.shutdown()


if __name__=='__main__':
    main()