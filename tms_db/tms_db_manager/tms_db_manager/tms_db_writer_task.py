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
from tms_msg_db.msg import Tmsdb


class TmsDbWriterTask(Node):
    """Write task data to MongoDB."""

    def __init__(self):
        super().__init__("tms_db_writer_task")

        # Declare parameters
        self.declare_parameter("db_host", "localhost")
        self.declare_parameter("db_port", 27017)
        self.declare_parameter("init_db", False)

        # Get parameters
        self.db_host: str = (
            self.get_parameter("db_host").get_parameter_value().string_value
        )
        self.db_port: int = (
            self.get_parameter("db_port").get_parameter_value().integer_value
        )

        self.db = db_util.connect_db("rostmsdb", self.db_host, self.db_port)
        self.subscription = self.create_subscription(
            Tmsdb, "tms_db_data", self.db_write_callback, 10
        )

    def db_write_callback(self, msg: Tmsdb) -> None:
        """
        Store data.

        Parameters
        ----------
        msg : Tmsdb
            An instance of a ROS2 custom message to store data.
        """
        doc: dict = db_util.msg_to_document(msg)

        # Convert json to dictionary
        doc["msg"] = json.loads(msg.msg)

        collection: pymongo.collection.Collection = self.db['now']
        collection.update_one(
            {"id": doc["id"]},
            {"$set": doc},
            upsert=True
        )

        self.get_logger().info(f"Updated {doc['name']} (ID: {doc['id']}).")


def main(args=None):
    rclpy.init(args=args)
    tms_db_writer_task = TmsDbWriterTask()
    rclpy.spin(tms_db_writer_task)
    tms_db_writer_task.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()