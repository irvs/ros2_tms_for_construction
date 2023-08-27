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
from tms_msg_db.srv import TmsdbGetData


class TmsDbReader(Node):
    """Read data from MongoDB."""

    def __init__(self):
        super().__init__("tms_db_reader")

        # Declare parameters
        self.declare_parameter("db_host", "localhost")
        self.declare_parameter("db_port", 27017)

        # Get parameters
        self.db_host: str = (
            self.get_parameter("db_host").get_parameter_value().string_value
        )
        self.db_port: int = (
            self.get_parameter("db_port").get_parameter_value().integer_value
        )

        self.db: pymongo.database.Database = db_util.connect_db(
            "rostmsdb", self.db_host, self.db_port
        )
        self.srv = self.create_service(
            TmsdbGetData, "tms_db_reader", self.db_reader_srv_callback
        )

    def db_reader_srv_callback(self, request, response):
        """
        Respond to requests from client nodes.

        Parameters
        ----------
        request
            Request from client node.
        response
            Response to client node.

        Returns
        -------
        response
            Response to client node.
        """
        collection: pymongo.collection.Collection = self.db[request.type]

        if request.latest_only:
            latest_data: dict = self.get_latest_data(request, collection)
            if latest_data == None:
                return response
            response.tmsdbs.append(self.allocate_tmsdb(latest_data))
            return response
        else:
            all_data: pymongo.cursor.Cursor = self.get_all_data(request, collection)
            for data in all_data:
                msg: Tmsdb = self.allocate_tmsdb(data)
                response.tmsdbs.append(msg)
            return response

    def get_latest_data(
        self, request, collection: pymongo.collection.Collection
    ) -> dict:
        """
        Get latest data only.

        Parameters
        ----------
        request
            Request from a client node.
        collection
            MongoDB's target collection.

        Returns
        -------
        dict
            Requested latest data.
        """
        if request.name != "":
            latest_data: dict = collection.find_one(
                {"id": request.id, "name": request.name},
                sort=[("time", pymongo.DESCENDING)],
            )
        else:
            latest_data: dict = collection.find_one(
                {"id": request.id}, sort=[("time", pymongo.DESCENDING)]
            )

        return latest_data

    def get_all_data(self, request, collection) -> pymongo.cursor.Cursor:
        """
        Get all data.

        Parameters
        ----------
        request
            Request from a client node.
        collection
            MongoDB's target collection.

        Returns
        -------
        pymongo.cursor.Cursor
            Requested all data.
        """
        if request.name != "":
            all_data: pymongo.cursor.Cursor = collection.find(
                {"id": request.id, "name": request.name}
            ).sort([("time", pymongo.ASCENDING)])
        else:
            all_data: pymongo.cursor.Cursor = collection.find({"id": request.id}).sort(
                [("time", pymongo.ASCENDING)]
            )
        return all_data

    def allocate_tmsdb(self, data: dict) -> Tmsdb:
        """
        Allocate dictionary data to Tmsdb msg.

        Parameters
        ----------
        dict : data
            Dictionary data.

        Returns
        -------
        Tmsdb
            Tmsdb msg data.
        """
        tmsdb = Tmsdb()
        tmsdb.time = data["time"]
        tmsdb.type = data["type"]
        tmsdb.id = data["id"]
        tmsdb.name = data["name"]
        tmsdb.msg = json.dumps(data["msg"])
        return tmsdb


def main(args=None):
    rclpy.init(args=args)
    tms_db_reader = TmsDbReader()
    rclpy.spin(tms_db_reader)

    tms_db_reader.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
