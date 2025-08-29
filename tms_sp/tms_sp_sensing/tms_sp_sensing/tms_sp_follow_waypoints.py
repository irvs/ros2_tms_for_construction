#!/usr/bin/env python3
# Copyright 2023, IRVS Laboratory, Kyushu University, Japan.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from rclpy.node import Node
from pymongo import MongoClient
from sensing_msgs.msg import FollowWaypoints

# MongoDB connection settings
MONGODB_IPADDRESS = '127.0.0.1'
MONGODB_PORTNUMBER = 27017

class UpdateDB_Parameter(Node):
    def __init__(self):
        super().__init__("tms_sp_follow_waypoints")
        self.subscription = self.create_subscription(
            FollowWaypoints,
            '/tms_sp_follow_waypoints',
            self.update_db_parameter,
            10
        )

    def update_db_parameter(self, msg: FollowWaypoints) -> None:
        # Connect to MongoDB
        client = MongoClient(MONGODB_IPADDRESS, MONGODB_PORTNUMBER)
        db = client['rostmsdb']
        collection = db['parameter']

        # Query for existing parameter document
        query = {
            "model_name": msg.model_name,
            "type": "dynamic",
            "record_name": msg.record_name
        }
        parameter_info = collection.find_one(query)

        # If no document found, warn and skip
        if parameter_info is None:
            self.get_logger().warn(
                f"[UpdateDB_Parameter] No parameter document found for query: {query}"
            )
            return

        # Prepare update fields
        update_parameter_info = {}
        msg_fields = msg.get_fields_and_field_types().keys()

        for key, value in parameter_info.items():
            # Skip metadata fields
            if key in ("_id", "model_name", "type", "record_name", "description"):
                continue

            # If message contains this field, use its value (convert arrays to lists)
            lower_key = key.lower()
            if lower_key in msg_fields:
                raw = getattr(msg, lower_key)
                # Convert array.array to list for BSON encoding
                update_parameter_info[key] = list(raw) if hasattr(raw, '__iter__') else raw
            else:
                # Preserve existing value if not in message
                update_parameter_info[key] = value

        # Perform update
        update_query = {"$set": update_parameter_info}
        collection.update_one(query, update_query)


def main(args=None):
    rclpy.init(args=args)
    node = UpdateDB_Parameter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
