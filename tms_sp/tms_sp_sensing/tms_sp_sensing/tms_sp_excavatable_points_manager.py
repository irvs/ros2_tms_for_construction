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

"""
Publishes the "excavatable_points" candidates (x, y, z, theta_w) stored
under rostmsdb.parameter, and writes back the single point chosen by an
external system as the excavation target.

Source document shape (rostmsdb.parameter):
{
  "record_name": "target_excavate_pose_2",
  "excavatable_points": [
    {"x": -1.5, "y": -6.5, "z": -2.5, "theta_w": 1},
    ...
  ]
}

Target document shape (rostmsdb.parameter) that gets overwritten when a
point is chosen (model_name/record_name come from the Zx200EndEffector
message itself):
{
  "model_name": "zx200",
  "record_name": "target_excavate_pose_1",
  "waypoints": [
    {
      "type": "pose",
      "data": {"x": 1, "y": -7.5, "z": -1.5, "theta_w": 1}
    }
  ]
}
"""

import rclpy
from rclpy.node import Node

from pymongo import MongoClient
from std_srvs.srv import Trigger
from sensing_msgs.msg import TargetPoint, TargetPointArray, Zx200EndEffector

MONGODB_IPADDRESS = '127.0.0.1'
MONGODB_PORTNUMBER = 27017

SOURCE_RECORD_NAME = 'target_excavate_pose_2'


class ExcavatablePointsManager(Node):
    def __init__(self):
        super().__init__("tms_sp_excavatable_points_manager")

        self.publisher = self.create_publisher(
            TargetPointArray,
            'excavatable_points',
            10)

        self.srv = self.create_service(
            Trigger,
            'get_excavatable_points',
            self.get_excavatable_points_callback)

        self.subscription = self.create_subscription(
            Zx200EndEffector,
            'zx200/end_effector',
            self.update_excavate_target,
            10)
        
        self.get_logger().info("ExcavatablePointsManager node has been started.")

    def get_excavatable_points_callback(self, request, response):

        self.get_logger().info("Received request to get excavatable points.")

        client = MongoClient(MONGODB_IPADDRESS, MONGODB_PORTNUMBER)
        db = client['rostmsdb']
        collection = db['parameter']

        query = {"record_name": SOURCE_RECORD_NAME}
        parameter_info = collection.find_one(query)

        if parameter_info is None:
            response.success = False
            response.message = f"No document found for record_name='{SOURCE_RECORD_NAME}'"
            self.get_logger().warn(response.message)
            return response

        excavatable_points = parameter_info.get('excavatable_points', [])

        msg = TargetPointArray()
        msg.points = [
            TargetPoint(
                x=float(point.get('x', 0.0)),
                y=float(point.get('y', 0.0)),
                z=float(point.get('z', 0.0)),
                theta_w=float(point.get('theta_w', 0.0)))
            for point in excavatable_points
        ]
        self.publisher.publish(msg)

        response.success = True
        response.message = f"Published {len(msg.points)} excavatable_points"
        self.get_logger().info(response.message)
        return response

    def update_excavate_target(self, msg: Zx200EndEffector) -> None:

        self.get_logger().info(
            f"Received excavate point: model_name={msg.model_name}, record_name={msg.record_name}, "
            f"x={msg.x}, y={msg.y}, z={msg.z}, theta_w={msg.theta_w}")

        client = MongoClient(MONGODB_IPADDRESS, MONGODB_PORTNUMBER)
        db = client['rostmsdb']
        collection = db['parameter']

        # query = {"record_name": msg.record_name, "model_name": msg.model_name}
        query = {"record_name": "target_excavate_pose_1", "model_name": "zx200"}
        update_query = {
            "$set": {
                "waypoints.0.data": {
                    "x": float(msg.x),
                    "y": float(msg.y),
                    "z": float(msg.z),
                    "theta_w": float(msg.theta_w),
                }
            }
        }
        result = collection.update_one(query, update_query)

        if result.matched_count > 0:
            self.get_logger().info(
                f"Updated waypoints[0].data for record_name='{msg.record_name}', "
                f"model_name='{msg.model_name}': "
                f"x={msg.x}, y={msg.y}, z={msg.z}, theta_w={msg.theta_w}")
        else:
            self.get_logger().warn(
                f"No matching record found for record_name='{msg.record_name}', "
                f"model_name='{msg.model_name}'")


def main(args=None):
    rclpy.init(args=args)
    excavatable_points_manager = ExcavatablePointsManager()
    rclpy.spin(excavatable_points_manager)
    excavatable_points_manager.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
