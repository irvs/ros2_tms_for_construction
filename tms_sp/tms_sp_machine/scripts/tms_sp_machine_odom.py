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

from datetime import datetime
import json
import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_geometry_msgs import do_transform_pose

from tms_msg_db.msg import Tmsdb
import tms_db_manager.tms_db_util as db_util


NODE_NAME = "tms_sp_machine_odom"
DATA_ID = 2012
DATA_TYPE = "machine"


class TmsSpMachineOdom(Node):
    """Convert Odometry msg to Tmsdb msg and sent to tms_db_writer."""

    def __init__(self):
        super().__init__(NODE_NAME)

        # Declare parameters
        self.declare_parameter("machine_name", "machine_name")
        self.declare_parameter("to_frame", "world")

        # Get parameters
        self.machine_name: str = (
            self.get_parameter("machine_name").get_parameter_value().string_value
        )
        self.to_frame: str = (
            self.get_parameter("to_frame").get_parameter_value().string_value
        )

        self.publisher_ = self.create_publisher(Tmsdb, "tms_db_data", 10)
        self.subscription = self.create_subscription(
            Odometry, "~/input/odom", self.send_odom_to_db_writer, 10
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.is_excepted = False

        # This publisher is to check the transformation.
        self.publisher = self.create_publisher(
            Odometry, f"tf_machine/{self.machine_name}", 10
        )

        self.is_received = False

    def send_odom_to_db_writer(self, msg: Odometry) -> None:
        """
        Send topics to tms_db_writer (Write the received Odometry data to DB).

        parameters
        ----------
        msg : Odometry
            Target Object's Odometry.
        """
        # Log
        if not self.is_received:
            self.get_logger().info(f"Received {self.machine_name}'s Odometry msg")
            self.is_received = True

       # self.get_logger().info(msg.pose.pose.position)

        # Transform
       # msg = self.transform(msg)

        db_msg = self.create_db_msg(msg)
        self.publisher_.publish(db_msg)

    def transform(self, msg: Odometry) -> Odometry:
        """
        Transform Odometry msg to the specified frame.

        Parameters
        ----------
        msg : Odometry
            Target Machine's Odometry msg.

        Returns
        -------
        msg : Odometry
            Transformed Odometry msg.
        """
        prev_pose = msg.pose.pose

        from_frame = msg.header.frame_id

        try:
            tf = self.tf_buffer.lookup_transform(
                self.to_frame, from_frame, rclpy.time.Time()
            )
            if self.is_excepted:
                self.get_logger().info("Transformation is available.")
                self.is_excepted = False
        except TransformException as e:
            if not self.is_excepted:
                self.get_logger().info("There is no transformation.")
                self.get_logger().warning(str(e))
                self.is_excepted = True
            return msg

        msg.header.frame_id = self.to_frame

        # Apply translation
        transformed_pose = do_transform_pose(prev_pose, tf)
        msg.pose.pose = transformed_pose

        self.publisher.publish(msg)

        return msg

    def create_db_msg(self, msg: Odometry) -> Tmsdb:
        """
        Create Tmsdb msg from Odometry msg.

        Parameters
        ----------
        msg : Odometry
            Target Machine's Odometry msg.

        Returns
        -------
        tms_db_msg : Tmsdb
            Message containing Odometry msg sent to tms_db_writer.
        """
        tms_db_msg = Tmsdb()
        tms_db_msg.time = datetime.now().isoformat()
        tms_db_msg.type = DATA_TYPE
        tms_db_msg.id = DATA_ID
        tms_db_msg.name = self.machine_name

        # Convert Odometry msg to dictionary and then to json.
        doc: dict = db_util.msg_to_document(msg)
        tms_db_msg.msg: str = json.dumps(doc)

        return tms_db_msg


def main(args=None):
    rclpy.init(args=args)

    tms_sp_machine_odom = TmsSpMachineOdom()

    rclpy.spin(tms_sp_machine_odom)

    tms_sp_machine_odom.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
