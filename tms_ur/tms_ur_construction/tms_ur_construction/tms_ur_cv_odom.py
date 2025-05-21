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
from functools import partial
import json
from time import sleep
import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from tms_msg_db.srv import TmsdbGetData

import tms_db_manager.tms_db_util as db_util


NODE_NAME = "tms_ur_cv_odom"
DATA_ID = 3012#2012
DATA_TYPE = "machine_pose"


class TmsUrCvOdomNode(Node):
    """Get construction vehicle's Odometry data from tms_db_reader."""

    def __init__(self):
        super().__init__(NODE_NAME)

        # Declare parameters
        self.declare_parameter("latest", False)
        self.declare_parameter("machine_name", "machine_name")

        # Get parameters
        self.latest: bool = (
            self.get_parameter("latest").get_parameter_value().bool_value
        )
        self.machine_name: str = (
            self.get_parameter("machine_name").get_parameter_value().string_value
        )

        self.publisher_ = self.create_publisher(Odometry, "~/output/odom", 10)

        self.cli = self.create_client(TmsdbGetData, "tms_db_reader")
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("service not available, waiting again...")

        timer_period = 0.1
        self.call_timer = self.create_timer(timer_period, self.send_request)

    def send_request(self):
        """
        Send request to tms_db_reader to get Odometry data.
        """
        self.req = TmsdbGetData.Request()
        self.req.type = DATA_TYPE
        self.req.id = DATA_ID
        self.req.latest_only = self.latest
        self.req.name = self.machine_name

        future = self.cli.call_async(self.req)
        future.add_done_callback(partial(self.callback_response))

    def callback_response(self, future):
        """
        Set response from tms_db_reader.
        """
        try:
            self.res = future.result()
            self.tmsdbs = self.res.tmsdbs
            self.publish_odom()
        except:
            return

    def publish_odom(self) -> None:
        """
        Publish Odometry topics.
        """
        if self.latest:
            try:
                dict_msg: dict = json.loads(self.tmsdbs[0].msg)
            except:
                return

            msg: Odometry = db_util.document_to_msg(dict_msg, Odometry)
            self.publisher_.publish(msg)
        else:
            try:
                pre_time = datetime.strptime(
                    self.tmsdbs[0].time, "%Y-%m-%dT%H:%M:%S.%f"
                )
            except:
                return

            for tmsdb in self.tmsdbs:
                dict_msg: dict = json.loads(tmsdb.msg)
                msg: Odometry = db_util.document_to_msg(dict_msg, Odometry)

                # Convert string to datetime
                now_time = datetime.strptime(tmsdb.time, "%Y-%m-%dT%H:%M:%S.%f")

                # Time delta
                td = now_time - pre_time
                sleep(td.total_seconds())
                pre_time = now_time

                self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    tms_ur_cv_odom_node = TmsUrCvOdomNode()
    rclpy.spin(tms_ur_cv_odom_node)

    tms_ur_cv_odom_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
