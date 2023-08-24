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

from sensor_msgs.msg import CompressedImage
from tms_msg_db.srv import TmsdbGetData

import tms_db_manager.tms_db_util as db_util


NODE_NAME = "tms_ur_construction_theta_compressed"
DATA_ID = 3040
DATA_TYPE = "theta"


class TmsUrConstructionThetaCompressedNode(Node):
    def __init__(self):
        super().__init__(NODE_NAME)

        # Declare parameters
        self.declare_parameter("theta_name", "theta_name")
        self.declare_parameter("latest", False)

        # Get parameters
        self.theta_name: str = (
            self.get_parameter("theta_name").get_parameter_value().string_value
        )
        self.latest: bool = (
            self.get_parameter("latest").get_parameter_value().bool_value
        )

        self.publisher_ = self.create_publisher(
            CompressedImage, "~/output/theta/compressed", 10
        )

        self.cli = self.create_client(TmsdbGetData, "tms_db_reader")
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("service not available, waiting again...")

        timer_period = 0.1
        self.call_timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        """
        Get theta data from tms_db_reader and publish them.
        """
        self.send_request()

        try:
            self.tmsdbs = self.res.tmsdbs
        except:
            return

        self.publish_theta()

    def send_request(self):
        """
        Send request to tms_db_reader to get theta data.

        Returns
        -------
        Any
            Result of request to tms_db_reader.
        """
        self.req = TmsdbGetData.Request()
        self.req.type = DATA_TYPE
        self.req.id = DATA_ID
        self.req.latest_only = self.latest
        self.req.name = self.theta_name

        future = self.cli.call_async(self.req)
        future.add_done_callback(partial(self.callback_set_response))

    def callback_set_response(self, future):
        """
        Set response from tms_db_reader.

        Parameters
        ----------
        future : Any
            Result of request to tms_db_reader.
        """
        try:
            self.res = future.result()
        except Exception as e:
            self.get_logger().info("Service call failed %r" % (e,))

    def publish_theta(self) -> None:
        """
        Publish theta's CompressedImage topics.
        """
        if self.latest:
            try:
                dict_msg: dict = json.loads(self.tmsdbs[0].msg)
            except:
                # No theta data
                return

            # Convert dict_msg to CompressedImage
            msg: CompressedImage = db_util.document_to_msg(dict_msg, CompressedImage)
            self.publisher_.publish(msg)
        else:
            try:
                pre_time = datetime.strptime(
                    self.tmsdbs[0].time, "%Y-%m-%dT%H:%M:%S.%f"
                )
            except:
                # No theta data
                return

            for tmsdb in self.tmsdbs:
                dict_msg: dict = json.loads(tmsdb.msg)
                msg: CompressedImage = db_util.document_to_msg(
                    dict_msg, CompressedImage
                )

                # Convert string to datetime
                now_time = datetime.strptime(tmsdb.time, "%Y-%m-%dT%H:%M:%S.%f")

                # Time delta
                td = now_time - pre_time
                sleep(td.total_seconds())
                pre_time = now_time

                self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    tms_ur_construction_theta_compressed_node = TmsUrConstructionThetaCompressedNode()

    rclpy.spin(tms_ur_construction_theta_compressed_node)

    tms_ur_construction_theta_compressed_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
