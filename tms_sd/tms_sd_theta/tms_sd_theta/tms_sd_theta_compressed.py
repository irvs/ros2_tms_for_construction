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
from sensor_msgs.msg import CompressedImage

import tms_db_manager.tms_db_util as db_util
from tms_msg_db.msg import Tmsdb

NODE_NAME = "tms_sd_theta_compressed"
DATA_ID = 3040
DATA_TYPE = "theta"


class TmsSdThetaCompressed(Node):
    def __init__(self):
        super().__init__(NODE_NAME)

        # Declare parameters
        self.declare_parameter("theta_name", "theta_name")

        # Get parameters
        self.theta_name: str = (
            self.get_parameter("theta_name").get_parameter_value().string_value
        )

        self.publisher_ = self.create_publisher(Tmsdb, "tms_db_data", 10)
        self.subscription = self.create_subscription(
            CompressedImage,
            "~/input/theta/compressed",
            self.send_theta_to_db_writer,
            10,
        )

        self.is_received = False

    def send_theta_to_db_writer(self, msg: CompressedImage):
        """
        Send topics to tms_db_writer (Write the received CompressedImage data to DB).

        Parameters
        ----------
        msg : CompressedImage
            Target Object's CompressedImage.
        """
        # Log
        if not self.is_received:
            self.get_logger().info("Received theta data.")
            self.is_received = True

        db_msg: Tmsdb = self.create_db_msg(msg)
        self.publisher_.publish(db_msg)

    def create_db_msg(self, msg: CompressedImage) -> Tmsdb:
        tms_db_msg = Tmsdb()
        tms_db_msg.time = datetime.now().isoformat()
        tms_db_msg.type = DATA_TYPE
        tms_db_msg.id = DATA_ID
        tms_db_msg.name = self.theta_name

        # Convert CompressedImage to dictionary and then to json.
        doc: dict = db_util.msg_to_document(msg)
        tms_db_msg.msg: str = json.dumps(doc)

        return tms_db_msg


def main(args=None):
    rclpy.init(args=args)

    tms_sd_theta_compressed = TmsSdThetaCompressed()
    rclpy.spin(tms_sd_theta_compressed)

    tms_sd_theta_compressed.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
