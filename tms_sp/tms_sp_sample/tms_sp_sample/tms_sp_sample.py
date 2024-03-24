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
from tms_msg_db.msg import Tmsdb

import tms_db_manager.tms_db_util as db_util

# TODO: Change the node name, data ID, data type, and data name.
NODE_NAME = 'tms_sp_sample'
DATA_ID = 7000
DATA_TYPE = 'data_type' 
DATA_NAME = 'data_name'

class TmsSpSample(Node):
    """Sample node for tms_sp."""

    def __init__(self):
        super().__init__(NODE_NAME)

        self.publisher_ = self.create_publisher(Tmsdb, 'tms_db_data', 10)
        self.subscription = self.create_subscription(
            Odometry, # TODO: Change the msg to the one you want to use.
            '/cd_0/status/odom', # TODO: Change the topic name to the one you want to use.
            self.send_data_to_db_writer,
            10)

    def send_data_to_db_writer(self, msg: Odometry) -> None:
        """
        Send topics to tms_db_writer (Write the received data to DB).

        parameters
        ----------
        msg : Odometry (TODO: Change the msg to the one you want to use.)
            Target Object's Odometry.
        """
        db_msg = self.create_db_msg(msg)
        self.publisher_.publish(db_msg)

    def create_db_msg(self, msg: Odometry) -> Tmsdb:
        """
        Create Tmsdb msg from Odometry msg.

        Parameters
        ----------
        msg : Odometry (TODO: Change the msg to the one you want to use.)
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
        tms_db_msg.name = DATA_NAME

        # Convert msg to dictionary and then to json.
        doc: dict = db_util.msg_to_document(msg)
        tms_db_msg.msg: str = json.dumps(doc)

        return tms_db_msg


def main(args=None):
    rclpy.init(args=args)

    tms_sp_machine_odom = TmsSpSample()

    rclpy.spin(tms_sp_machine_odom)

    tms_sp_machine_odom.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()