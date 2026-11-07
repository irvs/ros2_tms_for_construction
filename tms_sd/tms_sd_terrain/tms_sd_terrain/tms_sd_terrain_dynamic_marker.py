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

import rclpy
from rclpy.node import Node

from visualization_msgs.msg import Marker
from tms_msg_db.msg import TmsdbGridFS


NODE_NAME = 'tms_sd_terrain_dynamic_marker'
DATA_ID   = 3031
DATA_TYPE = 'dynamic_marker'


class TmsSdTerrainDynamicMarker(Node):
    """Convert Marker msg to TmsdbGridFS msg sent to tms_db_writer_gridfs."""

    def __init__(self):
        super().__init__(NODE_NAME)

        # Declare parameters
        self.declare_parameter('terrain_name', 'terrain_name')

        # Get parameters
        self.terrain_name = (
            self.get_parameter('terrain_name')
            .get_parameter_value()
            .string_value
        )

        # Publisher
        self.publisher_ = self.create_publisher(
            TmsdbGridFS,
            'tms_db_gridfs_data',
            10
        )

        # Subscriber
        self.subscription = self.create_subscription(
            Marker,
            '~/input/terrain/dynamic/marker',
            self.send_marker_to_db_writer,
            10
        )

    def send_marker_to_db_writer(self, msg: Marker) -> None:
        """
        Send Marker msg to tms_db_writer_gridfs.

        Parameters
        ----------
        msg : Marker
            Target Object's Marker.
        """

        tmsdb_gridfs_msg = TmsdbGridFS()

        tmsdb_gridfs_msg.time = datetime.now().isoformat()
        tmsdb_gridfs_msg.type = DATA_TYPE
        tmsdb_gridfs_msg.id = DATA_ID

        # Set Marker data
        tmsdb_gridfs_msg.marker = msg

        self.publisher_.publish(tmsdb_gridfs_msg)

    def destroy_node(self):
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    node = TmsSdTerrainDynamicMarker()

    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()