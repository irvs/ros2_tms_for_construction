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

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2


NODE_NAME = 'tms_sd_terrain_static'
DATA_ID   = 3030 
DATA_TYPE = 'static'

class TmsSdTerrainStatic(Node):
    """Send PointCloud2 msg to tms_ss."""

    def __init__(self):
        super().__init__(NODE_NAME)
        
        self.publisher_ = self.create_publisher(PointCloud2, 'tms_sd_terrain_static', 10)
        self.subscription = self.create_subscription(
            PointCloud2,
            '~/input/terrain/static/pointcloud2',
            self.send_pointcloud_to_tms_ss,
            10)

    def send_pointcloud_to_tms_ss(self, msg: PointCloud2) -> None:
        """
        Send topics to tms_ss (create point cloud (.pcd) and mesh (.ply) file).

        Parameters
        ----------
        msg : PointCloud2
            Point cloud data.
        """
        self.get_logger().info('Static terrain info was received!')
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)

    tms_sd_terrain_static = TmsSdTerrainStatic()
    rclpy.spin(tms_sd_terrain_static)

    tms_sd_terrain_static.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
