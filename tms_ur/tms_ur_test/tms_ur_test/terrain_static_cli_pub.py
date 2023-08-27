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

from sensor_msgs.msg import PointCloud2
from tms_msg_db.srv import TerrainStaticSrv
import rclpy
from rclpy.node import Node


class MinimalClientAsync(Node):

    def __init__(self):
        super().__init__('terrain_static_cli')
        self.cli = self.create_client(TerrainStaticSrv, '/tms_ur_construction_terrain_static/output/terrain/static_srv')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = TerrainStaticSrv.Request()

    def send_request(self):
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()


class MinimalPublisher(Node):

    def __init__(self, pointcloud2: PointCloud2):
        super().__init__('terrain_static_pub')
        self.publisher_ = self.create_publisher(PointCloud2, '/output/terrain/static/demo', 10)
        timer_period = 5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.pointcloud2 = pointcloud2

    def timer_callback(self):
        msg = self.pointcloud2
        self.publisher_.publish(msg)
        self.get_logger().info("Publishing")


def main(args=None):
    rclpy.init(args=args)

    minimal_client = MinimalClientAsync()
    response = minimal_client.send_request()

    minimal_client.destroy_node()
    rclpy.shutdown()

    rclpy.init()
    publisher = MinimalPublisher(response.pointcloud2)
    rclpy.spin(publisher)
    publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()