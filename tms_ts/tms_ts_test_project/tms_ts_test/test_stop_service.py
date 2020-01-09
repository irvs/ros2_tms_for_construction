# Copyright 2016 Open Source Robotics Foundation, Inc.
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

from example_interfaces.srv import AddTwoInts

import rclpy
from rclpy.node import Node
import time
from std_msgs.msg import String
from rclpy.callback_groups import ReentrantCallbackGroup
import threading
from rclpy.executors import MultiThreadedExecutor

class WaitNode(Node):
    def __init__(self):
        super().__init__('wait')
        self.cb_group = ReentrantCallbackGroup()
        self.srv = self.create_service(AddTwoInts, 'wait', self.wait_callback, callback_group=self.cb_group)
    
    def wait_callback(self, request, response):
        time.sleep(request.a)
        return response

class MinimalService(Node):

    def __init__(self):
        super().__init__('minimal_service')
        self.cb_group = ReentrantCallbackGroup()
        self.sub = self.create_subscription(String, 'topic', self.stop_callback, callback_group=self.cb_group)
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback, callback_group=self.cb_group)

    async def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info('Incoming request\na: %d b: %d' % (request.a, request.b))
        self.cli = self.create_client(AddTwoInts, 'wait', callback_group=self.cb_group)
        req = AddTwoInts.Request()
        req.a = 10
        await self.cli.call_async(req)
        # time.sleep(10.0) # sleep for 10 seconds
        return response

    def stop_callback(self, msg):
        # stop service
        self.get_logger().info("stop!!")
        return


def main(args=None):
    rclpy.init(args=args)
    try:
        minimal_service = MinimalService()
        wait_node = WaitNode()

        executor = MultiThreadedExecutor()
        executor.add_node(minimal_service)
        executor.add_node(wait_node)
        try:
            executor.spin()
        finally:
            executor.shutdown()
            minimal_service.destroy_node()
            wait_node.destroy_node()
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()