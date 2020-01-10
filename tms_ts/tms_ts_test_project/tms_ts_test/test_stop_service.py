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
from threading import Event
import asyncio

class WaitNode(Node):
    def __init__(self):
        super().__init__('wait')
        self.cb_group = ReentrantCallbackGroup()
        self.srv = self.create_service(AddTwoInts, 'wait', self.wait_callback, callback_group=self.cb_group)
    
    def wait_callback(self, request, response):
        time.sleep(request.a)
        return response   

class MinimalService(Node):

    def __init__(self, loop):
        super().__init__('minimal_service')
        self.loop = loop
        self.cb_group = ReentrantCallbackGroup()
        self.sub = self.create_subscription(String, 'topic', self.stop_callback, callback_group=self.cb_group)
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback, callback_group=self.cb_group)

    async def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info('Incoming request\na: %d b: %d' % (request.a, request.b))
        self.cli = self.create_client(AddTwoInts, 'wait', callback_group=self.cb_group)
        req = AddTwoInts.Request()
        req.a = 10
        self.future = self.cli.call_async(req)
        self.flag_stoptask = False
        
        while not self.flag_stoptask and not self.future.done():
            pass
        # _count = 0
        # asyncio.set_event_loop(self.loop)
        # while not self.future.done() or self.flag_stoptask:
        #     try: 
        #         #await asyncio.wait_for(asyncio.shield(self.future), timeout=1.0)
        #         pass
        #     except asyncio.TimeoutError:
        #         print(f'timeout: {_count}')
        if self.flag_stoptask:
            print('stop!')
            response.sum = 404
        else:
            print('complete!')
            response.sum = 1
        return response

    def stop_callback(self, msg):
        # stop service
        self.get_logger().info("stop!!")
        self.flag_stoptask = True


def main(args=None):
    rclpy.init(args=args)
    loop = asyncio.new_event_loop()
    try:
        minimal_service = MinimalService(loop)
        wait_node = WaitNode()

        executor = MultiThreadedExecutor()
        # executor = CustomThreadedExecutor(loop)
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