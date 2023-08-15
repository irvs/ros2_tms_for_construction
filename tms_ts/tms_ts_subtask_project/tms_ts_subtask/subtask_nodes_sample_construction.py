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
from tms_ts_subtask.subtask_node_base import SubtaskNodeBase
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
import math
import numpy as np
import time
from tms_msg_ur.srv import SpeakerSrv
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from std_msgs.msg import String, Float64
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import sys

def main(args=None):
    global executor
    rclpy.init(args=args)
    try:
        executor = MultiThreadedExecutor()
        executor.add_node(SubtaskControlZx120Boom())
        executor.add_node(SubtaskControlZx120Swing())
        executor.add_node(SubtaskControlZx120Arm())
        executor.add_node(SubtaskControlZx120Bucket())
        
        try:
            executor.spin()
        finally:
            executor.shutdown()
    finally:
        rclpy.shutdown()

if __name__=='__main__':
    main()

"""class SubtaskSpeakerAnnounce(SubtaskNodeBase):
    def node_name(self):
        return "subtask_speaker_announce"
    
    def id(self):
        return 9300
    
    async def service_callback(self, request, response, goal_handle):
        self.get_logger().info(f'{request["announce"]}')
        self.cli = self.create_client(SpeakerSrv, "speaker_srv", callback_group=ReentrantCallbackGroup())
        req = SpeakerSrv.Request()
        req.data = request["announce"]
        await self.cli.call_async(req)
        response.message = "Success"
        return response
    
    def init_argument(self):
        return {"announce" : "よくわかりませんでした"}"""

class SubtaskControlZx120Boom(SubtaskNodeBase):
    def node_name(self):
        return "subtask_zx120_boom_sample"
    
    def id(self):
        return 8001
    
    async def service_callback(self, request, response, goal_handle):
        #boom joint (degrees : -70 to 44)
        #pitch角は値が小さくなるほどzx120の上方向に持ち上がる
        #初期姿勢で関節角を20度以上にしてしまうとバケットが地面に付き、zx120自体を持ち上げてしまうので注意
        self.publisher = self.create_publisher(Float64, '/zx120/boom/cmd', 10)
        self.msg = Float64()
        self.initial_position = float(request["boom_joint"][0])  #max:44[deg]
        self.target_position = float(request["boom_joint"][1])  #min:-70[deg]
        self.deg = self.initial_position
        self.response = response
        self.timer = self.create_timer(0.5, self.timer_callback)
        while not self.timer.is_canceled():
            pass
        self.get_logger().info("Complete boom modification")
        response.message = "Success"
        return response
    
    async def timer_callback(self):
        self.deg += self.target_position/20
        self.msg.data = math.radians(self.deg)
        self.publisher.publish(self.msg)
        self.get_logger().info("boom position : %f [deg]" % self.deg)
        if self.deg <= self.target_position:
            self.timer.cancel()
            
    

class SubtaskControlZx120Swing(SubtaskNodeBase):
    def node_name(self):
        return "subtask_zx120_swing_sample"
    
    def id(self):
        return 8002
    
    async def service_callback(self, request, response, goal_handle):
        #swing joint (degrees: continuous)
        self.publisher = self.create_publisher(Float64, '/zx120/swing/cmd', 10)
        self.msg = Float64()
        self.initial_position = request["swing_joint"][0]  #max: continuous
        self.target_position = request["swing_joint"][1]  #min: continuous
        self.deg = self.initial_position
        #self.response = response
        self.timer = self.create_timer(0.5, self.timer_callback)
        while not self.timer.is_canceled():
            pass
        self.get_logger().info("Complete swing modification")
        response.message = "Success"
        return response
    
    def timer_callback(self):
        self.deg += self.target_position/20
        self.msg.data = math.radians(self.deg)
        self.publisher.publish(self.msg)
        self.get_logger().info("swing position : %f [deg]" % self.deg)
        if self.deg >= self.target_position:
            self.timer.cancel()
            #self.get_logger().info("Complete swing modification")
            

    

class SubtaskControlZx120Arm(SubtaskNodeBase):
    def node_name(self):
        return "subtask_zx120_arm_sample"
    
    def id(self):
        return 8003
    
    async def service_callback(self, request, response, goal_handle):
    #arm joint (degrees : 30 to 152)
        self.publisher = self.create_publisher(Float64, '/zx120/arm/cmd', 10)
        self.msg = Float64()
        self.initial_position = request["arm_joint"][0]  #min:30[deg]
        self.target_position = request["arm_joint"][1]  #max:152[deg]
        self.deg = self.initial_position
        self.response = response
        self.timer = self.create_timer(0.5, self.timer_callback)
        while not self.timer.is_canceled():
            pass
        self.get_logger().info("Complete arm modification")
        response.message = "Success"
        return response

    
    def timer_callback(self):
        self.deg -= self.target_position/5
        self.msg.data = math.radians(self.deg)
        self.publisher.publish(self.msg)
        self.get_logger().info("arm position : %f [deg]" % self.deg)
        if self.target_position <= self.deg:
            self.timer.cancel()
            

class SubtaskControlZx120Bucket(SubtaskNodeBase):
    def node_name(self):
        return "subtask_zx120_bucket_sample"
    
    def id(self):
        return 8004
    
    async def service_callback(self, request, response, goal_handle):
        #bucket joint (degrees : -33 to 143)
        self.publisher = self.create_publisher(Float64, '/zx120/bucket/cmd', 10)
        self.msg = Float64()
        self.initial_position = request["bucket_joint"][0]  #min:-33[deg]
        self.target_position = request["bucket_joint"][1]  #max:143[deg]
        self.deg = self.initial_position
        self.response = response
        self.timer = self.create_timer(0.5, self.timer_callback)
        while not self.timer.is_canceled():
            pass
        self.get_logger().info("Complete bucket modification")
        response.message = "Success"
        return response
    
    def timer_callback(self):
        self.deg += self.target_position/2
        self.msg.data = math.radians(self.deg)
        self.publisher.publish(self.msg)
        self.get_logger().info("bucket position : %f [deg]" % self.deg)
        if self.target_position >= self.deg:
            self.timer.cancel()
            
            

class SubtaskControlIc120Clawler(SubtaskNodeBase):
    def node_name(self):
        return "subtask_ic120_crawler_sample"
    
    def id(self):
        return 8005
    
    async def service_callback(self, request, response, goal_handle):
        #bucket joint (degrees : -33 to 143)
        self.publisher = self.create_publisher(Float64, '/zx120/bucket/cmd', 10)
        self.msg = Float64()
        self.initial_position = 0  #min:-33[deg]
        self.target_position = 20  #max:143[deg]
        self.deg = self.initial_position
        self.response = response
        self.timer = self.create_timer(0.5, self.timer_callback)
    
    def timer_callback(self):
        self.deg += self.target_position/2
        self.msg.data = math.radians(self.deg)
        self.publisher.publish(self.msg)
        self.get_logger().info("bucket position : %f [deg]" % self.deg)
        if self.target_position <= self.deg:
            self.timer.cancel()
            self.get_logger().info("Complete bucket modification")
            self.response.message = "Success"
            return self.response