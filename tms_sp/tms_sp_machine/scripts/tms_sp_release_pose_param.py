#!/usr/bin/env python3

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

from geometry_msgs.msg import Point

from tms_msg_db.msg import TmsdbQuery
from tms_msg_db.srv import TmsdbGetData

from scipy.spatial.transform import Rotation as R
import math

from functools import partial

NODE_NAME = "tms_sp_machine_odom"
DATA_ID = 3012 #2012
READ_DATA_TYPE = "parameter"
WRITE_DATA_TYPE = "pose_query"
WRITE_MODE="over_write"
RECORD_NAME="RELEASE_POINT_test" #放土位置 ####適宜変更
FLG_RECORD_NAME="SAMPLE_BLACKBOARD_SIPDEMO202508"#放土回数を管理するフラグ####適宜変更



class TmsSpMachineOdom(Node):
    """Convert Odometry msg to Tmsdb msg and sent to tms_db_writer."""

    def __init__(self):
        super().__init__(NODE_NAME)

        self.publisher_ = self.create_publisher(TmsdbQuery, "tms_db_param_data", 10)

        # Declare parameters
        self.declare_parameter("latest", False)
        self.declare_parameter("machine_name", "machine_name")
        

        # Get parameters
        self.machine_name: str = (
            self.get_parameter("machine_name").get_parameter_value().string_value
        )

        # Get parameters
        self.latest: bool = (
            self.get_parameter("latest").get_parameter_value().bool_value
        )


        self.cli = self.create_client(TmsdbGetData, "tms_db_param_reader")
        while not self.cli.wait_for_service(timeout_sec=0.5):
            self.get_logger().info("service not available, waiting again...")


        timer_period = 1
        self.call_timer = self.create_timer(timer_period, self.send_request_flg)
        self.release_counter = 0
        self.release_number = 0
        self.Rotation = [0.0, 0.0, 0.0, 0.0]
        self.initial_X = 0
        self.initial_Y = 0


    def send_request_flg(self):
        """
        Send request to tms_db_reader to get PoseStamped data.
        """
        self.req = TmsdbGetData.Request()
        self.req.type = READ_DATA_TYPE
        self.req.id = DATA_ID
        self.req.latest_only = self.latest
        self.req.flgorparam = "flg"
        self.req.name = FLG_RECORD_NAME

        future = self.cli.call_async(self.req)
        future.add_done_callback(partial(self.callback_flg_response))

    
    def callback_flg_response(self, future):
        res = future.result()
        if not res.tmsdbs:
            self.get_logger().warn("No data received.")
            return
        

        import json
        msg_data = json.loads(res.tmsdbs[0].msg)

        release_count_raw = msg_data["Release_Count"]

        if isinstance(release_count_raw, list):
            release_count = int(release_count_raw[0])
        else:
            release_count = int(release_count_raw)

        if self.release_counter != release_count: 
            self.release_counter = release_count
            self.release_number += 1
            self.get_logger().info("Counter is changed.")
            self.send_pose_request()

        else:
            return

    
    def send_pose_request(self):
        """
        Send request to tms_db_reader to get PoseStamped data.
        """
        self.req = TmsdbGetData.Request()
        self.req.type = READ_DATA_TYPE
        self.req.id = DATA_ID
        self.req.latest_only = self.latest
        self.req.flgorparam = "param"
        self.req.name = RECORD_NAME

        future = self.cli.call_async(self.req)
        future.add_done_callback(partial(self.callback_response))

    
    def callback_response(self, future):
        
        res = future.result()
        if not res.tmsdbs:
            self.get_logger().warn("No data received.")
            return

        import json
        msg_data = json.loads(res.tmsdbs[0].msg)

        x = float(msg_data["x"][0])
        y = float(msg_data["y"][0])
        z = 0.0
        qx = float(msg_data["qx"][0])
        qy = float(msg_data["qy"][0])
        qz = float(msg_data["qz"][0])
        qw = float(msg_data["qw"][0])
        if(self.release_number == 1):
            self.initial_X = x
            self.initial_Y = y
        Rotation = [qx, qy, qz, qw]
        release_point = self.calculate_position(x,y,Rotation,self.release_number, self.initial_X, self.initial_Y)
        distance = ((release_point.x - self.initial_X)**2 + (release_point.y - self.initial_Y)**2)**0.5
        self.get_logger().info(f"Distance from initial: {distance:.2f} m")
        Max_backward_distance = 10
        if (distance >= Max_backward_distance):
            return

        db_msg = self.create_db_msg(release_point, Rotation)
        self.publisher_.publish(db_msg)

        return


    def calculate_position(self, X, Y, rotation_quat, Counter, INITIAL_X, INITIAL_Y):
        
        self.get_logger().info(f"Calculate Release Position")

        backward_distance = 0.25 # 放土0.5回毎に位置を前にずらす距離[m] ####適宜変更

        rotation = R.from_quat(rotation_quat)
        roll, pitch, yaw = rotation.as_euler('xyz')

        backward_x = X - backward_distance * math.cos(yaw)
        backward_y = Y - backward_distance * math.sin(yaw)


        distance = ((INITIAL_X - backward_x)**2 + (INITIAL_Y - backward_y)**2)**0.5

        release_pos = Point(x=backward_x, y=backward_y, z=0.0)

        return release_pos
    



    def create_db_msg(self, LoadPoint, ForwardQuat) -> TmsdbQuery:
        """
        Create Tmsdb msg from Odometry msg.

        Parameters
        ----------
        msg : Odometry
            Target Machine's Odometry msg.

        Returns
        -------
        tms_db_msg : Tmsdb
            Message containing Odometry msg sent to tms_db_writer.
        """

        tms_db_msg = TmsdbQuery()
        tms_db_msg.type = WRITE_DATA_TYPE
        tms_db_msg.writemode = WRITE_MODE
      #  tms_db_msg.id = DATA_ID
        tms_db_msg.vehicle_name = self.machine_name
        tms_db_msg.record_name=RECORD_NAME


        tms_db_msg.add_path.pose.position.x = LoadPoint.x
        tms_db_msg.add_path.pose.position.y = LoadPoint.y
        tms_db_msg.add_path.pose.position.z = LoadPoint.z
        
        tms_db_msg.add_path.pose.orientation.x = ForwardQuat[0]
        tms_db_msg.add_path.pose.orientation.y = ForwardQuat[1]
        tms_db_msg.add_path.pose.orientation.z = ForwardQuat[2]
        tms_db_msg.add_path.pose.orientation.w = ForwardQuat[3]
        

        return tms_db_msg
    


def main(args=None):
    rclpy.init(args=args)

    tms_sp_machine_odom = TmsSpMachineOdom()

    rclpy.spin(tms_sp_machine_odom)

    tms_sp_machine_odom.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
