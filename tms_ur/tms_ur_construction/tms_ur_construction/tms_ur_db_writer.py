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

from datetime import datetime
import json
import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from tf2_ros.transform_listener import TransformListener
from tms_msg_ur.srv import TmsdbTerrainDBPoseWriteSrv
from tms_msg_ur.msg import TmsdbPoseWriteMsg

from tms_msg_db.msg import TmsdbQuery
import tms_db_manager.tms_db_util as db_util


NODE_NAME = "tms_sp_machine_odom"
DATA_ID = 3012 #2012
DATA_TYPE = "para"


class TmsURMachinePoseWriter(Node):
    """Convert PoseStamped msg to Tmsdb msg and sent to tms_db_writer."""

    def __init__(self):
        super().__init__(NODE_NAME)


        self.publisher_ = self.create_publisher(TmsdbQuery, "tms_db_param_data", 10)
        self.subscription = self.create_subscription(
            TmsdbPoseWriteMsg, "/input/WriteDB/pose_joint", self.send_msg_to_db_writer, 10
        )

        self.is_received = False

        self.get_logger().info("DBWriter service is ready")
        ############
        self.srv = self.create_service(
                TmsdbTerrainDBPoseWriteSrv, "/input/DB_machine/data", self.send_odom_to_db_writer
        )

    def send_odom_to_db_writer(self, request, response):
        """
        TmsdbTerrainDBPoseWriteSrv型のリクエストを受け取ってDBに保存
        """
        # DBに書き込む処理
        response.written = True  # レスポンスを設定

        self.get_logger().info(f"Received {request.machine_name}'s PoseStamped msg")
    
        # PoseStampedデータを処理する処理を追加
        if not self.is_received:
            self.get_logger().info(f"Received {request.machine_name}'s PoseStamped msg")
            self.is_received = True

    
        db_msg = self.create_db_msg(request)  # リクエストを使ってDBメッセージを作成
        self.publisher_.publish(db_msg)  # DBメッセージを発行

        return response
    

    def send_msg_to_db_writer(self, msg:TmsdbPoseWriteMsg) -> None:
        """
        TmsdbTerrainDBPoseWriteSrv型のリクエストを受け取ってDBに保存
        """
        self.get_logger().info(f"Received {msg.machine_name}'s PoseStamped msg")
    
        # PoseStampedデータを処理する処理を追加
        if not self.is_received:
            self.get_logger().info(f"Received {msg.machine_name}'s PoseStamped msg")
            self.is_received = True

    
        db_msg = self.create_db_msg(msg)  # リクエストを使ってDBメッセージを作成
        self.publisher_.publish(db_msg)  # DBメッセージを発行




    def create_db_msg(self, msg: Odometry) -> TmsdbQuery:
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
        tms_db_msg.time = datetime.now().isoformat()
        tms_db_msg.type = msg.record_type
        tms_db_msg.id = DATA_ID
        tms_db_msg.vehicle_name = msg.machine_name
        tms_db_msg.add_path = msg.pose
        tms_db_msg.add_joints = msg.joints


        return tms_db_msg


def main(args=None):
    rclpy.init(args=args)

    tms_sp_machine_odom = TmsURMachinePoseWriter()

    rclpy.spin(tms_sp_machine_odom)

    tms_sp_machine_odom.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
