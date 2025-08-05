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

from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Point, Quaternion

from tms_msg_db.msg import TmsdbQuery
from tms_msg_db.srv import TmsdbGetData

from scipy.spatial.transform import Rotation as R
import math


#from tf2_ros import TransformListener, Buffer
from functools import partial

NODE_NAME = "tms_sp_machine_odom"
DATA_ID = 3012 #2012
READ_DATA_TYPE = "parameter"
WRITE_DATA_TYPE = "pose_query"
WRITE_MODE="over_write"
RECORD_NAME="RELEASE_POINT_test"

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

        self.record_name="RELEASE_POINT_test"

        self.cli = self.create_client(TmsdbGetData, "tms_db_param_reader")
        while not self.cli.wait_for_service(timeout_sec=0.5):
            self.get_logger().info("service not available, waiting again...")


        timer_period = 1
        self.call_timer = self.create_timer(timer_period, self.send_request)
        self.release_counter = 0
        self.Rotation = [0.0, 0.0, 0.0, 0.0]


    def send_request(self):
        """
        Send request to tms_db_reader to get PoseStamped data.
        """
        self.req = TmsdbGetData.Request()
        self.req.type = READ_DATA_TYPE
        self.req.id = DATA_ID
        self.req.latest_only = self.latest
        self.req.name = self.record_name

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


        # x, y, z のリストの先頭要素を取得（または必要に応じて全部使う）
        if self.release_counter != int(msg_data["Release_Count"]):
            self.release_counter = int(msg_data["Release_Count"])

            # x, y, z のリストの先頭要素を取得（または必要に応じて全部使う）
            x = float(msg_data["x"][0])
            y = float(msg_data["y"][0])
            z = 0.0
            qx = float(msg_data["qx"][0])
            qy = float(msg_data["qy"][0])
            qz = float(msg_data["qz"][0])
            qw = float(msg_data["qw"][0])
            Rotation = [qx, qy, qz, qw]
            release_point = self.calculate_position(x,y,Rotation,self.release_counter)
            db_msg = self.create_db_msg(release_point, Rotation)
            self.publisher_.publish(db_msg)
        

        return


    def calculate_position(self, X, Y, rotation_quat, Counter):
        
        self.get_logger().info(f"Calculate Release Position")
     #   self.get_logger().info(f"Calculate Release Position {x}")


        # 距離（ロボットの後方に取りたい距離[m]）
        backward_distance = 0.5


        rotation = R.from_quat(rotation_quat)
        roll, pitch, yaw = rotation.as_euler('xyz')

        backward_x = X - backward_distance * Counter * math.cos(yaw)
        backward_y = Y - backward_distance * Counter * math.sin(yaw)


        dump_forward = [0,0,yaw - math.pi/2]
        r = R.from_euler('xyz', dump_forward)
        quat = r.as_quat()

        release_pos = Point(x=backward_x, y=backward_y, z=0.0)
        forward_quat = Quaternion(x=quat[0], y=quat[1], z=quat[2], w=quat[3])

        self.get_logger().info(f"Current position: ({X:.2f}, {Y:.2f}), yaw: {math.degrees(yaw):.2f} deg")
        self.get_logger().info(f"{backward_distance}m backward position: ({backward_x:.2f}, {backward_y:.2f})")

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
       # tms_db_msg.time = datetime.now().isoformat()
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
