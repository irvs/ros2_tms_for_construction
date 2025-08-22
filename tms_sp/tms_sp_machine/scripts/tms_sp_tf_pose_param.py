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

from geometry_msgs.msg import Point, Quaternion

from tms_msg_db.msg import TmsdbQuery

from scipy.spatial.transform import Rotation as R
import math

from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformListener, Buffer

NODE_NAME = "tms_sp_machine_odom"
DATA_ID = 3012 #2012
DATA_TYPE = "pose_query"
WRITE_MODE="over_write"
RECORD_NAME="LOAD_POINT_TEST"       #waypoint1####適宜変更
RECORD_NAME2="R1_SIGNAL_POINT_TEST" #waypoint2####適宜変更

class TmsSpMachineOdom(Node):
    """Convert Odometry msg to Tmsdb msg and sent to tms_db_writer."""

    def __init__(self):
        super().__init__(NODE_NAME)

        # Declare parameters
        self.declare_parameter("machine_name", "machine_name")
        self.declare_parameter("to_frame", "world")

        # Get parameters
        self.machine_name: str = (
            self.get_parameter("machine_name").get_parameter_value().string_value
        )
        self.to_frame: str = (
            self.get_parameter("to_frame").get_parameter_value().string_value
        )

        self.publisher_ = self.create_publisher(TmsdbQuery, "tms_db_param_data", 10)
    #    self.subscription = self.create_subscription(
    #        PoseStamped, "~/input/odom", self.send_odom_to_db_writer, 10
    #    )

        self.is_received = False
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.timer = self.create_timer(0.5, self.send_odom_to_db_writer)

    def send_odom_to_db_writer(self) -> None:
        if not self.is_received:
          #  self.get_logger().info(f"Received {self.machine_name}'s Odometry msg")
            self.is_received = True

        load_point, second_point, forward_quat = self.calculate_position()
        if load_point is None:
          #  self.get_logger().warn("Skipping DB write because transform is unavailable")
            return

        db_msg = self.create_db_msg()
        self.publisher_.publish(db_msg)
        db_msg2 = self.create_db_msg2()
        self.publisher_.publish(db_msg2)


    def calculate_position(self):
        result = self.transform()
        if result is None:
          #  self.get_logger().warn("Transform is None, skipping position calculation")
            return None, None, None
    
        backward_distance = 6.0         # 距離（backhoeの後方(waypoint1)に取りたい距離[m]）  ####適宜変更
        dump_forward_distance = 13.0    # 距離（backhoeの右後方(waypoint2)に取りたい距離[m]）####適宜変更

        world_to_map_x, world_to_map_y, world_to_map_z, yaw =self.transform()

        x = world_to_map_x
        y = world_to_map_y

        backward_x = x - backward_distance * math.cos(yaw)
        backward_y = y - backward_distance * math.sin(yaw)

        second_point_x = backward_x + dump_forward_distance* math.cos(yaw - math.pi/2)
        second_point_y = backward_y + dump_forward_distance* math.sin(yaw - math.pi/2)

        dump_forward = [0,0,yaw - math.pi/2]
        r = R.from_euler('xyz', dump_forward)
        quat = r.as_quat()

        load_pos = Point(x=backward_x, y=backward_y, z=0.0)
        second_pos = Point(x=second_point_x, y=second_point_y, z=0.0)
        forward_quat = Quaternion(x=quat[0], y=quat[1], z=quat[2], w=quat[3])

        return load_pos, second_pos, forward_quat
    

    def transform(self):
        from_frame = 'map'               ####適宜変更
        to_frame = 'ic120_tf/base_link'  ####適宜変更

        try:
            now = rclpy.time.Time()
            trans: TransformStamped = self.tf_buffer.lookup_transform(
                from_frame,
                to_frame,
                now,
                timeout=rclpy.duration.Duration(seconds=1.0)
            )
            world_to_map_x = trans.transform.translation.x
            world_to_map_y = trans.transform.translation.y
            world_to_map_z = trans.transform.translation.z

            quat = trans.transform.rotation
            quaternion = [quat.x, quat.y, quat.z, quat.w]
            rotation = R.from_quat(quaternion)
            roll, pitch, yaw = rotation.as_euler('xyz')#, degrees=True)

            return world_to_map_x, world_to_map_y, world_to_map_z, yaw

        except Exception as e:
         #   self.get_logger().warn(f'Could not get transform: {e}')
            return None

    def create_db_msg(self) -> TmsdbQuery:
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
        LoadPoint, SecondPoint, ForwardQuat = self.calculate_position()

        tms_db_msg = TmsdbQuery()
        tms_db_msg.type = DATA_TYPE
        tms_db_msg.writemode = WRITE_MODE
      #  tms_db_msg.id = DATA_ID
        tms_db_msg.vehicle_name = self.machine_name
        tms_db_msg.record_name=RECORD_NAME

        tms_db_msg.add_path.pose.position.x = LoadPoint.x
        tms_db_msg.add_path.pose.position.y = LoadPoint.y
        tms_db_msg.add_path.pose.position.z = LoadPoint.z
        tms_db_msg.add_path.pose.orientation.x = ForwardQuat.x
        tms_db_msg.add_path.pose.orientation.y = ForwardQuat.y
        tms_db_msg.add_path.pose.orientation.z = ForwardQuat.z
        tms_db_msg.add_path.pose.orientation.w = ForwardQuat.w

        return tms_db_msg
    
    def create_db_msg2(self) -> TmsdbQuery:
        LoadPoint, SecondPoint, ForwardQuat = self.calculate_position()

        tms_db_msg = TmsdbQuery()
        tms_db_msg.type = DATA_TYPE
        tms_db_msg.writemode = WRITE_MODE
      #  tms_db_msg.id = DATA_ID
        tms_db_msg.vehicle_name = self.machine_name
        tms_db_msg.record_name=RECORD_NAME2

        tms_db_msg.add_path.pose.position.x = SecondPoint.x
        tms_db_msg.add_path.pose.position.y = SecondPoint.y
        tms_db_msg.add_path.pose.position.z = SecondPoint.z

        tms_db_msg.add_path.pose.orientation.x = ForwardQuat.x
        tms_db_msg.add_path.pose.orientation.y = ForwardQuat.y
        tms_db_msg.add_path.pose.orientation.z = ForwardQuat.z
        tms_db_msg.add_path.pose.orientation.w = ForwardQuat.w
        return tms_db_msg


def main(args=None):
    rclpy.init(args=args)

    tms_sp_machine_odom = TmsSpMachineOdom()

    rclpy.spin(tms_sp_machine_odom)

    tms_sp_machine_odom.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
