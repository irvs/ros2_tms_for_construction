from datetime import datetime
import json
import math
import rclpy
from rclpy.node import Node

import numpy as np
import quaternion

from nav_msgs.msg import Odometry
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from tms_msg_db.msg import Tmsdb
import tms_db_manager.tms_db_util as db_util


NODE_NAME = "tms_sp_machine_odom"
DATA_ID = 2012
DATA_TYPE = "machine"


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

        self.publisher_ = self.create_publisher(Tmsdb, "tms_db_data", 10)
        self.subscription = self.create_subscription(
            Odometry, "~/input/odom", self.send_odom_to_db_writer, 10
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.is_excepted = False

        # This publisher is to check the transformation.
        self.publisher = self.create_publisher(
            Odometry, f"tf_machine/{self.machine_name}", 10
        )

        self.is_received = False

    def send_odom_to_db_writer(self, msg: Odometry) -> None:
        """
        Send topics to tms_db_writer (Write the received Odometry data to DB).

        parameters
        ----------
        msg : Odometry
            Target Object's Odometry.
        """
        # Log
        if not self.is_received:
            self.get_logger().info(f"Received {self.machine_name}'s Odometry msg")
            self.is_received = True

        # Transform
        msg = self.transform(msg)

        db_msg = self.create_db_msg(msg)
        self.publisher_.publish(db_msg)

    def transform(self, msg: Odometry) -> Odometry:
        # Euler to Quaternion
        """
        Transform Odometry msg to the specified frame.

        Parameters
        ----------
        msg : Odometry
            Target Machine's Odometry msg.

        Returns
        -------
        msg : Odometry
            Transformed Odometry msg.
        """
        from_frame = msg.header.frame_id

        try:
            tf = self.tf_buffer.lookup_transform(
                self.to_frame, from_frame, rclpy.time.Time()
            )
            if self.is_excepted:
                self.get_logger().info("Transformation is available.")
                self.is_excepted = False
        except TransformException as e:
            if not self.is_excepted:
                self.get_logger().info("There is no transformation.")
                self.get_logger().warning(str(e))
                self.is_excepted = True
            return msg

        msg.header.frame_id = self.to_frame

        # Apply translation
        # msg.pose.pose.position.x += tf.transform.translation.x
        # msg.pose.pose.position.y += tf.transform.translation.y
        # msg.pose.pose.position.z += tf.transform.translation.z

        # Euler
        # roll = math.radians(tf.transform.rotation.x)
        # pitch = math.radians(tf.transform.rotation.y)
        # yaw = math.radians(tf.transform.rotation.z)
        # roll, pitch, yaw = self.quaternion_to_euler(
        #     tf.transform.rotation.x,
        #     tf.transform.rotation.y,
        #     tf.transform.rotation.z,
        #     tf.transform.rotation.w,
        # )
        roll, pitch, yaw = quaternion.as_euler_angles(
            quaternion.quaternion(
                tf.transform.rotation.w,
                tf.transform.rotation.x,
                tf.transform.rotation.y,
                tf.transform.rotation.z,
            )
        )

        x = msg.pose.pose.position.x + tf.transform.translation.x
        y = msg.pose.pose.position.y + tf.transform.translation.y
        z = msg.pose.pose.position.z + tf.transform.translation.z

        # Apply rotation
        sin_roll = math.sin(roll)
        cos_roll = math.cos(roll)
        sin_pitch = math.sin(pitch)
        cos_pitch = math.cos(pitch)
        sin_yaw = math.sin(yaw)
        cos_yaw = math.cos(yaw)
        msg.pose.pose.position.x = (
            x * (cos_yaw * cos_pitch * cos_roll - sin_yaw * sin_roll)
            + y * (-sin_yaw * cos_pitch * cos_roll - cos_yaw * sin_roll)
            + z * (sin_pitch * cos_roll)
        )
        msg.pose.pose.position.y = (
            x * (cos_yaw * cos_pitch * sin_roll + sin_yaw * cos_roll)
            + y * (-sin_yaw * cos_pitch * sin_roll + cos_yaw * cos_roll)
            + z * (sin_pitch * sin_roll)
        )
        msg.pose.pose.position.z = (
            x * (-cos_yaw * sin_pitch) + y * (sin_yaw * sin_pitch) + z * (cos_pitch)
        )

        # Quaternion
        tf_q = np.quaternion(
            tf.transform.rotation.w,
            tf.transform.rotation.x,
            tf.transform.rotation.y,
            tf.transform.rotation.z,
        )

        # Apply rotation
        pose_quat = np.quaternion(
            msg.pose.pose.orientation.w,
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
        )

        # Rotate the pose_quat by tf_quat
        new_quat = pose_quat * tf_q
        new_quat = new_quat.normalized()

        # Assign the rotated quaternion to the message
        msg.pose.pose.orientation.x = new_quat.x
        msg.pose.pose.orientation.y = new_quat.y
        msg.pose.pose.orientation.z = new_quat.z
        msg.pose.pose.orientation.w = new_quat.w

        self.publisher.publish(msg)

        return msg

    # def quaternion_to_euler(self, x, y, z, w) -> tuple:
    #     roll = math.atan2(2 * (w * x + y * z), w**2 - x**2 - y**2 + z**2)
    #     pitch = math.asin(2 * (w * y - z * x))
    #     yaw = math.atan2(2 * (w * z + x * y), w**2 + x**2 - y**2 - z**2)

    #     return roll, pitch, yaw

    # def euler_to_quaternion(self, roll, pitch, yaw) -> np.quaternion:
    #     cr = math.cos(roll / 2)
    #     sr = math.sin(roll / 2)
    #     cp = math.cos(pitch / 2)
    #     sp = math.sin(pitch / 2)
    #     cy = math.cos(yaw / 2)
    #     sy = math.sin(yaw / 2)

    #     q = np.quaternion(
    #         cr * cp * cy + sr * sp * sy,
    #         sr * cp * cy - cr * sp * sy,
    #         cr * sp * cy + sr * cp * sy,
    #         cr * cp * sy - sr * sp * cy,
    #     )

    #     return q

    def create_db_msg(self, msg: Odometry) -> Tmsdb:
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
        tms_db_msg = Tmsdb()
        tms_db_msg.time = datetime.now().isoformat()
        tms_db_msg.type = DATA_TYPE
        tms_db_msg.id = DATA_ID
        tms_db_msg.name = self.machine_name

        # Convert Odometry msg to dictionary and then to json.
        doc: dict = db_util.msg_to_document(msg)
        tms_db_msg.msg: str = json.dumps(doc)

        return tms_db_msg


def main(args=None):
    rclpy.init(args=args)

    tms_sp_machine_odom = TmsSpMachineOdom()

    rclpy.spin(tms_sp_machine_odom)

    tms_sp_machine_odom.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
