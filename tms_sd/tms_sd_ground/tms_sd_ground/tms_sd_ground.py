from datetime import datetime
import json

import numpy as np
import quaternion

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_geometry_msgs import do_transform_pose

import tms_db_manager.tms_db_util as db_util
from tms_msg_db.msg import Tmsdb


NODE_NAME = "tms_sd_ground"
DATA_ID = 3032
DATA_TYPE = "sensor"


class TmsSdGround(Node):
    """Convert OccupancyGrid msg to Tmsdb msg and sent to tms_db_writer."""

    def __init__(self):
        super().__init__(NODE_NAME)

        # Declare parameters
        self.declare_parameter("ground_name", "ground_name")
        self.declare_parameter("to_frame", "world")

        # Get parameters
        self.ground_name: str = (
            self.get_parameter("ground_name").get_parameter_value().string_value
        )
        self.to_frame: str = (
            self.get_parameter("to_frame").get_parameter_value().string_value
        )

        self.publisher_ = self.create_publisher(Tmsdb, "tms_db_data", 10)
        self.subscription = self.create_subscription(
            OccupancyGrid,
            "~/input/occupancy_grid",
            self.send_occupancy_grid_to_db_writer,
            10,
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.is_excepted = False

        # This publisher is to check the transformation.
        self.publisher = self.create_publisher(OccupancyGrid, "tf_ground", 10)

        self.is_received = False

    def send_occupancy_grid_to_db_writer(self, msg: OccupancyGrid) -> None:
        """
        Send topics to tms_db_writer (Write the received OccupancyGrid data to DB).

        Parameters
        ----------
        msg : OccupancyGrid
            Target Object's OccupancyGrid.
        """
        # Log
        if not self.is_received:
            self.get_logger().info(f"Received OccupancyGrid msg")
            self.is_received = True

        # Transform
        msg = self.transform_msg(msg)

        db_msg: Tmsdb = self.create_db_msg(msg)
        self.publisher_.publish(db_msg)

    def transform_msg(self, msg: OccupancyGrid) -> OccupancyGrid:
        """
        Transform OccupancyGrid msg.

        Parameters
        ----------
        msg : OccupancyGrid
            Target OccupancyGrid msg.

        Returns
        -------
        msg : OccupancyGrid
            Transformed OccupancyGrid msg.
        """
        prev_origin = msg.info.origin

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
        transformed_origin = do_transform_pose(prev_origin, tf)
        msg.info.origin = transformed_origin

        self.publisher.publish(msg)

        return msg

    def create_db_msg(self, msg: OccupancyGrid) -> Tmsdb:
        """
        Create Tmsdb msg from OccupancyGrid msg.

        Parameters
        ----------
        msg : OccupancyGrid
            Target Machine's OccupancyGrid msg.

        Returns
        -------
        tms_db_msg : Tmsdb
            Message containing OccupancyGrid msg sent to tms_db_writer.
        """
        tms_db_msg = Tmsdb()
        tms_db_msg.time = datetime.now().isoformat()
        tms_db_msg.type = DATA_TYPE
        tms_db_msg.id = DATA_ID
        tms_db_msg.name = self.ground_name

        # Convert OccupancyGrid msg to dictionary and then to json.
        doc: dict = db_util.msg_to_document(msg)
        tms_db_msg.msg: str = json.dumps(doc)

        return tms_db_msg


def main(args=None):
    rclpy.init(args=args)

    tms_sd_ground = TmsSdGround()
    rclpy.spin(tms_sd_ground)

    tms_sd_ground.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
