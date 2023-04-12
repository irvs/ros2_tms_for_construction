import gridfs
import os

import rclpy
from rclpy.node import Node

import tms_db_manager.tms_db_util as db_util
from tms_msg_db.msg import TmsdbGridFS


class TmsDbWriterGridFS(Node):
    """Write file data to MongoDB."""

    def __init__(self):
        super().__init__('tms_db_writer_gridfs')

        # Declare parameters
        self.declare_parameter('db_host', 'localhost')
        self.declare_parameter('db_port', 27017)

        # Get parameters
        self.db_host: str = self.get_parameter('db_host').get_parameter_value().string_value
        self.db_port: int = self.get_parameter('db_port').get_parameter_value().integer_value

        self.db = db_util.connect_db('rostmsdb', self.db_host, self.db_port)
        self.subscription = self.create_subscription(
            TmsdbGridFS,
            'tms_db_gridfs_data',
            self.db_write_callback,
            10)

    def db_write_callback(self, msg: TmsdbGridFS) -> None:
        """
        Store terrain data using GridFS.

        Parameters
        ----------
        msg : TmsdbGridFS
            An instance of a ROS2 custom message to store data using GridFS.
        """
        self.fs = gridfs.GridFS(self.db)
        db_util.set_time_index(self.db["fs.files"])

        if msg.type == 'static':
            self.save_static_terrain(msg)
        elif msg.type == 'mesh':
            self.save_static_terrain_mesh(msg)
        elif msg.type == 'dynamic':
            self.save_dynamic_terrain(msg)
        else:
            self.get_logger().info("Please set a terrain type 'static' or 'dynamic'")

    def save_static_terrain(self, msg: TmsdbGridFS):
        """
        Save static terrain data using GridFS.

        Parameters
        ----------
        msg : TmsdbGridFS
            An instance of a ROS2 custom message to store data using GridFS.
        """
        filename: str = msg.filename
        
        f = open(filename, 'rb')

        self.fs.put(f.read(), filename=filename, time=msg.time, type=msg.type, id=msg.id)

        f.close()
        os.remove(filename)

    def save_static_terrain_mesh(self, msg: TmsdbGridFS) -> None:
        """
        Save mesh of static terrain using GridFS.

        Parameters
        ----------
        msg : TmsdbGridFS
            An instance of a ROS2 custom message to store data using GridFS.
        """
        filename: str = msg.filename

        f = open(filename, 'rb')

        self.fs.put(f.read(), filename=filename, time=msg.time, type=msg.type, id=msg.id)

        f.close()
        os.remove(filename)

    def save_dynamic_terrain(self, msg: TmsdbGridFS):
        """
        Save dynamic terrain data using GridFS.

        Parameters
        ----------
        msg : TmsdbGridFS
            An instance of a ROS2 custom message to store data using GridFS.
        """
        self.fs.put(
            ','.join(str(i) for i in msg.pointcloud2.data).encode(),
            time=msg.time,
            type=msg.type,
            id=msg.id,
            height=msg.pointcloud2.height,
            width=msg.pointcloud2.width,
            is_bigendian=msg.pointcloud2.is_bigendian,
            point_step=msg.pointcloud2.point_step,
            row_step=msg.pointcloud2.row_step,
            fields=[repr(field) for field in msg.pointcloud2.fields],
            is_dense=msg.pointcloud2.is_dense,
        )


def main(args=None):
    rclpy.init(args=args)

    tms_db_writer_gridfs = TmsDbWriterGridFS()
    rclpy.spin(tms_db_writer_gridfs)

    tms_db_writer_gridfs.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
