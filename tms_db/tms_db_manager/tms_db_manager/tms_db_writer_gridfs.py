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

        # declare parameter
        self.declare_parameter('db_host', 'localhost')
        self.declare_parameter('db_port', 27017)

        # get parameter
        self.db_host = self.get_parameter('db_host').get_parameter_value().string_value
        self.db_port = self.get_parameter('db_port').get_parameter_value().integer_value

        self.db = db_util.connect_db('rostmsdb', self.db_host, self.db_port)
        self.subscription = self.create_subscription(
            TmsdbGridFS,
            'tms_db_gridfs_data',
            self.db_write_callback,
            10)

    def db_write_callback(self, msg: TmsdbGridFS) -> None:
        """
        Store file data by using GridFS.

        Parameters
        ----------
        msg : TmsdbGridFS
            An instance of a ROS2 custom message to store file data.
        """
        filename = msg.filename
        
        f = open(filename, 'rb')

        fs = gridfs.GridFS(self.db)
        fs.put(f.read(), filename=filename, time=msg.time, type=msg.type, id=msg.id)

        f.close()
        os.remove(filename)

        self.get_logger().info('ok')


def main(args=None):
    rclpy.init(args=args)

    tms_db_writer_gridfs = TmsDbWriterGridFS()

    rclpy.spin(tms_db_writer_gridfs)

    tms_db_writer_gridfs.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
