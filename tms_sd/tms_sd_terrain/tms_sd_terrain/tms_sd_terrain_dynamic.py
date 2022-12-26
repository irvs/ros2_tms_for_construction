from datetime import datetime

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2

from tms_msg_db.msg import TmsdbGridFS


NODE_NAME = 'tms_sd_terrain_dynamic'
DATA_ID   = 3031 
DATA_TYPE = 'dynamic'

class TmsSdTerrainDynamic(Node):
    """Convert PointCloud2 msg to TmsdbGridFS msg sent to tms_db_writer_gridfs."""

    def __init__(self):
        super().__init__(NODE_NAME)

        # Declare parameters
        self.declare_parameter('terrain_name', 'terrain_name')

        # Get parameters
        self.terrain_name: str = self.get_parameter('terrain_name').get_parameter_value().string_value

        self.publisher_   = self.create_publisher(TmsdbGridFS, 'tms_db_gridfs_data', 10)
        self.subscription = self.create_subscription(
            PointCloud2,
            '~/input/terrain/dynamic/pointcloud2',
            self.send_pointcloud_to_db_writer,
            10)

    def send_pointcloud_to_db_writer(self, msg: PointCloud2) -> None:
        """
        Send topics to tms_db_writer_gridfs (Write the received PointCloud2 data to DB).

        Parameters
        ----------
        msg : PointCloud2
            Target Object's PointCloud2.
        """
        tmsdb_gridfs_msg             = TmsdbGridFS()
        tmsdb_gridfs_msg.time        = datetime.fromtimestamp(msg.header.stamp.sec + msg.header.stamp.nanosec * 10 ** (-9)).isoformat()
        tmsdb_gridfs_msg.type        = DATA_TYPE
        tmsdb_gridfs_msg.id          = DATA_ID
        tmsdb_gridfs_msg.pointcloud2 = msg
 
        self.publisher_.publish(tmsdb_gridfs_msg)


def main(args=None):
    rclpy.init(args=args)

    tms_sd_terrain_dynamic = TmsSdTerrainDynamic()
    rclpy.spin(tms_sd_terrain_dynamic)

    tms_sd_terrain_dynamic.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
