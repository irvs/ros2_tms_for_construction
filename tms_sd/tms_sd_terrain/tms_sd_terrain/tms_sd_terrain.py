from datetime import datetime
import rclpy
from rclpy.node import Node

import numpy as np
import open3d as o3d
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from tms_msg_db.msg import TmsdbGridFS


NODE_NAME = 'tms_sd_terrain'
DATA_ID   = 3030 
DATA_TYPE = 'sensor'

class TmsSdTerrain(Node):
    """Write PointCloud2 msg to file and sent the file name to tms_db_writer_gridfs."""

    def __init__(self):
        super().__init__(NODE_NAME)

        # declare parameter
        self.declare_parameter('file_name', 'file_name')
        self.file_name = self.get_parameter('file_name').get_parameter_value().string_value

        self.publisher_ = self.create_publisher(TmsdbGridFS, 'tms_db_gridfs_data', 10)
        self.subscription = self.create_subscription(
            PointCloud2,
            '~/input/pointcloud2',
            self.send_pointcloud_to_db_writer_gridfs,
            10)

    def send_pointcloud_to_db_writer_gridfs(self, msg: PointCloud2) -> None:
        """
        Send topics to tms_db_writer_gridfs (Write the received PointCloud2 data to DB as a pcd file).

        Parameters
        ----------
        msg : PointCloud2
            Point cloud data.
        """
        self.convert_pointcloud2_to_pcd(msg)

        tmsdb_gridfs_msg           = TmsdbGridFS()
        tmsdb_gridfs_msg.time      = datetime.now().isoformat()
        tmsdb_gridfs_msg.type      = DATA_TYPE
        tmsdb_gridfs_msg.id        = DATA_ID
        tmsdb_gridfs_msg.file_name = self.file_name

        self.publisher_.publish(tmsdb_gridfs_msg)

    def convert_pointcloud2_to_pcd(self, msg: PointCloud2) -> None:
        """
        Convert PointCloud2 msg to PCD file.

        Parameters
        ----------
        msg : PointCloud2
            Point cloud data.
        """
        field_names = [field.name for field in msg.fields]
        cloud_data  = list(point_cloud2.read_points(msg, skip_nans=True, field_names = field_names))

        points = []
        colors = []
        for x, y, z, r, g, b in cloud_data:
            xyz = (x, y, z)
            rgb = (r, g, b)
            points.append(xyz)
            colors.append(rgb)

        o3d_cloud = o3d.geometry.PointCloud()
        o3d_cloud.points = o3d.utility.Vector3dVector(np.array(points))
        o3d_cloud.colors = o3d.utility.Vector3dVector(np.array(colors))

        o3d.io.write_point_cloud(self.file_name, o3d_cloud)

def main(args=None):
    rclpy.init(args=args)

    tms_sd_terrain = TmsSdTerrain()

    rclpy.spin_once(tms_sd_terrain)

    tms_sd_terrain.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
