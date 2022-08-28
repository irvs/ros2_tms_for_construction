from typing import Tuple
import numpy as np
import open3d as o3d
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
from tms_msg_db.srv import TmsdbGridFSGetData


NODE_NAME = 'tms_ur_construction_terrain' 
DATA_ID   = 3030 
DATA_TYPE = 'sensor'

class TmsUrConstructionTerrainClient(Node):
    """Get ground's data from tms_db_reader_gridfs."""

    def __init__(self):
        super().__init__(NODE_NAME)

        # declare_parameter
        self.declare_parameter('filename', 'filename')
        self.filename = self.get_parameter('filename').get_parameter_value().string_value

        self.cli = self.create_client(TmsdbGridFSGetData, 'tms_db_reader_gridfs')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = TmsdbGridFSGetData.Request()


    def send_request(self):
        """
        Send request to tms_db_reader_gridfs to get ground data.
        
        Returns
        -------
        Any
            Result of request to tms_db_reader_gridfs.
        """
        self.req.type      = DATA_TYPE
        self.req.id        = DATA_ID
        self.req.filename  = self.filename
        self.future        = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()


class TmsUrConstructionTerrainPublisher(Node):
    """Publish ground's PointCloud2."""

    def __init__(self):
        super().__init__(NODE_NAME)

        # declare parameter
        self.declare_parameter('filename', 'filename')
        self.declare_parameter('voxel_size', '0.0')

        # get parameter
        self.filename = self.get_parameter('filename').get_parameter_value().string_value
        self.voxel_size = self.get_parameter('voxel_size').get_parameter_value().double_value

        self.publisher_ = self.create_publisher(PointCloud2, '~/output/pointcloud2', 10)
        timer_period = 5
        self.msg = self.create_msg()
        self.timer = self.create_timer(timer_period, self.publish_pointcloud2)

    def publish_pointcloud2(self):
        """
        Publish terrain's PointCloud2 topics.
        """
        self.publisher_.publish(self.msg)

    def create_msg(self) -> PointCloud2:
        """
        Create PointCloud2 msg from a .pcd file.

        Returns
        -------
        PointCluod2
            Point cloud data.
        """
        pcd = self.get_downsampled_pcd()

        points = np.asarray(pcd.points)
        colors = np.asarray(pcd.colors)

        points_colors = np.hstack([points, colors])
        msg = self.create_pointcloud2(points_colors)
        return msg

    def get_downsampled_pcd(self) -> o3d.geometry.PointCloud:
        """
        Get a .pcd file and downsampling it.

        Returns
        -------
        o3d.geometry.PointCloud
            Point cloud data.
        """
        pcd = o3d.io.read_point_cloud(self.filename)

        # downsampling
        if self.voxel_size > 0:
            pcd = pcd.voxel_down_sample(voxel_size=self.voxel_size)
        return pcd

    def create_pointcloud2(self, points_colors: np.ndarray) -> PointCloud2:
        """
        Create a PointCloud2 msg.

        Parameters
        ----------
        points_colors : np.ndarray
            Coordinate and color data.

        Returns
        -------
        PointCloud2
            Point cloud data.
        """
        ros_dtype = PointField.FLOAT32
        dtype     = np.float32
        itemsize  = np.dtype(dtype).itemsize
        data      = points_colors.astype(dtype).tobytes()
        fields    = [
            PointField(name=n, offset=i*itemsize, datatype=ros_dtype, count=1) for i, n in enumerate(['x', 'y', 'z', 'r','g','b'])
        ]

        msg = PointCloud2(
            header=Header(frame_id='map'),
            height=1,
            width=points_colors.shape[0],
            is_dense=False,
            is_bigendian=False,
            fields=fields,
            point_step=(itemsize * 6),
            row_step=(itemsize * 6 * points_colors.shape[0]),
            data=data,
        )
        return msg


def run_client(args=None) -> Tuple[bool, str]:
    """
    Run a client node.

    Parameters
    ----------
    args : List[str]
        List of command line arguments.

    Returns
    -------
    bool
        Result of request to service node.
    str
        Target's file name.
    """
    rclpy.init(args=args)

    tms_ur_construction_terrain_client = TmsUrConstructionTerrainClient()
    response = tms_ur_construction_terrain_client.send_request()

    tms_ur_construction_terrain_client.destroy_node()
    rclpy.shutdown()

    return response.result, tms_ur_construction_terrain_client.filename


def run_publisher() -> None:
    """
    Run a publisher node.
    """
    rclpy.init()

    tms_ur_construction_terrain_publisher = TmsUrConstructionTerrainPublisher()
    rclpy.spin(tms_ur_construction_terrain_publisher)

    tms_ur_construction_terrain_publisher.destroy_node()
    rclpy.shutdown()


def main(args=None):
    # client
    result, filename = run_client(args=args)

    if result == False:
        print(f'Failed to fetch file data. Maybe the file name ({filename}) was wrong.')
        return

    # publisher
    run_publisher()


if __name__ == '__main__':
    main()