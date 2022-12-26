from typing import Tuple
import numpy as np
import open3d as o3d
from time import sleep
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
from tms_msg_db.action import TmsdbGridFS


NODE_NAME = 'tms_ur_construction_terrain_static' 
DATA_ID   = 3030 
DATA_TYPE = 'static'
CALLBACK_TIME = 0.5


class TmsUrConstructionTerrainStaticClient(Node):
    """Get static terrain data from tms_db_reader_gridfs."""

    def __init__(self) -> None:
        super().__init__(NODE_NAME)

        # Declare parameters
        self.declare_parameter('filename', 'filename')
        self.declare_parameter('voxel_size', '0.0')

        # Get parameters
        self.filename: str     = self.get_parameter('filename').get_parameter_value().string_value
        self.voxel_size: float = self.get_parameter('voxel_size').get_parameter_value().double_value

        self.publisher_ = self.create_publisher(PointCloud2, '~/output/terrain/static/pointcloud2', 10)
        self._action_client = ActionClient(self, TmsdbGridFS, 'tms_db_reader_gridfs')

    def send_goal(self) -> None:
        """
        Send goal to action server.
        """
        self.goal_msg = TmsdbGridFS.Goal()
        self.goal_msg.type     = DATA_TYPE
        self.goal_msg.id       = DATA_ID
        self.goal_msg.filename = self.filename

        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(self.goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future) -> None:
        """
        Get goal response call back from action server.

        Parameters
        ----------
        future
            Outcome of a task in the future.
        """
        goal_handle = future.result()
        if not goal_handle.accepted:
            # Goal denied
            return

        # Goal accepted
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future) -> None:
        """
        Publish PointCloud2 msg responded from tms_db_reader_gridfs as a result.

        Parameters
        ----------
        future
            Outcome of a task in the future.
        """
        result = future.result().result
        if not result.result:
            return

        # Create PointCloud2 msg
        self.msg: PointCloud2 = self.create_msg()

        while True:
            self.publisher_.publish(self.msg)
            sleep(CALLBACK_TIME)

    def feedback_callback(self, feedback_msg) -> None:
        """
        Callback function for feedback associated with the goal.

        Parameters
        ----------
        feedback_msg
            Feedback message.
        """
        pass

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

        # Downsampling
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


def main(args=None):
    rclpy.init(args=args)

    action_client = TmsUrConstructionTerrainStaticClient()
    action_client.send_goal()
    rclpy.spin(action_client)

    action_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()