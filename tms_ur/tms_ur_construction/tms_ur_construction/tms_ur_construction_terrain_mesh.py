from typing import Tuple
import numpy as np
import open3d as o3d
from time import sleep
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from shape_msgs.msg import Mesh, MeshTriangle
from geometry_msgs.msg import Point
from tms_msg_db.action import TmsdbGridFS


NODE_NAME = 'tms_ur_construction_terrain_mesh'
DATA_ID = 3030
DATA_TYPE = 'static'
CALLBACK_TIME = 0.5

class TmsUrConstructionTerrainMeshClient(Node):
    """Get ground's data from tms_db_reader_gridfs."""

    def __init__(self):
        super().__init__(NODE_NAME)

        # Declare parameters
        self.declare_parameter('filename', 'filename')
        self.declare_parameter('voxel_size', '0.0')
        self.declare_parameter('alpha', '1.0')

        # Get parameters
        self.filename: str     = self.get_parameter('filename').get_parameter_value().string_value
        self.voxel_size: float = self.get_parameter('voxel_size').get_parameter_value().double_value
        self.alpha: float      = self.get_parameter('alpha').get_parameter_value().double_value

        self.publisher_ = self.create_publisher(Mesh, '~/output/mesh', 10)
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
        Publish Mesh msg responded from tms_db_reader_gridfs as a result.

        Parameters
        ----------
        future
            Outcome of a task in the future.
        """
        result = future.result().result
        if not result.result:
            return

        # Create Mesh msg
        self.msg: Mesh = self.create_msg()

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

    def create_msg(self) -> Mesh:
        """
        Create Mesh msg from a .pcd file.

        Returns
        -------
        PointCluod2
            Point cloud data.
        """
        pcd = self.get_downsampled_pcd()
        msg = self.create_mesh(pcd)
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

    def create_mesh(self, pcd):
        """
        Generate ground mesh data.

        Parameters
        ----------
        pcd : o3d.geometry.PointCloud
            Point cloud data.

        Returns
        -------
        Mesh
            Mesh data.
        """
        mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_alpha_shape(pcd, self.alpha)
        mesh.compute_vertex_normals

        msg = Mesh()
        msg.triangles = self.get_triangles(np.asarray(mesh.triangles).astype(np.uint32))
        msg.vertices  = self.get_vertices(np.asarray(mesh.vertices))
        return msg

    def get_triangles(self, mesh_triangles: np.ndarray):
        """
        Get triangles of mesh.

        Parameters
        ----------
        mesh_triangles : numpy.ndarray
            Mesh triangles.

        Returns
        -------
        List[MeshTriangle]
            Triangles.
        """
        triangles = []
        for mesh_triangle in mesh_triangles:
            triangle = MeshTriangle()
            triangle.vertex_indices = mesh_triangle
            triangles.append(triangle)
        return triangles

    def get_vertices(self, mesh_vertices: np.ndarray):
        """
        Get vertices of mesh.

        Parameters
        ----------
        mesh_vertices : numpy.ndarray
            Mesh vertices.

        Returns
        -------
        List[Point]
            Vertices.
        """
        vertices = []
        for mesh_vertice in mesh_vertices:
            point = Point()
            point.x = mesh_vertice[0]
            point.y = mesh_vertice[1]
            point.z = mesh_vertice[2]
            vertices.append(point)
        return vertices


def main(args=None):
    rclpy.init(args=args)

    action_client = TmsUrConstructionTerrainMeshClient()
    action_client.send_goal()
    rclpy.spin(action_client)

    action_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()