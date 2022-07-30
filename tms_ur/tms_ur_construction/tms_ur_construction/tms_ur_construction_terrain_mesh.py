from typing import Tuple
import numpy as np
import open3d as o3d
import rclpy
from rclpy.node import Node

from shape_msgs.msg import Mesh, MeshTriangle
from geometry_msgs.msg import Point
from tms_msg_db.srv import TmsdbGridFSGetData


NODE_NAME = 'tms_ur_construction_terrain_mesh'
DATA_ID = 3030
DATA_TYPE = 'sensor'

class TmsUrConstructionTerrainMeshClient(Node):
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



class TmsUrConstructionTerrainMeshPublisher(Node):
    """Publish ground's Mesh."""

    def __init__(self):
        super().__init__(NODE_NAME)

        # declare parameter
        self.declare_parameter('filename', 'filename')
        self.declare_parameter('voxel_size', '0.0')
        self.declare_parameter('alpha', '1.0')

        # get parameter
        self.filename = self.get_parameter('filename').get_parameter_value().string_value
        self.voxel_size = self.get_parameter('voxel_size').get_parameter_value().double_value
        self.alpha = self.get_parameter('alpha').get_parameter_value().double_value

        self.publisher_ = self.create_publisher(Mesh, '~/output/mesh', 10)
        timer_period = 5
        self.msg = self.create_msg()
        self.timer = self.create_timer(timer_period, self.publish_mesh)

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

    def publish_mesh(self):
        """
        Publish terrain's Mesh topics.
        """
        self.publisher_.publish(self.msg)

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

    tms_ur_construction_terrain_mesh_client = TmsUrConstructionTerrainMeshClient()
    response = tms_ur_construction_terrain_mesh_client.send_request()

    tms_ur_construction_terrain_mesh_client.destroy_node()
    rclpy.shutdown()

    return response.result, tms_ur_construction_terrain_mesh_client.filename


def run_publisher() -> None:
    """
    Run a publisher node.
    """
    rclpy.init()

    tms_ur_construction_terrain_publisher = TmsUrConstructionTerrainMeshPublisher()
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