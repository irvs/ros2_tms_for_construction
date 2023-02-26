from datetime import datetime
import numpy as np
import open3d as o3d

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2

from tms_msg_db.msg import TmsdbGridFS


NODE_NAME = 'tms_ss_terrain_static_mesh'
DATA_ID   = 3030 
DATA_TYPE = 'mesh'

class TmsSsTerrainStaticMesh(Node):
    """Create mesh file using PointCloud2 and send the file name to tms_db_writer_gridfs."""

    def __init__(self):
        super().__init__(NODE_NAME)

        # Declare parameters
        self.declare_parameter('filename_mesh', 'filename_mesh.ply')
        self.declare_parameter('voxel_size', 0.0)
        self.declare_parameter('octree_depth', 2)
        self.declare_parameter('density_th', 0.1)

        # Get parameters
        self.filename_mesh: str = self.get_parameter('filename_mesh').get_parameter_value().string_value
        self.voxel_size: float  = self.get_parameter('voxel_size').get_parameter_value().double_value
        self.octree_depth: int  = self.get_parameter('octree_depth').get_parameter_value().integer_value
        self.density_th: float  = self.get_parameter('density_th').get_parameter_value().double_value

        self.publisher_ = self.create_publisher(TmsdbGridFS, 'tms_db_gridfs_data', 10)
        self.subscription = self.create_subscription(
            PointCloud2,
            'tms_sd_terrain_static',
            self.send_mesh_to_db_writer_gridfs,
            10)

    def send_mesh_to_db_writer_gridfs(self, msg: PointCloud2) -> None:
        """
        Send topics to tms_db_writer_gridfs (Write the received PointCloud2 data to DB as a pcd file).

        Parameters
        ----------
        msg : PointCloud2
            Point cloud data.
        """
        self.msg = msg
        self.pointcloud2_mesh_handler()

        tmsdb_gridfs_msg          = TmsdbGridFS()
        tmsdb_gridfs_msg.time     = datetime.now().isoformat()
        tmsdb_gridfs_msg.type     = DATA_TYPE
        tmsdb_gridfs_msg.id       = DATA_ID
        tmsdb_gridfs_msg.filename = self.filename_mesh

        self.publisher_.publish(tmsdb_gridfs_msg)

    def pointcloud2_mesh_handler(self) -> None:
        """
        Convert PointCloud2 msg to PCD file.

        Parameters
        ----------
        msg : PointCloud2
            Point cloud data.
        """
        self.get_logger().info("Creating a file of static terrain's mesh ...")

        self.pcd: o3d.geometry.PointCloud    = self.convert_pointcloud2_to_pcd()
        self.mesh: o3d.geometry.TriangleMesh = self.convert_pcd_to_mesh()

        self.get_logger().info("The static terrain's mesh file was created!")

        o3d.io.write_triangle_mesh(self.filename_mesh, self.mesh)

    def convert_pointcloud2_to_pcd(self) -> o3d.geometry.PointCloud:
        """
        Convert PointCloud2 msg to open3d.geometry.PointCloud.

        Returns
        -------
        pcd : open3d.geometry.PointCloud
            Point Cloud of static terrain.
        """
        field_names = [field.name for field in self.msg.fields]
        cloud_data  = list(point_cloud2.read_points(self.msg, skip_nans=True, field_names = field_names))

        points = []
        colors = []
        for x, y, z, r, g, b in cloud_data:
            xyz = (x, y, z)
            rgb = (r, g, b)
            points.append(xyz)
            colors.append(rgb)

        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(np.array(points))
        pcd.colors = o3d.utility.Vector3dVector(np.array(colors))

        # Downsampling
        if self.voxel_size > 0:
            pcd = pcd.voxel_down_sample(voxel_size=self.voxel_size)

        return pcd

    def convert_pcd_to_mesh(self) -> o3d.geometry.TriangleMesh:
        """
        Convert open3d.geometry.PointCloud to open3d.geometry.TriangleMesh

        Returns
        -------
        mesh : open3d.geometry.TriangleMesh
            Mesh of static terrain.
        """
        self.pcd.estimate_normals()

        # If you want to check the normals, please uncomment out the following
        # o3d.visualization.draw_geometries([self.pcd], point_show_normal=True)

        # Fix normals
        self.pcd.orient_normals_to_align_with_direction(orientation_reference=np.asarray([0.0, 0.0, -1.0]))

        # If you want to check the normals, please uncomment out the following.
        # o3d.visualization.draw_geometries([self.pcd], point_show_normal=True)

        # Generate mesh using "Poisson surface reconstruction [Kazhdan 2006]" method
        mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(self.pcd, depth=self.octree_depth)

        # Remove all vertices (and connected triangles) that have a lower density value than the density threshold of all density values.
        densities = np.asarray(densities)
        vertices_to_remove = densities < np.quantile(densities, self.density_th)
        mesh.remove_vertices_by_mask(vertices_to_remove)

        # If you want to check the generated mesh, please uncomment out the following.
        # o3d.visualization.draw_geometries([mesh], point_show_normal=True)

        return mesh


def main(args=None):
    rclpy.init(args=args)

    tms_ss_terrain_static_mesh = TmsSsTerrainStaticMesh()
    rclpy.spin_once(tms_ss_terrain_static_mesh)

    tms_ss_terrain_static_mesh.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
