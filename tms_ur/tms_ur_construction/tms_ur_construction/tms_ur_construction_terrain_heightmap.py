# Copyright 2023, IRVS Laboratory, Kyushu University, Japan.
# 
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
# 
#     http://www.apache.org/licenses/LICENSE-2.0
# 
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import numpy as np
import open3d as o3d
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from shape_msgs.msg import MeshTriangle
from geometry_msgs.msg import Point, Vector3
from std_msgs.msg import ColorRGBA

from tms_msg_db.msg import ColoredMesh
from tms_msg_db.msg import TmsdbTerrainImageMsg
from tms_msg_db.srv import TmsdbTerrainImageSrv
from tms_msg_db.action import TmsdbTerrainImage

NODE_NAME = "tms_ur_construction_terrain_mesh"
DATA_ID = 4031
#DATA_TYPE = "heightmap"


class TmsUrConstructionTerrainMeshClient(Node):
    """Get terrain's heightmap data from tms_db_reader_heightmap."""

    def __init__(self):
        super().__init__(NODE_NAME)

        # Declare parameters
        self.declare_parameter("filename_mesh", "filename_mesh")
        self.declare_parameter("DATA_TYPE", "default_value")
        self.DATA_TYPE: str = (
            self.get_parameter("DATA_TYPE").get_parameter_value().string_value
        )

        # Get parameters
        self.filename_mesh: str = (
            self.get_parameter("filename_mesh").get_parameter_value().string_value
        )

        # Action Client
        self._action_client = ActionClient(self, TmsdbTerrainImage, "tms_db_reader_heightmap")

    def send_goal(self) -> None:
        """
        Send goal to action server.
        """
        self.goal_msg = TmsdbTerrainImage.Goal()
        self.goal_msg.type = self.DATA_TYPE
        self.goal_msg.id = DATA_ID
        self.goal_msg.filename = self.filename_mesh

        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(
            self.goal_msg, feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)

        self.get_logger().info("Seng a goal of heightmap")

    def feedback_callback(self, feedback_msg) -> None:
        """
        Callback function for feedback associated with the goal.

        Parameters
        ----------
        feedback_msg
            Feedback message.
        """
        pass

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
            self.get_logger().info("Goal is denied")
            return

        # Goal accepted
        self.get_logger().info("Goal is accepted")
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
        self.get_logger().info("Prepare for send heightmap")
        result = future.result().result
        if not result.result:
            self.get_logger().info("Return a response of None")
            return

        self.get_logger().info("Return a response of Heightmap")

        
        # 画像をダウンロード
        #file_idを使って画像を取得
        file_data = result.image

        # Create ColoredMesh msg
        self.msg: TmsdbTerrainImageMsg.time = result.time
        self.msg1: TmsdbTerrainImageMsg.terrainheight = result.terrainheight
        self.msg2: TmsdbTerrainImageMsg.terrainwidth = result.terrainwidth
        self.msg3: TmsdbTerrainImageMsg.terrainelevation = result.terrainelevation
        self.msg4: TmsdbTerrainImageMsg.offset_x = result.offset_x
        self.msg5: TmsdbTerrainImageMsg.offset_y = result.offset_y
        self.msg6: TmsdbTerrainImageMsg.image = result.image

        self.get_logger().info("Terrain service is ready")
        
        # Service server
        if self.DATA_TYPE == "heightmap":

            self.srv = self.create_service(
                TmsdbTerrainImageSrv, "~/output/terrain/mesh_srv", self.terrain_terrain_srv_callback
            )
        elif  self.DATA_TYPE == "texture":
            self.srv = self.create_service(
                TmsdbTerrainImageSrv, "output/terrain/texture", self.terrain_terrain_srv_callback
            )

        """
        # Service server
        self.srv = self.create_service(
            TmsdbTerrainImageSrv, "~/output/terrain/mesh_srv", self.terrain_terrain_srv_callback
        )
        """

        """
        self.srv = self.create_service(
            TmsdbTerrainImageSrv, "~/output/terrain/heightmap_srv", self.terrain_terrain_srv_callback
        )
        """
        # Create ColoredMesh msg
      #  self.msg: ColoredMesh = self.create_msg()

     #   self.get_logger().info("Colored Mesh service is ready")

        # Service server
    #    self.srv = self.create_service(
     #       ColoredMeshSrv, "~/output/terrain/mesh_srv", self.terrain_mesh_srv_callback
      #  )


    def terrain_terrain_srv_callback(self, request, response):
        """
        Callback function.

        Returns
        -------
        response
            Service callback response.
        """
        response.time = self.msg
        response.terrainheight = self.msg1
        response.terrainwidth = self.msg2
        response.terrainelevation = self.msg3
        response.offset_x = self.msg4
        response.offset_y = self.msg5
        response.image = self.msg6
       # response.image = file_data


        self.get_logger().info("Return a response of ColoredMesh")
        return response

        #DATA_TYPE = "texture"
        #send_goal()
    

    def terrain_mesh_srv_callback(self, request, response):
        """
        Callback function.

        Returns
        -------
        response
            Service callback response.
        """
        response.colored_mesh = self.msg
        self.get_logger().info("Return a response of ColoredMesh")
        return response

    def create_msg(self) -> ColoredMesh:
        """
        Create Mesh msg from a .pcd file.

        Returns
        -------
        msg : PointCluod2
            Point cloud data.
        """
        self.mesh: o3d.geometry.TriangleMesh = o3d.io.read_triangle_mesh(
            self.filename_mesh
        )
        msg: ColoredMesh = self.convert_triangle_mesh_to_colored_mesh()

        return msg

    def convert_triangle_mesh_to_colored_mesh(self) -> ColoredMesh:
        """
        Convert open3d.geometry.TriangleMesh to ColoredMesh msg.

        Returns
        -------
        msg : ColoredMesh
            ColoredMesh msg.
        """
        msg: ColoredMesh = ColoredMesh()
        msg.triangles: list = self.get_triangles(
            np.asarray(self.mesh.triangles).astype(np.uint32)
        )
        msg.vertices: list = self.get_vertices(np.asarray(self.mesh.vertices))
        msg.vertex_colors: list = self.get_vertex_colors(
            np.asarray(self.mesh.vertex_colors)
        )
        msg.vertex_normals: list = self.get_vertex_normals(
            np.asarray(self.mesh.vertex_normals)
        )

        return msg

    def get_triangles(self, mesh_triangles: np.ndarray) -> list:
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

    def get_vertices(self, mesh_vertices: np.ndarray) -> list:
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

    def get_vertex_colors(self, mesh_vertex_colors: np.ndarray) -> list:
        """
        Get vertex colors.

        Parameters
        ----------
        mesh_vertex_colors : numpy.ndarray
            Mesh vertex colors.

        Returns
        -------
        List[ColorRGBA]
            Vertex colores.
        """
        vertex_colors = []
        for mesh_vertex_color in mesh_vertex_colors:
            vertex_color = ColorRGBA()
            vertex_color.r = mesh_vertex_color[0]
            vertex_color.g = mesh_vertex_color[1]
            vertex_color.b = mesh_vertex_color[2]
            vertex_colors.append(vertex_color)
        return vertex_colors

    def get_vertex_normals(self, mesh_vertex_normals: np.ndarray) -> list:
        """
        Get vertex normals.

        Parameters
        ----------
        mesh_vertex_normals : numpy.ndarray
            Mesh vertex normals.

        Returns
        -------
        List[Vector3]
            Vertex normals.
        """
        vertex_normals = []
        for mesh_vertex_normal in mesh_vertex_normals:
            vertex_normal = Vector3()
            vertex_normal.x = mesh_vertex_normal[0]
            vertex_normal.y = mesh_vertex_normal[1]
            vertex_normal.z = mesh_vertex_normal[2]
            vertex_normals.append(vertex_normal)
        return vertex_normals


def main(args=None):
    rclpy.init(args=args)

    action_client = TmsUrConstructionTerrainMeshClient()
    action_client.send_goal()
    rclpy.spin(action_client)

    action_client.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
