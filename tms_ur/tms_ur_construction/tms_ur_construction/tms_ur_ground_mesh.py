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

from datetime import datetime
from functools import partial
import json
import rclpy
from rclpy.node import Node

from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import ColorRGBA
from tms_msg_db.msg import ColoredMesh
from tms_msg_db.srv import TmsdbGetData, ColoredMeshSrv

import tms_db_manager.tms_db_util as db_util

import numpy as np
import math
import open3d as o3d
import quaternion


NODE_NAME = "tms_ur_ground_mesh"
DATA_ID = 3032
DATA_TYPE = "sensor"


class TmsUrGroundMesh(Node):
    """Publish the combined data of ground's data and terrain mesh data."""

    def __init__(self):
        super().__init__(NODE_NAME)

        # Declare parameters
        self.declare_parameter("timer_period", 10)  # seconds

        # Get parameters
        self.timer_period: int = (
            self.get_parameter("timer_period").get_parameter_value().integer_value
        )

        self.publisher_ = self.create_publisher(ColoredMesh, "~/output/ground_mesh", 10)

        # Tmsdb Srv for ground data
        self.tmsdb_srv_cli = self.create_client(TmsdbGetData, "tms_db_reader")
        while not self.tmsdb_srv_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("service for ground not available, waiting again...")

        # Colored Mesh Srv for terrain mesh data
        self.coloredmesh_srv_cli = self.create_client(
            ColoredMeshSrv, "/output/terrain/mesh_srv"
        )
        while not self.coloredmesh_srv_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("service for mesh not available, waiting again...")

        self.call_timer = self.create_timer(self.timer_period, self.timer_callback)

    def timer_callback(self):
        """
        Get OccupancyGrid data from tms_db_reader and publish them.
        """
        self.send_request_for_ground()

        try:
            self.tmsdbs = self.tmsdb_srv_res.tmsdbs
        except:
            return

        self.send_request_for_static_mesh()

        try:
            self.colored_mesh = self.coloredmesh_srv_res.colored_mesh
        except:
            return

        self.publish_colored_mesh()

    def send_request_for_ground(self):
        """
        Send request to tms_db_reader to get ground data.

        Returns
        -------
        Any
            Result of request to tms_db_reader.
        """
        self.req = TmsdbGetData.Request()
        self.req.type = DATA_TYPE
        self.req.id = DATA_ID
        self.req.latest_only = True

        future = self.tmsdb_srv_cli.call_async(self.req)
        future.add_done_callback(partial(self.callback_set_response_for_ground))

    def callback_set_response_for_ground(self, future):
        """
        Set response from tms_db_reader.
        """
        self.tmsdb_srv_res = future.result()

    def send_request_for_static_mesh(self):
        """
        Send request to tms_db_reader to get static mesh data.
        """
        self.coloredmesh_srv_cli = self.create_client(
            ColoredMeshSrv, "/output/terrain/mesh_srv"
        )
        while not self.coloredmesh_srv_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("service for mesh not available, waiting again...")

        self.coloredmesh_srv_req = ColoredMeshSrv.Request()

        future = self.coloredmesh_srv_cli.call_async(self.coloredmesh_srv_req)
        future.add_done_callback(partial(self.callback_set_response_for_static_mesh))

    def callback_set_response_for_static_mesh(self, future):
        """
        Set response from tms_db_reader.
        """
        self.coloredmesh_srv_res = future.result()

    def calculate_euler(self):
        """
        Calculate euler angles and their sin and cos from quaternion.
        """
        roll, pitch, yaw = quaternion.as_euler_angles(
            quaternion.quaternion(
                self.occupancy_grid_msg.info.origin.orientation.w,
                self.occupancy_grid_msg.info.origin.orientation.x,
                self.occupancy_grid_msg.info.origin.orientation.y,
                self.occupancy_grid_msg.info.origin.orientation.z,
            ).conj()
        )

        self.sin_roll = math.sin(roll)
        self.cos_roll = math.cos(roll)
        self.sin_pitch = math.sin(pitch)
        self.cos_pitch = math.cos(pitch)
        self.sin_yaw = math.sin(yaw)
        self.cos_yaw = math.cos(yaw)

    def rotated_x(self, vertex_x: float, vertex_y: float, vertex_z: float) -> float:
        """
        Rotate x coordinate of vertex with euler angles.

        Parameters
        ----------
        vertex_x : float
            x coordinate of vertex.
        vertex_y : float
            y coordinate of vertex.
        vertex_z : float
            z coordinate of vertex.

        Returns
        -------
        float
            Rotated x coordinate of vertex with euler angles.
        """
        vertex_x -= self.occupancy_grid_msg.info.origin.position.x
        vertex_y -= self.occupancy_grid_msg.info.origin.position.y
        return (
            vertex_x
            * (
                self.cos_yaw * self.cos_pitch * self.cos_roll
                - self.sin_yaw * self.sin_roll
            )
            + vertex_y
            * (
                -self.sin_yaw * self.cos_pitch * self.cos_roll
                - self.cos_yaw * self.sin_roll
            )
            + vertex_z * (self.sin_pitch * self.cos_roll)
        ) + self.occupancy_grid_msg.info.origin.position.x

    def rotated_y(self, vertex_x: float, vertex_y: float, vertex_z: float) -> float:
        """
        Rotate y coordinate of vertex with euler angles.

        Parameters
        ----------
        vertex_x : float
            x coordinate of vertex.
        vertex_y : float
            y coordinate of vertex.
        vertex_z : float
            z coordinate of vertex.

        Returns
        -------
        float
            Rotated y coordinate of vertex with euler angles.
        """
        vertex_x -= self.occupancy_grid_msg.info.origin.position.x
        vertex_y -= self.occupancy_grid_msg.info.origin.position.y
        return (
            vertex_x
            * (
                self.cos_yaw * self.cos_pitch * self.sin_roll
                + self.sin_yaw * self.cos_roll
            )
            + vertex_y
            * (
                -self.sin_yaw * self.cos_pitch * self.sin_roll
                + self.cos_yaw * self.cos_roll
            )
            + vertex_z * (self.sin_pitch * self.sin_roll)
        ) + self.occupancy_grid_msg.info.origin.position.y

    def publish_colored_mesh(self) -> None:
        """
        Publish ground's OccupancyGrid topics.
        """
        try:
            dict_msg: dict = json.loads(self.tmsdbs[0].msg)
        except:
            # No ground data
            return

        self.occupancy_grid_msg: OccupancyGrid = db_util.document_to_msg(
            dict_msg, OccupancyGrid
        )

        self.calculate_euler()

        self.get_logger().info("start search vertices and change color")
        vertices: list = self.colored_mesh.vertices
        vertex_colors: list = self.colored_mesh.vertex_colors

        # # create open3d mesh with no ground data (for debug)
        # self.get_logger().info("start create open3d mesh")
        # mesh = o3d.geometry.TriangleMesh()
        # mesh.vertices = o3d.utility.Vector3dVector(
        #     [[vertex.x, vertex.y, vertex.z] for vertex in vertices]
        # )
        # mesh.vertex_colors = o3d.utility.Vector3dVector(
        #     [[color.r, color.g, color.b] for color in vertex_colors]
        # )
        # mesh.triangles = o3d.utility.Vector3iVector(
        #     [
        #         [
        #             triangle.vertex_indices[0],
        #             triangle.vertex_indices[1],
        #             triangle.vertex_indices[2],
        #         ]
        #         for triangle in self.colored_mesh.triangles
        #     ]
        # )

        # # show
        # o3d.visualization.draw_geometries([mesh])
        # self.get_logger().info("end")

        from_y: float = self.occupancy_grid_msg.info.origin.position.y

        resolution: float = self.occupancy_grid_msg.info.resolution
        grid_data: np.ndarray = np.array(self.occupancy_grid_msg.data).reshape(
            self.occupancy_grid_msg.info.height, self.occupancy_grid_msg.info.width
        )
        height: int = self.occupancy_grid_msg.info.height
        width: int = self.occupancy_grid_msg.info.width
        for row in range(height):
            to_y: float = from_y + resolution
            from_x: float = self.occupancy_grid_msg.info.origin.position.x

            # Find vertices within the range from_y to to_y
            relevant_vertices = []
            for i, vertex in enumerate(vertices):
                if from_y <= self.rotated_y(vertex.x, vertex.y, vertex.z) <= to_y:
                    relevant_vertices.append((i, vertex))

            for col in range(width):
                to_x: float = from_x + resolution

                # Change vertex colors with grid data for relevant vertices within the range from_x to to_x
                for i, vertex in relevant_vertices:
                    if from_x <= self.rotated_x(vertex.x, vertex.y, vertex.z) <= to_x:
                        grid_value = grid_data[row][col]
                        vertex_colors[i] = ColorRGBA(
                            r=float(1 - (grid_value / 100)),
                            g=1.0,
                            b=1.0,
                            a=1.0,
                        )

                from_x = to_x
            from_y = to_y

        # # create open3d mesh with ground data (for debug)
        # self.get_logger().info("start create open3d mesh")
        # mesh = o3d.geometry.TriangleMesh()
        # mesh.vertices = o3d.utility.Vector3dVector(
        #     [[vertex.x, vertex.y, vertex.z] for vertex in vertices]
        # )
        # mesh.vertex_colors = o3d.utility.Vector3dVector(
        #     [[color.r, color.g, color.b] for color in vertex_colors]
        # )
        # mesh.triangles = o3d.utility.Vector3iVector(
        #     [
        #         [
        #             triangle.vertex_indices[0],
        #             triangle.vertex_indices[1],
        #             triangle.vertex_indices[2],
        #         ]
        #         for triangle in self.colored_mesh.triangles
        #     ]
        # )

        # # show
        # o3d.visualization.draw_geometries([mesh])
        # self.get_logger().info("end")

        # Set vertex_colors only for optimization
        self.colored_mesh.triangles = []
        self.colored_mesh.vertices = []
        self.colored_mesh.vertex_colors = vertex_colors
        self.colored_mesh.vertex_normals = []

        self.publisher_.publish(self.colored_mesh)


def main(args=None):
    rclpy.init(args=args)

    tms_ur_ground_mesh_node = TmsUrGroundMesh()
    rclpy.spin(tms_ur_ground_mesh_node)

    tms_ur_ground_mesh_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
