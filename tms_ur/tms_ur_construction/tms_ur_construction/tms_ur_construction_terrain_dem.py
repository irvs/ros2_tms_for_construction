import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from nav_msgs.msg import OccupancyGrid

from tms_msg_db.srv import DemSrv
from tms_msg_db.action import TmsdbGridFS


NODE_NAME = "tms_ur_construction_terrain_dem"
DATA_ID = 3030
DATA_TYPE = "dem"


class TmsUrConstructionTerrainDemClient(Node):
    """Get terrain's dem data from tms_db_reader_gridfs."""

    def __init__(self):
        super().__init__(NODE_NAME)

        # Declare parameters
        self.declare_parameter("filename_dem", "filename_dem.npy")
        self.declare_parameter("resolution", 0.1)  # [m]

        # Get parameters
        self.filename_dem: str = (
            self.get_parameter("filename_dem").get_parameter_value().string_value
        )
        self.resolution: float = (
            self.get_parameter("resolution").get_parameter_value().double_value
        )

        # Action Client
        self._action_client = ActionClient(self, TmsdbGridFS, "tms_db_reader_gridfs")

    def send_goal(self) -> None:
        """
        Send goal to action server.
        """
        self.goal_msg = TmsdbGridFS.Goal()
        self.goal_msg.type = DATA_TYPE
        self.goal_msg.id = DATA_ID
        self.goal_msg.filename = self.filename_dem

        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(
            self.goal_msg, feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)

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
            return

        # Goal accepted
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future) -> None:
        """
        Get result call back from action server and create OccupancyGrid msg for service callback.

        Parameters
        ----------
        future
            Outcome of a task in the future.
        """
        result = future.result().result
        if not result.result:
            return

        # Create OccupancyGrid msg
        self.msg: OccupancyGrid = self.create_msg()

        self.get_logger().info("DEM service is ready")

        # Service server
        self.srv = self.create_service(
            DemSrv, "~/output/terrain/dem_srv", self.terrain_dem_srv_callback
        )

    def terrain_dem_srv_callback(self, request, response):
        """
        Callback function.

        Returns
        -------
        response
            Service callback response.
        """
        response.occupancy_grid = self.msg
        self.get_logger().info("Return a response of DEM")
        return response

    def create_msg(self) -> OccupancyGrid:
        """
        Create OccupancyGrid msg from a .npy file.

        Returns
        -------
        msg : OccupancyGrid
            Terrain's DEM data.
        """
        self.dem: np.ndarray = np.load(self.filename_dem)
        dem_shape = self.dem.shape  # (?,?)

        data = np.ravel(self.dem).tolist()  # 2D [[]] -> 1D []

        msg = OccupancyGrid()
        msg.header.frame_id = "map"
        msg.data = data
        msg.info.resolution = self.resolution
        msg.info.width = dem_shape[1]
        msg.info.height = dem_shape[0]
        msg.info.origin.position.x = -(dem_shape[1] * self.resolution) / 2
        msg.info.origin.position.y = -(dem_shape[0] * self.resolution) / 2
        msg.info.origin.position.z = 0.0

        return msg


def main(args=None):
    rclpy.init(args=args)

    action_client = TmsUrConstructionTerrainDemClient()
    action_client.send_goal()
    rclpy.spin(action_client)

    action_client.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
