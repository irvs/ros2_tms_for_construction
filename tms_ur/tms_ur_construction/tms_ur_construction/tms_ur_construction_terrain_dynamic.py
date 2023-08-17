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

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2

from tms_msg_db.action import TmsdbGridFS


NODE_NAME = 'tms_ur_construction_terrain_dynamic' 
DATA_ID   = 3031 
DATA_TYPE = 'dynamic'


class TmsUrConstructionTerrainDynamicClient(Node):
    """Get dynamic terrain data from tms_db_reader_gridfs."""

    def __init__(self) -> None:
        super().__init__(NODE_NAME)

        # Declare parameters
        self.declare_parameter('latest', False)

        # Get parameters
        self.latest: bool = self.get_parameter('latest').get_parameter_value().bool_value

        self.publisher_ = self.create_publisher(PointCloud2, '~/output/terrain/dynamic/pointcloud2', 10)
        self._action_client = ActionClient(self, TmsdbGridFS, 'tms_db_reader_gridfs')

    def send_goal(self) -> None:
        """
        Send goal to action server.
        """
        self.goal_msg             = TmsdbGridFS.Goal()
        self.goal_msg.type        = DATA_TYPE
        self.goal_msg.id          = DATA_ID
        self.goal_msg.latest_only = self.latest

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
        Callback to be executed when the task is done.

        Parameters
        ----------
        future
            Outcome of a task in the future.
        """
        pass

    def feedback_callback(self, feedback_msg) -> None:
        """
        Callback function for publishing PointCloud2 msg feedbacked from tms_db_reader_gridfs.

        Parameters
        ----------
        feedback_msg
            Feedback message.
        """
        feedback = feedback_msg.feedback
        if not feedback.pointcloud2:
            return

        self.publisher_.publish(feedback.pointcloud2)


def main(args=None):
    rclpy.init(args=args)

    action_client = TmsUrConstructionTerrainDynamicClient()
    action_client.send_goal()

    rclpy.spin(action_client)
    action_client.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()