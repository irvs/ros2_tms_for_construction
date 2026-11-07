# Copyright 2023, IRVS Laboratory, Kyushu University, Japan.
#
# Licensed under the Apache License, Version 2.0.
#
# You may not use this file except in compliance with the License.
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
import gridfs
import json
import pymongo
from time import sleep, monotonic

import rclpy
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import Marker

from rosidl_runtime_py.set_message import set_message_fields

import tms_db_manager.tms_db_util as db_util
from tms_msg_db.action import TmsdbGridFS


class TmsDbReaderGridFSActionServer(Node):
    """Read data from MongoDB using GridFS."""

    def __init__(self) -> None:
        super().__init__('tms_db_reader_gridfs')

        # Declare parameters
        self.declare_parameter('db_host', 'localhost')
        self.declare_parameter('db_port', 27017)

        # Get parameters
        self.db_host: str = (
            self.get_parameter('db_host')
            .get_parameter_value()
            .string_value
        )

        self.db_port: int = (
            self.get_parameter('db_port')
            .get_parameter_value()
            .integer_value
        )

        self.db = db_util.connect_db(
            'rostmsdb',
            self.db_host,
            self.db_port
        )

        self._action_server = ActionServer(
            self,
            TmsdbGridFS,
            'tms_db_reader_gridfs',
            self.db_reader_action_callback,
            callback_group=ReentrantCallbackGroup()
        )

    # --------------------------------------------------
    # Action Callback
    # --------------------------------------------------

    def db_reader_action_callback(self, goal_handle):

        self.request = goal_handle.request
        self.fs = gridfs.GridFS(self.db)

        if (
            self.request.type == 'static'
            or self.request.type == 'mesh'
            or self.request.type == 'dem'
        ):
            return self.static_or_mesh_terrain_handler(goal_handle)

        elif self.request.type == 'dynamic':
            return self.dynamic_terrain_handler(goal_handle)

        elif self.request.type == 'dynamic_marker':
            return self.dynamic_marker_handler(goal_handle)

        else:
            self.get_logger().error(
                "Please set a terrain type: "
                "'static', 'mesh', 'dem', "
                "'dynamic' or 'dynamic_marker'"
            )

            goal_handle.abort()

            result = TmsdbGridFS.Result()
            result.result = False

            return result

    # --------------------------------------------------
    # Static / Mesh / DEM
    # --------------------------------------------------

    def static_or_mesh_terrain_handler(self, goal_handle):

        file_obj = None

        while file_obj is None:

            if goal_handle.is_cancel_requested:
                goal_handle.canceled()

                result = TmsdbGridFS.Result()
                result.result = True

                return result

            file_obj = self.fs.find_one(
                {
                    'type': self.request.type,
                    'filename': self.request.filename
                },
                sort=[('time', pymongo.DESCENDING)]
            )

            if file_obj is None:
                sleep(0.1)

        self.get_logger().info(
            f'Create a {goal_handle.request.type} terrain file'
        )

        with open(self.request.filename, 'wb') as f:
            f.write(file_obj.read())

        goal_handle.succeed()

        result = TmsdbGridFS.Result()
        result.result = True

        return result

    # --------------------------------------------------
    # Dynamic PointCloud2 Handler
    # --------------------------------------------------

    def dynamic_terrain_handler(self, goal_handle):
        """
        Handler of dynamic PointCloud2 terrain.
        """

        if self.request.latest_only:
            # Handle latest data only when updated
            last_data_id = None
            next_check_time = monotonic()

            while True:
                try:
                    # Check cancellation
                    if goal_handle.is_cancel_requested:
                        goal_handle.canceled()
                        result = TmsdbGridFS.Result()
                        result.result = False
                        return result

                    # Get latest data from GridFS
                    latest_data = self.get_latest_dynamic_terrain_data(
                        goal_handle.request
                    )

                    # Publish only when new data is available
                    if latest_data is not None and latest_data._id != last_data_id:

                        feedback_msg = TmsdbGridFS.Feedback()

                        feedback_msg.pointcloud2 = PointCloud2()
                        feedback_msg.pointcloud2.header.frame_id = 'map'
                        feedback_msg.pointcloud2.height = latest_data.height
                        feedback_msg.pointcloud2.width = latest_data.width
                        feedback_msg.pointcloud2.fields = [
                            eval(field) for field in latest_data.fields
                        ]
                        feedback_msg.pointcloud2.is_bigendian = latest_data.is_bigendian
                        feedback_msg.pointcloud2.point_step = latest_data.point_step
                        feedback_msg.pointcloud2.row_step = latest_data.row_step
                        feedback_msg.pointcloud2.data = [
                            int(i) for i in latest_data.read().decode().split(',')
                        ]
                        feedback_msg.pointcloud2.is_dense = latest_data.is_dense

                        # Send feedback
                        goal_handle.publish_feedback(feedback_msg)

                        # Remember the published record
                        last_data_id = latest_data._id

                        self.get_logger().info(
                            'Published updated PointCloud2 terrain data.'
                        )

                except Exception as e:
                    self.get_logger().error(str(e))
                    goal_handle.abort()

                    result = TmsdbGridFS.Result()
                    result.result = False
                    return result

                # Check every 1 second
                next_check_time += 1.0
                sleep(max(0.0, next_check_time - monotonic()))

        else:

            # Handle all stored PointCloud2 data
            feedback_msg = TmsdbGridFS.Feedback()

            [
                time_list,
                height_list,
                width_list,
                fields_list,
                is_bigendian_list,
                point_step_list,
                row_step_list,
                data_list,
                is_dense_list,
            ] = self.get_all_dynamic_terrain_data()

            if not time_list:
                self.get_logger().warning(
                    'No dynamic PointCloud2 data found.'
                )

                goal_handle.succeed()

                result = TmsdbGridFS.Result()
                result.result = True

                return result

            while True:

                try:
                    pre_time = datetime.fromisoformat(time_list[0])

                    for (
                        str_time,
                        height,
                        width,
                        fields,
                        is_bigendian,
                        point_step,
                        row_step,
                        data,
                        is_dense,
                    ) in zip(
                        time_list,
                        height_list,
                        width_list,
                        fields_list,
                        is_bigendian_list,
                        point_step_list,
                        row_step_list,
                        data_list,
                        is_dense_list,
                    ):

                        if goal_handle.is_cancel_requested:
                            goal_handle.canceled()

                            result = TmsdbGridFS.Result()
                            result.result = True

                            return result

                        feedback_msg.pointcloud2 = PointCloud2()

                        feedback_msg.pointcloud2.header.frame_id = 'map'
                        feedback_msg.pointcloud2.height = height
                        feedback_msg.pointcloud2.width = width

                        feedback_msg.pointcloud2.fields = [
                            eval(field) for field in fields
                        ]

                        feedback_msg.pointcloud2.is_bigendian = (
                            is_bigendian
                        )

                        feedback_msg.pointcloud2.point_step = point_step
                        feedback_msg.pointcloud2.row_step = row_step

                        feedback_msg.pointcloud2.data = [
                            int(i) for i in data.split(',')
                        ]

                        feedback_msg.pointcloud2.is_dense = is_dense

                        # Time delta
                        now_time = datetime.fromisoformat(str_time)

                        td = now_time - pre_time

                        sleep(max(0.0, td.total_seconds()))

                        pre_time = now_time

                        # Feedback
                        goal_handle.publish_feedback(feedback_msg)

                except Exception as e:

                    self.get_logger().error(str(e))

                    goal_handle.abort()

                    result = TmsdbGridFS.Result()
                    result.result = False

                    return result

    # --------------------------------------------------
    # Dynamic Marker Handler
    # --------------------------------------------------

    def dynamic_marker_handler(self, goal_handle):
        """
        Handler of dynamic Marker terrain.
        """

        feedback_msg = TmsdbGridFS.Feedback()

        if self.request.latest_only:
            # Handle latest marker data only when updated
            last_data_id = None
            next_check_time = monotonic()

            while True:
                try:
                    # Check cancellation
                    if goal_handle.is_cancel_requested:
                        goal_handle.canceled()
                        result = TmsdbGridFS.Result()
                        result.result = False
                        return result

                    # Get latest data from GridFS
                    latest_data = self.get_latest_dynamic_terrain_data(
                        goal_handle.request
                    )

                    # Publish only when new data is available
                    if latest_data is not None and latest_data._id != last_data_id:

                        # Read JSON data from GridFS
                        marker_json = latest_data.read().decode('utf-8')
                        marker_dict = json.loads(marker_json)

                        # Convert JSON to Marker
                        marker_msg = Marker()
                        set_message_fields(marker_msg, marker_dict)

                        # Create feedback
                        feedback_msg = TmsdbGridFS.Feedback()
                        feedback_msg.marker = marker_msg

                        # Send feedback
                        goal_handle.publish_feedback(feedback_msg)

                        # Remember the published record
                        last_data_id = latest_data._id

                        self.get_logger().info(
                            'Published updated Marker terrain data.'
                        )

                except Exception as e:
                    self.get_logger().error(str(e))
                    goal_handle.abort()

                    result = TmsdbGridFS.Result()
                    result.result = False
                    return result

                # Check every 1 second
                next_check_time += 1.0
                sleep(max(0.0, next_check_time - monotonic()))

        else:

            # Handle all stored Marker data
            marker_data_list = self.get_all_dynamic_marker_data()

            if not marker_data_list:
                self.get_logger().warning(
                    'No dynamic Marker data found.'
                )

                goal_handle.succeed()

                result = TmsdbGridFS.Result()
                result.result = True

                return result

            while True:

                try:
                    pre_time = datetime.fromisoformat(
                        marker_data_list[0][0]
                    )

                    for str_time, marker_json in marker_data_list:

                        if goal_handle.is_cancel_requested:
                            goal_handle.canceled()

                            result = TmsdbGridFS.Result()
                            result.result = True

                            return result

                        # Convert JSON to Marker
                        feedback_msg.marker = (
                            self.convert_json_to_marker(marker_json)
                        )

                        # Time delta
                        now_time = datetime.fromisoformat(str_time)

                        td = now_time - pre_time

                        sleep(max(0.0, td.total_seconds()))

                        pre_time = now_time

                        # Feedback
                        goal_handle.publish_feedback(feedback_msg)

                except Exception as e:

                    self.get_logger().error(str(e))

                    goal_handle.abort()

                    result = TmsdbGridFS.Result()
                    result.result = False

                    return result

    # --------------------------------------------------
    # Convert JSON to Marker
    # --------------------------------------------------

    def convert_json_to_marker(self, marker_json: str) -> Marker:
        """
        Convert JSON data to visualization_msgs/Marker.
        """

        marker_dict = json.loads(marker_json)

        marker = Marker()

        set_message_fields(marker, marker_dict)

        return marker

    # --------------------------------------------------
    # Get Latest Dynamic PointCloud2
    # --------------------------------------------------

    def get_latest_dynamic_terrain_data(self, request):
        """
        Get latest dynamic PointCloud2 data from MongoDB.
        """

        latest_data = self.fs.find_one(
            {'type': request.type},
            sort=[('time', pymongo.DESCENDING)]
        )

        return latest_data

    # --------------------------------------------------
    # Get All Dynamic PointCloud2
    # --------------------------------------------------

    def get_all_dynamic_terrain_data(self) -> list:
        """
        Get all stored dynamic PointCloud2 data from MongoDB.
        """

        time_list = []
        height_list = []
        width_list = []
        fields_list = []
        is_bigendian_list = []
        point_step_list = []
        row_step_list = []
        data_list = []
        is_dense_list = []

        cursor = self.fs.find(
            {'type': self.request.type}
        ).sort(
            [('time', pymongo.ASCENDING)]
        )

        for data in cursor:

            time_list.append(data.time)
            height_list.append(data.height)
            width_list.append(data.width)
            fields_list.append(data.fields)
            is_bigendian_list.append(data.is_bigendian)
            point_step_list.append(data.point_step)
            row_step_list.append(data.row_step)
            data_list.append(data.read().decode())
            is_dense_list.append(data.is_dense)

        return [
            time_list,
            height_list,
            width_list,
            fields_list,
            is_bigendian_list,
            point_step_list,
            row_step_list,
            data_list,
            is_dense_list,
        ]

    # --------------------------------------------------
    # Get Latest Dynamic Marker
    # --------------------------------------------------

    def get_latest_dynamic_marker_data(self, request):
        """
        Get latest dynamic Marker data from MongoDB.
        """

        latest_data = self.fs.find_one(
            {'type': request.type},
            sort=[('time', pymongo.DESCENDING)]
        )

        return latest_data

    # --------------------------------------------------
    # Get All Dynamic Marker
    # --------------------------------------------------

    def get_all_dynamic_marker_data(self) -> list:
        """
        Get all stored dynamic Marker data from MongoDB.
        """

        marker_data_list = []

        cursor = self.fs.find(
            {'type': self.request.type}
        ).sort(
            [('time', pymongo.ASCENDING)]
        )

        for data in cursor:

            marker_json = data.read().decode('utf-8')

            marker_data_list.append(
                (data.time, marker_json)
            )

        return marker_data_list


def main(args=None):
    rclpy.init(args=args)

    action_server = TmsDbReaderGridFSActionServer()

    executor = MultiThreadedExecutor()

    try:
        executor.add_node(action_server)
        executor.spin()

    finally:
        executor.shutdown()
        action_server.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()