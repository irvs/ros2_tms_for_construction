from datetime import datetime
import gridfs
import pymongo
from time import sleep

import rclpy
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
import sensor_msgs
from sensor_msgs.msg import PointCloud2

import tms_db_manager.tms_db_util as db_util
from tms_msg_db.action import TmsdbGridFS


class TmsDbReaderGridFSActionServer(Node):
    """Read data from MongoDB using GridFS"""

    def __init__(self) -> None:
        super().__init__('tms_db_reader_gridfs')

        # Declare parameters
        self.declare_parameter('db_host', 'localhost')
        self.declare_parameter('db_port', 27017)

        # Get parameters
        self.db_host: str = self.get_parameter('db_host').get_parameter_value().string_value
        self.db_port: int = self.get_parameter('db_port').get_parameter_value().integer_value

        self.db: pymongo.database.Database  = db_util.connect_db('rostmsdb', self.db_host, self.db_port)

        self._action_server = ActionServer(
            self,
            TmsdbGridFS,
            'tms_db_reader_gridfs',
            self.db_reader_action_callback,
            callback_group=ReentrantCallbackGroup()
        )

    def db_reader_action_callback(self, goal_handle):
        """
        Action callback.

        Parameters
        ----------
        goal_handle
            Action goal handler.
        
        Returns
        -------
        result
            Action result.
        """
        self.request = goal_handle.request
        self.fs = gridfs.GridFS(self.db)

        if goal_handle.request.type == 'static' or goal_handle.request.type == 'mesh' or goal_handle.request.type == 'dem':
            return self.static_or_mesh_terrain_handler(goal_handle)
        elif goal_handle.request.type == 'dynamic':
            return self.dynamic_terrain_handler(goal_handle)
        else:
            self.get_logger().info("Please set a terrain type 'static' or 'dynamic'")

    def static_or_mesh_terrain_handler(self, goal_handle):
        """
        Handler of static or mesh terrain.

        Parameters
        ----------
        goal_handle
            Action goal handler.

        Returns
        -------
        result
            Action result.
        """
        file_obj = None
        while file_obj is None:
            # Get latest static terrain data
            file_obj = self.fs.find_one(
                {'type': self.request.type, 'filename': self.request.filename},
                sort=[('time', pymongo.DESCENDING)]
            )

            feedback_msg = TmsdbGridFS.Feedback()
            goal_handle.publish_feedback(feedback_msg)

        # Create file
        self.get_logger().info(f'Create a {goal_handle.request.type} terrain file')
        f = open(self.request.filename, 'wb')
        f.write(file_obj.read())
        f.close()

        goal_handle.succeed()
        result = TmsdbGridFS.Result()
        result.result = True
        return result

    def dynamic_terrain_handler(self, goal_handle):
        """
        Handler of dynamic terrain.

        Parameters
        ----------
        goal_handle
            Action goal handler.

        Returns
        -------
        result
            Action result.
        """
        if self.request.latest_only:
            # Handle latest data
            feedback_msg = TmsdbGridFS.Feedback()

            while True:
                try:
                    latest_data: gridfs.grid_file.GridOut = self.get_latest_dynamic_terrain_data(goal_handle.request)
                    if latest_data is None:
                        continue

                    feedback_msg.pointcloud2                 = PointCloud2()
                    feedback_msg.pointcloud2.header.frame_id = 'map'
                    feedback_msg.pointcloud2.height          = latest_data.height
                    feedback_msg.pointcloud2.width           = latest_data.width
                    feedback_msg.pointcloud2.fields          = [eval(field) for field in latest_data.fields]
                    feedback_msg.pointcloud2.is_bigendian    = latest_data.is_bigendian
                    feedback_msg.pointcloud2.point_step      = latest_data.point_step
                    feedback_msg.pointcloud2.row_step        = latest_data.row_step
                    feedback_msg.pointcloud2.data            = [int(i) for i in latest_data.read().decode().split(',')]
                    feedback_msg.pointcloud2.is_dense        = latest_data.is_dense

                    goal_handle.publish_feedback(feedback_msg)
                except Exception as e:
                    self.get_logger().info(str(e))
                    result = TmsdbGridFS.Result()
                    return result
        else:
            # Handle stored data
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
            while True:
                try:
                    pre_time: datetime = datetime.strptime(time_list[0], '%Y-%m-%dT%H:%M:%S.%f')

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
                        feedback_msg.pointcloud2                 = PointCloud2()
                        feedback_msg.pointcloud2.header.frame_id = 'map'
                        feedback_msg.pointcloud2.height          = height
                        feedback_msg.pointcloud2.width           = width
                        feedback_msg.pointcloud2.fields          = [eval(field) for field in fields]
                        feedback_msg.pointcloud2.is_bigendian    = is_bigendian
                        feedback_msg.pointcloud2.point_step      = point_step
                        feedback_msg.pointcloud2.row_step        = row_step
                        feedback_msg.pointcloud2.data            = [int(i) for i in data.split(',')]
                        feedback_msg.pointcloud2.is_dense        = is_dense

                        # Time delta
                        now_time: datetime = datetime.strptime(str_time, '%Y-%m-%dT%H:%M:%S.%f')
                        td: datetime = now_time - pre_time
                        sleep(td.total_seconds())
                        pre_time = now_time

                        # Feedback
                        goal_handle.publish_feedback(feedback_msg)
                except Exception as e:
                    self.get_logger().info(str(e))
                    result = TmsdbGridFS.Result()
                    return result

    def get_latest_dynamic_terrain_data(self, request) -> gridfs.grid_file.GridOut:
        """
        Get latest dynamic terrain data from MongoDB using GridFS.

        Parameters
        ----------
        request
            Request from action client. 
            This use request instead of self.request because self.request will be cached and you cannot get latest data.

        Returns
        -------
        gridfs.grid_file.GridOut
            Latest dynamic terrain data.
        """
        latest_data = self.fs.find_one(
            {'type': request.type},
            sort=[('time', pymongo.DESCENDING)]
        )
        return latest_data

    def get_all_dynamic_terrain_data(self) -> list:
        """
        Get all stored dynamic terrain data from MongoDB using GridFS.

        Returns
        -------
        list
            List of stored dynamic terrain's PointCloud2 data.
        """
        time_list         = []
        height_list       = []
        width_list        = []
        fields_list       = []
        is_bigendian_list = []
        point_step_list   = []
        row_step_list     = []
        data_list         = []
        is_dense_list     = []
        
        [(
            time_list.append(data.time),
            height_list.append(data.height),
            width_list.append(data.width),
            fields_list.append(data.fields),
            is_bigendian_list.append(data.is_bigendian),
            point_step_list.append(data.point_step),
            row_step_list.append(data.row_step),
            data_list.append(data.read().decode()),
            is_dense_list.append(data.is_dense),
        ) for data in self.fs.find({'type': self.request.type}).sort([('time', pymongo.ASCENDING)])]

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


def main(args=None):
    rclpy.init(args=args)
    
    action_server = TmsDbReaderGridFSActionServer()
    executor = MultiThreadedExecutor()
    rclpy.spin(action_server, executor=executor)

    action_server.destroy()

    rclpy.shutdown()


if __name__ == '__main__':
	main()
