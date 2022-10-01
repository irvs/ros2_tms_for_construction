from datetime import datetime
from functools import partial
import json
from time import sleep
import rclpy
from rclpy.node import Node

from nav_msgs.msg import OccupancyGrid
from tms_msg_db.srv import TmsdbGetData

import tms_db_manager.tms_db_util as db_util


NODE_NAME = 'tms_ur_construction_ground' 
DATA_ID   = 3031 
DATA_TYPE = 'sensor'


class TmsUrConstructionGroundNode(Node):
    """Get ground's data from tms_db_reader."""

    def __init__(self):
        super().__init__(NODE_NAME)

        # parameter
        self.declare_parameter('latest', 'False')
        self.latest = self.get_parameter('latest').get_parameter_value().bool_value

        self.publisher_ = self.create_publisher(OccupancyGrid, '~/output/occupancy_grid', 10)
        timer_period = 0.1
        self.call_timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        """
        Get OccupancyGrid data from tms_db_reader and publish them.
        """
        self.send_request()

        try:
            self.tmsdbs = self.res.tmsdbs
        except:
            return
        self.publish_occupancy_grid()

    def send_request(self):
        """
        Send request to tms_db_reader to get ground data.
        
        Returns
        -------
        Any
            Result of request to tms_db_reader.
        """
        self.cli = self.create_client(TmsdbGetData, 'tms_db_reader')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

        self.req = TmsdbGetData.Request()
        self.req.type        = DATA_TYPE
        self.req.id          = DATA_ID
        self.req.latest_only = self.latest
        future = self.cli.call_async(self.req)
        future.add_done_callback(partial(self.callback_set_response))

    def callback_set_response(self, future):
        """
        Set response from tms_db_reader.
        """
        self.res = future.result()

    def publish_occupancy_grid(self) -> None:
        """
        Publish ground's OccupancyGrid topics.
        """
        if self.latest:
            try:
                dict_msg = json.loads(self.tmsdbs[0].msg)
            except:
                self.get_logger().info("no data")
                return
                
            msg: OccupancyGrid = db_util.document_to_msg(dict_msg, OccupancyGrid)
            self.publisher_.publish(msg)
        else:
            try:
                pre_time = datetime.strptime(self.tmsdbs[0].time, '%Y-%m-%dT%H:%M:%S.%f')
            except:
                self.get_logger().info("no data")
                return

            for tmsdb in self.tmsdbs:
                dict_msg = json.loads(tmsdb.msg)
                msg: OccupancyGrid = db_util.document_to_msg(dict_msg, OccupancyGrid)

                # convert string to datetime
                now_time = datetime.strptime(tmsdb.time, '%Y-%m-%dT%H:%M:%S.%f')

                # time delta
                td = now_time - pre_time
                sleep(td.total_seconds())
                pre_time = now_time

                self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    tms_ur_construction_ground_node = TmsUrConstructionGroundNode()
    rclpy.spin(tms_ur_construction_ground_node)

    tms_ur_construction_ground_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()