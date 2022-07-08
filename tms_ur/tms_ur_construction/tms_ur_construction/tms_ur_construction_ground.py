from datetime import datetime
import json
from time import sleep
from typing import Tuple
import rclpy
from rclpy.node import Node

from nav_msgs.msg import OccupancyGrid
from tms_msg_db.srv import TmsdbGetData

import tms_db_manager.tms_db_util as db_util


NODE_NAME = 'tms_ur_construction_ground' 
DATA_ID   = 3031 
DATA_TYPE = 'sensor'

class TmsUrConstructionGroundClient(Node):
    """Get ground's data from tms_db_reader."""

    def __init__(self):
        super().__init__(NODE_NAME)

        # parameter
        self.declare_parameter('latest', 'False')
        self.latest = self.get_parameter('latest').get_parameter_value().bool_value

        self.cli = self.create_client(TmsdbGetData, 'tms_db_reader')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = TmsdbGetData.Request()


    def send_request(self):
        """
        Send request to tms_db_reader to get ground data.
        
        Returns
        -------
        Any
            Result of request to tms_db_reader.
        """
        self.req.type        = DATA_TYPE
        self.req.id          = DATA_ID
        self.req.latest_only = self.latest
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()


class TmsUrConstructionGroundPublisher(Node):
    """Publish ground's OccupancyGrid."""

    def __init__(self, tmsdbs: list):
        super().__init__(NODE_NAME)

        # parameter
        self.declare_parameter('latest', 'False')
        self.latest = self.get_parameter('latest').get_parameter_value().bool_value

        self.tmsdbs = tmsdbs
        self.publisher_ = self.create_publisher(OccupancyGrid, '~/output/occupancy_grid', 10)
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.publish_occupancy_grid)

    def publish_occupancy_grid(self) -> None:
        """
        Publish ground's OccupancyGrid topics.
        """
        if self.latest:
            dict_msg = json.loads(self.tmsdbs[0].msg)
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


def run_client(args=None) -> Tuple[list, bool]:
    """
    Run a client node.

    Parameters
    ----------
    args : List[str]
        List of command line arguments.

    Returns
    -------
    list
        List of Tmsdb msg.
    bool
        Identifier indicating whether getting the latest data or debugging.
    """
    rclpy.init(args=args)

    tms_ur_construction_ground_client = TmsUrConstructionGroundClient()
    response = tms_ur_construction_ground_client.send_request()

    tms_ur_construction_ground_client.destroy_node()
    rclpy.shutdown()

    return response.tmsdbs, tms_ur_construction_ground_client.latest


def run_publisher(tmsdbs: list, latest: bool) -> None:
    """
    Run a publisher node.

    Parameters
    ----------
    list
        List of Tmsdb msg.
    bool
        Identifier indicating whether getting the latest data or debugging.
    """
    rclpy.init()
    tms_ur_construction_ground_publisher = TmsUrConstructionGroundPublisher(tmsdbs)

    if latest:
        # publish latest data
        rclpy.spin(tms_ur_construction_ground_publisher)
    else:
        # debugging
        rclpy.spin_once(tms_ur_construction_ground_publisher)

    tms_ur_construction_ground_publisher.destroy_node()
    rclpy.shutdown()


def main(args=None):
    # client
    tmsdbs, latest = run_client(args=args)

    # publisher
    run_publisher(tmsdbs, latest)


if __name__ == '__main__':
    main()