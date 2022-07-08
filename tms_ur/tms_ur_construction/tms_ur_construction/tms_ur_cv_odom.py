from datetime import datetime
import json
from time import sleep
import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from tms_msg_db.srv import TmsdbGetData

import tms_db_manager.tms_db_util as db_util


NODE_NAME = 'tms_ur_cv_odom' 
DATA_ID   = 11001
DATA_TYPE = 'machine'

class TmsUrCvOdomClient(Node):
    """Get construction vehicle's Odometry data from tms_db_reader."""

    def __init__(self):
        super().__init__(NODE_NAME)

        # declare_parameter
        self.declare_parameter('latest', 'False')
        self.latest = self.get_parameter('latest').get_parameter_value().bool_value

        self.cli = self.create_client(TmsdbGetData, 'tms_db_reader')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = TmsdbGetData.Request()


    def send_request(self):
        """
        Send request to tms_db_reader to get Odometry data.
        
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


class TmsUrCvOdomPublisher(Node):
    """Publish construction vehicle's Odometry."""

    def __init__(self, tmsdbs: list, latest: bool):
        super().__init__(NODE_NAME)
        self.tmsdbs = tmsdbs
        self.latest = latest
        self.publisher_ = self.create_publisher(Odometry, '~/output/odom', 10)
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.publish_odom)

    def publish_odom(self) -> None:
        """
        Publish odom's Odometry topics.
        """
        if self.latest:
            dict_msg = json.loads(self.tmsdbs[0].msg)
            msg: Odometry = db_util.document_to_msg(dict_msg, Odometry)
            self.publisher_.publish(msg)
        else:
            try:
                pre_time = datetime.strptime(self.tmsdbs[0].time, '%Y-%m-%dT%H:%M:%S.%f')
            except:
                self.get_logger().info("no data")
                return

            for tmsdb in self.tmsdbs:
                dict_msg = json.loads(tmsdb.msg)
                msg: Odometry = db_util.document_to_msg(dict_msg, Odometry)

                # convert string to datetime
                now_time = datetime.strptime(tmsdb.time, '%Y-%m-%dT%H:%M:%S.%f')

                # time delta
                td = now_time - pre_time
                sleep(td.total_seconds())
                pre_time = now_time

                self.publisher_.publish(msg)


def main(args=None):
    # client
    rclpy.init(args=args)

    tms_ur_cv_odom_client = TmsUrCvOdomClient()
    response = tms_ur_cv_odom_client.send_request()
    latest = tms_ur_cv_odom_client.latest

    tms_ur_cv_odom_client.destroy_node()
    rclpy.shutdown()

    # publisher
    rclpy.init()

    if latest:
        # publish latest data
        tms_ur_cv_odom_publisher = TmsUrCvOdomPublisher(response.tmsdbs, True)
        rclpy.spin(tms_ur_cv_odom_publisher)
    else:
        # debugging
        tms_ur_cv_odom_publisher = TmsUrCvOdomPublisher(response.tmsdbs, False)
        rclpy.spin_once(tms_ur_cv_odom_publisher)

    tms_ur_cv_odom_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()