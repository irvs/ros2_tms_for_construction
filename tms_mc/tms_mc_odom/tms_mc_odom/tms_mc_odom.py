from datetime import datetime
import json
import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from tms_msg_db.msg import Tmsdb

import tms_db_manager.tms_db_util as db_util


NODE_NAME = 'tms_mc_odom'
DATA_ID   = 11001
DATA_TYPE = 'machine'

class TmsMcOdom(Node):
    """Convert Odometry msg to Tmsdb msg and sent to tms_db_writer."""

    def __init__(self):
        super().__init__(NODE_NAME)
        self.declare_parameter('machine_name', 'machine_name')
        self.machine_name = self.get_parameter('machine_name').get_parameter_value().string_value

        self.publisher_ = self.create_publisher(Tmsdb, 'tms_db_data', 10)
        self.subscription = self.create_subscription(
            Odometry,
            '~/input/odom',
            self.send_odom_to_db_writer,
            10)

    def send_odom_to_db_writer(self, msg: Odometry) -> None:
        """
        Send topics to tms_db_writer (Write the received Odometry data to DB).

        parameters
        ----------
        msg : Odometry
            Target Object's Odometry.
        """
        db_msg = self.create_db_msg(msg)
        self.publisher_.publish(db_msg)

    def create_db_msg(self, msg: Odometry) -> Tmsdb:
        """
        Create Tmsdb msg from Odometry msg.

        Parameters
        ----------
        msg : Odometry
            Target Machine's Odometry msg.

        Returns
        -------
        tms_db_msg : Tmsdb
            Message containing Odometry msg sent to tms_db_writer. 
        """
        tms_db_msg                      = Tmsdb()
        tms_db_msg.time                 = datetime.now().isoformat()
        tms_db_msg.type                 = DATA_TYPE
        tms_db_msg.id                   = DATA_ID
        tms_db_msg.name                 = self.machine_name
        tms_db_msg.is_insert            = True

        # Convert Odometry msg to dictionary and then to json.
        doc: dict = db_util.msg_to_document(msg)
        tms_db_msg.msg: str = json.dumps(doc)

        return tms_db_msg


def main(args=None):
    rclpy.init(args=args)

    tms_mc_odom = TmsMcOdom()

    rclpy.spin(tms_mc_odom)

    tms_mc_odom.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()