from datetime import datetime
import json
import rclpy
from rclpy.node import Node

from nav_msgs.msg import OccupancyGrid
from tms_msg_db.msg import Tmsdb

import tms_db_manager.tms_db_util as db_util


NODE_NAME = 'tms_sd_ground'
DATA_ID   = 3031 
DATA_TYPE = 'sensor'

class TmsSdGround(Node):
    """Convert OccupancyGrid msg to Tmsdb msg and sent to tms_db_writer."""

    def __init__(self):
        super().__init__(NODE_NAME)
        self.declare_parameter('ground_name', 'ground_name')
        self.ground_name = self.get_parameter('ground_name').get_parameter_value().string_value

        self.publisher_   = self.create_publisher(Tmsdb, 'tms_db_data', 10)
        self.subscription = self.create_subscription(
            OccupancyGrid,
            '~/input/occupancy_grid',
            self.send_occupancy_grid_to_db_writer,
            10)

    def send_occupancy_grid_to_db_writer(self, msg: OccupancyGrid) -> None:
        """
        Send topics to tms_db_writer (Write the received OccupancyGrid data to DB).

        parameters
        ----------
        msg : OccupancyGrid
            Target Object's OccupancyGrid.
        """
        db_msg = self.create_db_msg(msg)
        self.publisher_.publish(db_msg)

    def create_db_msg(self, msg: OccupancyGrid) -> Tmsdb:
        """
        Create Tmsdb msg from OccupancyGrid msg.

        Parameters
        ----------
        msg : OccupancyGrid
            Target Machine's OccupancyGrid msg.

        Returns
        -------
        tms_db_msg : Tmsdb
            Message containing OccupancyGrid msg sent to tms_db_writer. 
        """
        tms_db_msg           = Tmsdb()
        tms_db_msg.time      = datetime.now().isoformat()
        tms_db_msg.type      = DATA_TYPE
        tms_db_msg.id        = DATA_ID
        tms_db_msg.name      = self.ground_name
        tms_db_msg.is_insert = True

        # Convert OccupancyGrid msg to dictionary and then to json.
        doc: dict           = db_util.msg_to_document(msg)
        tms_db_msg.msg: str = json.dumps(doc)

        return tms_db_msg


def main(args=None):
    rclpy.init(args=args)

    tms_sd_ground = TmsSdGround()

    rclpy.spin(tms_sd_ground)

    tms_sd_ground.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
