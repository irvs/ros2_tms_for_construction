import json
import pymongo

import rclpy
from rclpy.node import Node

import tms_db_manager.tms_db_util as db_util
from tms_msg_db.msg import Tmsdb


class TmsDbWriter(Node):
    """Write data to MongoDB."""

    def __init__(self):
        super().__init__("tms_db_writer")

        # Declare parameters
        self.declare_parameter('db_host', 'localhost')
        self.declare_parameter('db_port', 27017)
        self.declare_parameter('init_db', False)

        # Get parameters
        self.db_host: str  = self.get_parameter('db_host').get_parameter_value().string_value
        self.db_port: int  = self.get_parameter('db_port').get_parameter_value().integer_value
        self.init_db: bool = self.get_parameter('init_db').get_parameter_value().bool_value

        self.db = db_util.connect_db('rostmsdb', self.db_host, self.db_port)
        if self.init_db:
            self.reset_db()
        self.subscription = self.create_subscription(
            Tmsdb,
            "tms_db_data",
            self.db_write_callback,
            10)

    def db_write_callback(self, msg: Tmsdb) -> None:
        """
        Store data.

        Parameters
        ----------
        msg : Tmsdb
            An instance of a ROS2 custom message to store data.
        """
        doc: dict = db_util.msg_to_document(msg)

        # convert json to dictionary.
        doc['msg'] = json.loads(msg.msg)

        collection: pymongo.collection.Collection = self.db[doc['type']]

        db_util.set_time_index(collection)

        collection.insert_one(doc)

    def reset_db(self) -> None:
        """
        Drop collections before storing data to database.
        """
        collection_names: list[str] = self.db.list_collection_names()
        for collection_name in collection_names:
            self.db.drop_collection(collection_name)



def main(args=None):
    rclpy.init(args=args)

    node = TmsDbWriter()
    
    rclpy.spin(node) 

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
