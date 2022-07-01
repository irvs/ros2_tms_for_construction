import rclpy
import json
from rclpy.node import Node

import tms_db_manager.tms_db_util as db_util
from tms_msg_db.msg import Tmsdb


class TmsDbWriter(Node):
    """Write data to MongoDB."""

    def __init__(self):
        super().__init__("tms_db_writer")

        # declare parameter
        self.declare_parameter('db_host', 'localhost')
        self.db_host = self.get_parameter('db_host').get_parameter_value().string_value
        self.declare_parameter('db_port', 27017)
        self.db_port = self.get_parameter('db_port').get_parameter_value().integer_value

        self.db = db_util.connect_db('rostmsdb', self.db_host, self.db_port)
        self.drop_collections_first()
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

        # Convert json to dictionary.
        doc['msg'] = json.loads(msg.msg)

        collection = db_util.get_collection_by_id(self.db, doc['id'])
        if msg.is_insert:
            collection.insert_one(doc)
        else:
            collection.update_one(
                {'name': doc['name']},
                {'$set': doc},
                upsert=True
            )

    def drop_collections_first(self) -> None:
        """
        Drop collections before storing data to database.
        """
        collection_names = self.db.list_collection_names()
        for collection_name in collection_names:
            if collection_name not in ['default', 'now']:  # TODO Think undropped collections
                self.db.drop_collection(collection_name)



def main(args=None):
    rclpy.init(args=args)

    node = TmsDbWriter()
    
    rclpy.spin(node) 

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    # try:
    #     TmsDbWriter()
    #     rclpy.spin()
    # except rclpy.Exception:
    #     rclpy.loginfo("tms_db_writer node terminated.")
