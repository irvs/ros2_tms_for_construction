import os
import rclpy
from rclpy.node import Node
import tms_db_manager.tms_db_util as db_util
from tms_msg_db.msg import TmsdbCollision

class TmsDbWriterCollision(Node):
    """Write file data to MongoDB without using GridFS."""

    def __init__(self):
        super().__init__('tms_db_writer_collision')

        # Declare parameters
        self.declare_parameter('db_host', 'localhost')
        self.declare_parameter('db_port', 27017)

        # Get parameters
        self.db_host: str = self.get_parameter('db_host').get_parameter_value().string_value
        self.db_port: int = self.get_parameter('db_port').get_parameter_value().integer_value

        self.db = db_util.connect_db('rostmsdb', self.db_host, self.db_port)
        self.subscription = self.create_subscription(
            TmsdbCollision,
            'tms_db_collision',
            self.db_write_callback,
            10
        )

    def db_write_callback(self, msg: TmsdbCollision) -> None:
        """
        Store mesh file data directly into MongoDB.
        """
        collection = self.db["parameter"]
        self.save_collision_mesh(collection, msg)

    def save_collision_mesh(self, collection, msg: TmsdbCollision) -> None:
        """
        Save mesh file of static terrain directly into MongoDB.
        """

        with open(msg.filepath, 'rb') as f:
            file_data = f.read() 

        # Initialize
        collection.insert_one({
            "model_name": msg.model_name,
            "type": msg.type,
            "x": 8.0,
            "y": 0.0,
            "z": 0.0,
            "qx": 0.0,
            "qy": 0.0,
            "qz": 0.0,
            "qw": 1.0,
            "record_name": "collision_object_" + os.path.splitext(os.path.basename(msg.filepath))[0],
            "data": file_data
        })

        self.get_logger().info(f"Saved mesh {msg.filepath} to MongoDB")

        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)

    tms_db_writer_collision = TmsDbWriterCollision()
    rclpy.spin(tms_db_writer_collision)

    tms_db_writer_collision.destroy_node()

if __name__ == '__main__':
    main()
