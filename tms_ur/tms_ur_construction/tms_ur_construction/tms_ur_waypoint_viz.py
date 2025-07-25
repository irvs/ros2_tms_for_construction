import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

from tms_msg_db.srv import TmsdbGetData

import tms_db_manager.tms_db_util as db_util
from functools import partial

NODE_NAME = "waypoint_visualizer"
DATA_ID = 3012 #2012
DATA_TYPE = "parameter"

class WaypointVisualizer(Node):
    def __init__(self):
        super().__init__(NODE_NAME)

        self.publisher_ = self.create_publisher(Marker, 'waypoint_markers', 10)
    #    self.timer = self.create_timer(1.0, self.timer_callback)

        # ウェイポイントの定義 (x, y, z)
        self.waypoints = [
            Point(x=1.0, y=1.0, z=0.0),
            Point(x=2.0, y=2.0, z=0.0),
            Point(x=3.0, y=1.0, z=0.0)
        ]



         # Declare parameters
        self.declare_parameter("latest", False)
        self.declare_parameter("machine_name", "machine_name")

        # Get parameters
        self.latest: bool = (
            self.get_parameter("latest").get_parameter_value().bool_value
        )
        self.machine_name: str = (
            self.get_parameter("machine_name").get_parameter_value().string_value
        )

        self.cli = self.create_client(TmsdbGetData, "tms_db_reader")
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("service not available, waiting again...")

        timer_period = 0.1
        self.call_timer = self.create_timer(timer_period, self.send_request)


    def send_request(self):
        """
        Send request to tms_db_reader to get PoseStamped data.
        """
        self.req = TmsdbGetData.Request()
        self.req.type = DATA_TYPE
        self.req.id = DATA_ID
        self.req.latest_only = self.latest
        self.req.name = self.machine_name

        future = self.cli.call_async(self.req)
        future.add_done_callback(partial(self.callback_response))

    
    def callback_response(self, future):
        """
        Set response from tms_db_reader.
        """
        try:
            self.res = future.result()
            self.tmsdbs = self.res.tmsdbs
            self.timer_callback()
        except:
            return

    
    def timer_callback(self):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "waypoints"
        marker.id = 0
        marker.type = Marker.SPHERE_LIST
        marker.action = Marker.ADD

        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2

        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        marker.pose.orientation.w = 1.0
        marker.points = self.waypoints

        self.publisher_.publish(marker)

def main(args=None):
    rclpy.init(args=args)
    node = WaypointVisualizer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
