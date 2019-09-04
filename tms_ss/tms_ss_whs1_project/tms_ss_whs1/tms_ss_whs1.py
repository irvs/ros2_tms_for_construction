# This is ROS2 version of ros_tms/tms_ss/tms_ss_whs1/src/main.cpp

import rclpy
from rclpy.node import Node

from std_msgs.msg import String

class TmsSsWhs1(Node):

    def __init__(self):
        super().__init__('tms_ss_whs1')
        self.db_pub = self.create_publisher(TmsDbStamped, "tms_db_data", 10)
        


def main(args=None):
    rclpy.init(args=args)

    tms_ss_whs1 = TmsSsWhs1()

    rclpy.spin(tms_ss_whs1)

    tms_ss_whs1.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()