# cmd_velをTwist -> Vector3へと変換する
import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3

subscribe_topic_name = "/cmd2vel"  # 変換前のトピック名
publish_topic_name = "/vel"  # 変換後のトピック名

class ConvertTwist2Vector3(Node):

    def __init__(self):
        super().__init__('convert_twist2vector3')
        self.subscription = self.create_subscription(
            Twist,
            subscribe_topic_name,
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

        self.publisher_ = self.create_publisher(Vector3, publish_topic_name, 10)

    def listener_callback(self, msg):
        print(f"[converter] << receive Twist: {msg}")
        send_msg = Vector3()
        send_msg.x = msg.linear.x
        send_msg.y = 0
        send_msg.z = msg.angular.z 
        print(f"[converter] >> send Vector3: {send_msg}")
        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    convert_node = ConvertTwist2Vector3()

    rclpy.spin(convert_node)

    convert_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()