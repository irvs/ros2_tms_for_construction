import rclpy
from rclpy.node import Node
from tf2_ros import TransformListener, Buffer
from geometry_msgs.msg import TransformStamped
import tf_transformations

class TfListenerNode(Node):
    def __init__(self):
        super().__init__('tf_listener_node')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.timer = self.create_timer(0.5, self.timer_callback)

    def timer_callback(self):
        from_frame = 'map'  # ワールド座標など
        to_frame = 'ic120_tf/base_link'  # ロボット本体の座標

        try:
            now = rclpy.time.Time()
            trans: TransformStamped = self.tf_buffer.lookup_transform(
                from_frame,
                to_frame,
                now,
                timeout=rclpy.duration.Duration(seconds=1.0)
            )
            x = trans.transform.translation.x
            y = trans.transform.translation.y
            z = trans.transform.translation.z

            self.get_logger().info(f'Robot position in {from_frame}: x={x}, y={y}, z={z}')
        except Exception as e:
            self.get_logger().warn(f'Could not get transform: {e}')

def main():
    rclpy.init()
    node = TfListenerNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

