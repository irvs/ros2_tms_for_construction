## guidebot_odometry
# Robot端末からオドメトリを取得，
# TF(odom→base_footprint)，Odometryトピックを発行． はros2 dashing pythonではできない

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry


class GuidebotOdometry(Node):

    def __init__(self):
        super().__init__('guidebot_odometry')
        self.odom_frame = "odom"
        self.base_frame = "base_footprint"
        self.odom_pub = self.create_publisher(Odometry, '/odometry/wheel', 10)
        self.subscription = self.create_subscription(
            Odometry,
            '/hapirobo/loomo_odd',
            self.listener_callback,
            10)

    def listener_callback(self, data):
        # while data.pose.pose.orientation.z >  pi: data.pose.pose.orientation.z -= 2.0 * pi
        # while data.pose.pose.orientation.z < -pi: data.pose.pose.orientation.z += 2.0 * pi

        # quate = tf.transformations.quaternion_from_euler(0.0, 0.0, data.pose.pose.orientation.z)
        
        odom                         = Odometry()
        odom.header.stamp            = self.get_clock().now().to_msg()
        odom.header.frame_id         = self.odom_frame
        odom.child_frame_id          = self.base_frame
        #odom.pose.pose.position.x    = data.pose.pose.position.x
        #odom.pose.pose.position.y    = data.pose.pose.position.y

        
        odom.pose.pose.orientation.x = 0.0  # data.pose.pose.orientation.x
        odom.pose.pose.orientation.y = 0.0  # data.pose.pose.orientation.y
        odom.pose.pose.orientation.z = 0.0  # data.pose.pose.orientation.z
        odom.pose.pose.orientation.w = 1.0  # data.pose.pose.orientation.w
        

        odom.twist.twist.linear.x    = data.twist.twist.linear.x 
        odom.twist.twist.angular.z   = data.twist.twist.angular.z
    
        self.odom_pub.publish(odom)

def main(args=None):
    rclpy.init(args=args)

    guidebot_odometry = GuidebotOdometry()

    rclpy.spin(guidebot_odometry)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    guidebot_odometry.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

