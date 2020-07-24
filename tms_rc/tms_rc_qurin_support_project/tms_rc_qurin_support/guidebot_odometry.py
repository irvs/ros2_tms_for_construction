## guidebot_odometry
# Robot端末からオドメトリを取得，
# TF(odom→base_footprint)，Odometryトピックを発行． はros2 dashing pythonではできない

import numpy as np
import quaternion  # pip install numpy-quaternion
import math

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
        
        self.x = 0.0  # x座標
        self.y = 0.0  # y座標
        self.theta = 0.0  # 角度
        self.isFirstCallback = True

    def listener_callback(self, data):
        # while data.pose.pose.orientation.z >  pi: data.pose.pose.orientation.z -= 2.0 * pi
        # while data.pose.pose.orientation.z < -pi: data.pose.pose.orientation.z += 2.0 * pi

        # quate = tf.transformations.quaternion_from_euler(0.0, 0.0, data.pose.pose.orientation.z)
        
        if self.isFirstCallback:
            self.time = data.header.stamp
            self.isFirstCallback = False
            return 
        
        # 速度・角速度
        self.last_time = self.time  # １つ前の時刻
        self.time = data.header.stamp  # 現在の時刻
        delta_time = (self.time.sec - self.last_time.sec) + \
            (self.time.nanosec - self.last_time.nanosec) * 10**-9
        nu = data.twist.twist.linear.x
        omega = data.twist.twist.angular.z

        t0 = self.theta  # １つ前の角度
        if omega == 0.0:
            self.x += nu * math.cos(t0) * delta_time
            self.y += nu * math.sin(t0) * delta_time
            self.theta += omega * delta_time
        else:
            self.x += nu/omega*(math.sin(t0 + omega*delta_time) - math.sin(t0))
            self.y += nu/omega*(-math.cos(t0 + omega*delta_time) + math.cos(t0))
            self.theta += omega * delta_time
        # {self.time.sec}: nu:{nu} omega:{omega}, 
        # print(f"{self.x} {self.y} {self.theta}")

        odom                         = Odometry()
        odom.header.stamp            = self.get_clock().now().to_msg()
        odom.header.frame_id         = self.odom_frame
        odom.child_frame_id          = self.base_frame  
        
        odom.pose.pose.position.x    = self.x
        odom.pose.pose.position.y    = self.y
        odom.pose.pose.position.z    = 0.0

        quat = quaternion.from_euler_angles([0.0, 0.0, self.theta])
        
        odom.pose.pose.orientation.x = quat.x
        odom.pose.pose.orientation.y = quat.y
        odom.pose.pose.orientation.z = quat.z
        odom.pose.pose.orientation.w = quat.w
          
        odom.twist.twist.linear.x    = nu
        odom.twist.twist.angular.z   = omega
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

