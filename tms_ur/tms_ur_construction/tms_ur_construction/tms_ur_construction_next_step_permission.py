import rclpy
import time
from rclpy.action import ActionServer
from rclpy.node import Node

from tms_msg_ur.action import NextStepPermission
from diagnostic_msgs.msg import KeyValue
from rclpy.executors import MultiThreadedExecutor

class TmsUrConstructionNextStepPermission(Node):

    def __init__(self):
        super().__init__('st_wait_ur_node')
        ###
        self.permission = False
        self.taskname = "machine"
        self.tr_or_fa = "TorF"
        ###
        # Node=self を最初に指定
        self._action_server = ActionServer(
            self,
            NextStepPermission,
            'st_wait_ur_node',
            self.execute_callback
        )

        ###
        self.subscription = self.create_subscription(
            KeyValue,
            'urpermission',
            self.listener_callback,
            10
        )
        self.subscription
        ###

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.key)
        self.machine_name = msg.key
        self.tr_or_fa = msg.value
        if self.machine_name == self.taskname:
            self.permission = True
            self.get_logger().info(f'Subscribe permission : {self.machine_name}')
    ###
    def execute_callback(self, goal_handle):
        self.taskname = goal_handle.request.taskname
        self.get_logger().info(f'Executing goal: {self.taskname}')

        feedback = NextStepPermission.Feedback()
        result = NextStepPermission.Result()

        self.permission = False  # goal ごとにリセット

        while rclpy.ok():
            # cancel 対応（重要）
            if goal_handle.is_cancel_requested:
                self.get_logger().warn('Goal canceled')
                goal_handle.canceled()
                return result

            # 許可が来たら成功
            if self.permission and self.tr_or_fa == "true":
                self.get_logger().info('Permission received, succeed')
                goal_handle.succeed()
                return result
            
            elif self.permission and self.tr_or_fa == "false":
                self.get_logger().info('Permission received, failure')
                goal_handle.canceled()
                return result

            # feedback を返す
         #   feedback.status = 'waiting for permission'
        #    goal_handle.publish_feedback(feedback)

            time.sleep(0.5)  # 0.5秒周期

# def main(args=None):
#     rclpy.init(args=args)
#     action_server = TmsUrConstructionNextStepPermission()
#     rclpy.spin(action_server)
#     rclpy.shutdown()



def main(args=None):
    rclpy.init(args=args)
    node = TmsUrConstructionNextStepPermission()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()



if __name__ == '__main__':
    main()