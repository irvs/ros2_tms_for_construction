import rclpy
import time
import threading
from dataclasses import dataclass

from rclpy.action import ActionServer
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from diagnostic_msgs.msg import KeyValue

from tms_msg_ur.action import NextStepPermission


@dataclass
class GoalState:
    permission: bool = False
    tr_or_fa: str = ""
    event: threading.Event = threading.Event()


class TmsUrConstructionNextStepPermission(Node):

    def __init__(self):
        super().__init__('wait_ur_node')

        # goal_id → GoalState
        self.goal_states = {}
        self.goal_lock = threading.Lock()

        # Action Server
        self._action_server = ActionServer(
            self,
            NextStepPermission,
            'request_wait_ur',
            self.execute_callback
        )

        # permission subscriber
        self.subscription = self.create_subscription(
            KeyValue,
            'urpermission',
            self.listener_callback,
            10
        )

        # request publisher
        self.publisher_ = self.create_publisher(
            KeyValue,
            'permissionrequest',
            10
        )

    # permission topic callback
    def listener_callback(self, msg):
        goal_id_hex = msg.key  # key = goal_id.hex()

        with self.goal_lock:
            if goal_id_hex not in self.goal_states:
                self.get_logger().warn(
                    f'Permission for unknown goal: {goal_id_hex}'
                )
                return

            state = self.goal_states[goal_id_hex]
            state.tr_or_fa = msg.value
            state.permission = True
            state.event.set()

            self.get_logger().info(
                f'Permission received for goal {goal_id_hex}: {msg.value}'
            )

    # Action execution
    def execute_callback(self, goal_handle):
        goal_id_hex = bytes(goal_handle.goal_id.uuid).hex()

        self.get_logger().info(
            f'Execute goal: {goal_id_hex} machine={goal_handle.request.machinename}'
        )

        # goal専用状態を作成
        state = GoalState(event=threading.Event())

        with self.goal_lock:
            self.goal_states[goal_id_hex] = state

        # request publish（goal_id付き）
        msg = KeyValue()
        msg.key = goal_id_hex
        msg.value = goal_handle.request.machinename
        self.publisher_.publish(msg)

        # permission待ち
        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                self.get_logger().warn(f'Goal canceled: {goal_id_hex}')
                goal_handle.canceled()
                break

            # event wait (timeout付きでcancelチェック)
            if state.event.wait(timeout=0.5):
                if state.tr_or_fa == "true":
                    self.get_logger().info(f'Goal succeed: {goal_id_hex}')
                    goal_handle.succeed()
                else:
                    self.get_logger().info(f'Goal failed: {goal_id_hex}')
                    goal_handle.canceled()
                break

        # 後始末（重要）
        with self.goal_lock:
            if goal_id_hex in self.goal_states:
                del self.goal_states[goal_id_hex]

        result = NextStepPermission.Result()
        return result


def main(args=None):
    rclpy.init(args=args)
    node = TmsUrConstructionNextStepPermission()

    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()

    rclpy.shutdown()


if __name__ == '__main__':
    main()