from tms_msg_ts.srv import TsReq
import rclpy
from rclpy.node import Node

class TaskManagerTestClient(Node):

    def __init__(self):
        super().__init__('task_manager_test_client')
        self.cli = self.create_client(TsReq, 'task_viewer')
        # self.cli = self.create_client(TsReq, 'tms_ts_master')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = TsReq.Request()

    def send_request(self):
        self.req.task_id = 8003  # patrol
        self.req.robot_id = 2003  # smartpal
        self.req.object_id = 7001  # chipstar
        self.req.place_id = 6004  # chair
        self.future = self.cli.call_async(self.req)


def main(args=None):
    rclpy.init(args=args)

    task_manager_test_client = TaskManagerTestClient()
    task_manager_test_client.send_request()

    while rclpy.ok():
        rclpy.spin_once(task_manager_test_client)
        if task_manager_test_client.future.done():
            if task_manager_test_client.future.result() is not None:
                response = task_manager_test_client.future.result()
                print("send")
            else:
                print("fault")
            break

    task_manager_test_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()