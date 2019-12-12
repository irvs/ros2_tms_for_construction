import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time
from tms_msg_ts.srv import TaskTextRecognize

class TaskTextRecognizerTestClient(Node):
    def __init__(self):
        super().__init__('task_text_recognizer_test_client')
        self.cli = self.create_client(TaskTextRecognize, "tms_ts_text_recognizer")


def main(args=None):
    rclpy.init(args=args)
    node = TaskTextRecognizerTestClient()

    while rclpy.ok():
        # text = input('実行したい命令（日本語の文章でどうぞ）>>')
        send_data = TaskTextRecognize.Request()
        #send_data.data = text
        send_data.data = "ダブル、センサーに向かって"
        send_data.is_announce = False
        node.cli.call_async(send_data)

        print(f'{send_data.data} : OK')

        time.sleep(5.0)
    
    node.destroy_node()
    rclpy.shutdown()