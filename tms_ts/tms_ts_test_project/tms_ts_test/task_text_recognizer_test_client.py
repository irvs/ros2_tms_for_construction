import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class TaskTextRecognizerTestClient(Node):
    def __init__(self):
        super().__init__('task_text_recognizer_test_client')
        self.pub = self.create_publisher(String, 'tms_ts_text_recognizer', 10)

def main(args=None):
    rclpy.init(args=args)
    node = TaskTextRecognizerTestClient()

    while rclpy.ok():
        text = input('実行したい命令（日本語の文章でどうぞ）>>')
        send_data = String()
        send_data.data = text
        node.pub.publish(send_data)
        print('OK')
    
    node.destroy_node()
    rclpy.shutdown()