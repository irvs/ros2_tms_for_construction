import sys
import math

import rclpy

from rclpy.node import Node
from std_msgs.msg import String
from nmea_msgs.msg import Sentence
from sensor_msgs.msg import NavSatFix, NavSatStatus
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
import serial
import tms_ss_gnss.parser as parser


class NMEASerialReader(Node):
    def __init__(self):
        super().__init__('nmea_serial_reader')
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('frame_id', 'gnss')
        self.declare_parameter('sentence_topic_name', 'nmea_sentence')
        self.nmea_pub = self.create_publisher(Sentence, self.get_parameter('sentence_topic_name').value, 10)
        serial_port = self.get_parameter('port').value
        baud_rate = self.get_parameter('baud_rate').value
        self.serial = serial.Serial(serial_port, baud_rate, timeout=2)
        
    def read_serial(self):
        try:
            while rclpy.ok():
                data = self.serial.readline().strip()
                try:
                    if isinstance(data, bytes):
                        data = data.decode('ascii')
                except ValueError as e:
                    self.get_logger().warn(
                        "Value error, likely due to missing fields in the NMEA message. Error was: %s. " % e)

                sentence = Sentence()
                sentence.header.stamp = self.get_clock().now().to_msg()
                sentence.header.frame_id = self.get_parameter('frame_id').value
                sentence.sentence = data
                self.nmea_pub.publish(sentence)
        except KeyboardInterrupt:
            if self.serial:
                self.serial.close()
                sys.exit()

def main():
    rclpy.init()
    print('Hi from tms_ss_gnss.')

    nmea_serial_reader = NMEASerialReader()
    try:
       nmea_serial_reader.read_serial()
    finally:
        if nmea_serial_reader.serial:
            nmea_serial_reader.serial.close()
        nmea_serial_reader.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
