import sys
import math

import rclpy

from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix, NavSatStatus
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
import serial
import tms_ss_gnss.parser as parser


class NMEASerialDriver(Node):
    def __init__(self):
        super().__init__('nmea_serial_driver')

        self.current_fix = None
        self.current_gga = None

        self.declare_parameter('timer_preriod_serial', 0.1)
        self.declare_parameter('timer_preriod_fix', 0.1)
        self.declare_parameter('timer_preriod_gga', 0.1)
        
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('frame_id', 'gnss')
        self.declare_parameter('pub_sentence', False)

        self.declare_parameter('fix_topic_name', 'fix')
        self.declare_parameter('gga_topic_name', 'gga')
        self.declare_parameter('sentence_topic_name', 'sentence')


        # epe = estimated position error
        self.declare_parameter('epe_quality0', 1000000)
        self.declare_parameter('epe_quality1', 4.0)
        self.declare_parameter('epe_quality2', 0.1)
        self.declare_parameter('epe_quality4', 0.02)
        self.declare_parameter('epe_quality5', 4.0)
        self.declare_parameter('epe_quality9', 3.0)

        self.default_epe_quality0 = self.get_parameter('epe_quality0').value
        self.default_epe_quality1 = self.get_parameter('epe_quality1').value
        self.default_epe_quality2 = self.get_parameter('epe_quality2').value
        self.default_epe_quality4 = self.get_parameter('epe_quality4').value
        self.default_epe_quality5 = self.get_parameter('epe_quality5').value
        self.default_epe_quality9 = self.get_parameter('epe_quality9').value

        self.using_receiver_epe = False
        
        self.group = ReentrantCallbackGroup()
        timer_period_serial = self.get_parameter('timer_preriod_serial').value
        timer_period_fix = self.get_parameter('timer_preriod_fix').value
        timer_period_gga = self.get_parameter('timer_preriod_gga').value
        serial_port = self.get_parameter('port').value
        baud_rate = self.get_parameter('baud_rate').value
        self.timer_serial = self.create_timer(timer_period_serial, self.serial_callback, callback_group=self.group)
        self.timer_fix = self.create_timer(timer_period_fix, self.fix_callback, callback_group=self.group)
        self.timer_gga = self.create_timer(timer_period_gga, self.gga_callback, callback_group=self.group)
        self.serial = serial.Serial(serial_port, baud_rate, timeout=2)

        self.fix_publisher = self.create_publisher(NavSatFix, 'fix', 10)
        self.gga_publisher = self.create_publisher(String, 'gga', 10)
        self.sentence_publisher = self.create_publisher(String, 'sentence', 10)

        self.lon_std_dev = float("nan")
        self.lat_std_dev = float("nan")
        self.alt_std_dev = float("nan")

        """Format for this dictionary is the fix type from a GGA message as the key, with
        each entry containing a tuple consisting of a default estimated
        position error, a NavSatStatus value, and a NavSatFix covariance value."""
        self.gps_qualities = {
            # Unknown
            -1: [
                self.default_epe_quality0,
                NavSatStatus.STATUS_NO_FIX,
                NavSatFix.COVARIANCE_TYPE_UNKNOWN
            ],
            # Invalid
            0: [
                self.default_epe_quality0,
                NavSatStatus.STATUS_NO_FIX,
                NavSatFix.COVARIANCE_TYPE_UNKNOWN
            ],
            # SPS
            1: [
                self.default_epe_quality1,
                NavSatStatus.STATUS_FIX,
                NavSatFix.COVARIANCE_TYPE_APPROXIMATED
            ],
            # DGPS
            2: [
                self.default_epe_quality2,
                NavSatStatus.STATUS_SBAS_FIX,
                NavSatFix.COVARIANCE_TYPE_APPROXIMATED
            ],
            # RTK Fix
            4: [
                self.default_epe_quality4,
                NavSatStatus.STATUS_GBAS_FIX, # 2
                NavSatFix.COVARIANCE_TYPE_APPROXIMATED
            ],
            # RTK Float
            5: [
                self.default_epe_quality5,
                3,
                NavSatFix.COVARIANCE_TYPE_APPROXIMATED
            ],
            # WAAS
            9: [
                self.default_epe_quality9,
                NavSatStatus.STATUS_GBAS_FIX,
                NavSatFix.COVARIANCE_TYPE_APPROXIMATED
            ]
        }
    
    def fix_callback(self):
        if self.current_fix != None:
            self.fix_publisher.publish(self.current_fix)
    
    def gga_callback(self):
        if self.current_gga != None:
            self.gga_publisher.publish(self.current_gga)

    def serial_callback(self):
        try:
            sentence = self.serial.readline().strip()
            try:
                if isinstance(sentence, bytes):
                    sentence = sentence.decode('ascii')
            except ValueError as e:
                self.get_logger().warn(
                    "Value error, likely due to missing fields in the NMEA message. Error was: %s. " % e)

            parsed_sentence = parser.parse_nmea_sentence(sentence)
            if not parsed_sentence:
                self.get_logger().debug("Failed to parse NMEA sentence. Sentence was: %s" % sentence)
                return False
            
            if 'GGA' in parsed_sentence:
                gga_msg = String()
                gga_msg.data = sentence.replace('$', '')
                self.current_gga = gga_msg
                fix_msg = NavSatFix()
                fix_msg.header.frame_id = self.get_parameter('frame_id').value
                fix_msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_APPROXIMATED

                data = parsed_sentence['GGA']
                fix_type = data['fix_type']
                if not (fix_type in self.gps_qualities):
                    fix_type = -1
                gps_qual = self.gps_qualities[fix_type]
                default_epe = gps_qual[0]
                fix_msg.status.status = gps_qual[1]
                fix_msg.position_covariance_type = gps_qual[2]
                if fix_msg.status.status > 0:
                    self.valid_fix = True
                else:
                    self.valid_fix = False

                fix_msg.status.service = NavSatStatus.SERVICE_GPS
                latitude = data['latitude']
                if data['latitude_direction'] == 'S':
                    latitude = -latitude
                fix_msg.latitude = latitude

                longitude = data['longitude']
                if data['longitude_direction'] == 'W':
                    longitude = -longitude
                fix_msg.longitude = longitude

                # Altitude is above ellipsoid, so adjust for mean-sea-level
                altitude = data['altitude'] + data['mean_sea_level']
                fix_msg.altitude = altitude

                # use default epe std_dev unless we've received a GST sentence with epes
                if not self.using_receiver_epe or math.isnan(self.lon_std_dev):
                    self.lon_std_dev = default_epe
                if not self.using_receiver_epe or math.isnan(self.lat_std_dev):
                    self.lat_std_dev = default_epe
                if not self.using_receiver_epe or math.isnan(self.alt_std_dev):
                    self.alt_std_dev = default_epe * 2

                hdop = data['hdop']
                fix_msg.position_covariance[0] = (hdop * self.lon_std_dev) ** 2
                fix_msg.position_covariance[4] = (hdop * self.lat_std_dev) ** 2
                fix_msg.position_covariance[8] = (2 * hdop * self.alt_std_dev) ** 2  # FIXME

                self.current_fix = fix_msg

            elif 'GST' in parsed_sentence:
                data = parsed_sentence['GST']

                # Use receiver-provided error estimate if available
                self.using_receiver_epe = True
                self.lon_std_dev = data['lon_std_dev']
                self.lat_std_dev = data['lat_std_dev']
                self.alt_std_dev = data['alt_std_dev']
            
            if self.get_parameter('pub_sentence').value:
                sentence_msg = String()
                sentence.data = sentence
                self.sentence_publisher.publish(sentence_msg)

        except KeyboardInterrupt:
            if self.serial:
                self.serial.close()
                sys.exit()
                
def main():
    rclpy.init()
    print('Hi from tms_ss_gnss.')

    nmea_serial_driver = NMEASerialDriver()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(nmea_serial_driver)
    try:
        executor.spin()
    finally:
        nmea_serial_driver.serial.close()
        nmea_serial_driver.destroy_node()
        executor.shutdown()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
