import math

import rclpy

from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix, NavSatStatus
import datetime

class DummyNode(Node):
    def __init__(self):
        super().__init__('dummy_node')

        timer_period_pub = 0.1
        self.gpgga_publisher = self.create_publisher(String, 'gga', 10)
        self.timer_pub = self.create_timer(timer_period_pub, self.pub_callback)

    def latlng2GPGGA(self, lat, lng, alt, now):
        flagN = "N"
        flagE = "E"
        if lng > 180:
            lng = (lng-360) * -1
            flagE = "W"
        elif (lng < 0 and lng >= -180):
            lng = lng * -1
            flagE = "W"
        elif lng < -180:
            lng = lng + 360
            flagE = "E"

        if lat < 0:
            lat = lat * -1
            flagN = "S"
        lngDeg = int(lng)
        latDeg = int(lat)
        lngMin = (lng - lngDeg) * 60
        latMin = (lat - latDeg) * 60
        gpgga = "GPGGA,%02d%02d%04.2f,%02d%011.8f,%1s,%03d%011.8f,%1s,1,05,0.19,+00400,M,%5.3f,M,," % \
            (now.hour,now.minute,now.second,latDeg,latMin,flagN,lngDeg,lngMin,flagE,alt)
        
        return gpgga

    def pub_callback(self):
        now = datetime.datetime.utcnow()
        ##########################
        # test data
        ##########################
    
        test_lat = 33.5955114617
        test_lng = 130.219019033
        test_alt = 80.073
        gpggaString = self.latlng2GPGGA(test_lat, test_lng, test_alt, now)
        dummy_msg = String()
        dummy_msg.data = gpggaString

        self.gpgga_publisher.publish(dummy_msg)


def main():
    rclpy.init()
    print('Hi from dummy_pub.')

    dummy_node = DummyNode()
    try:
        rclpy.spin(dummy_node)
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()