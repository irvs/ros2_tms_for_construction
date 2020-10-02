"""
The Pozyx ready to localize tutorial (c) Pozyx Labs
Please read the tutorial that accompanies this sketch:
https://www.pozyx.io/Documentation/Tutorials/ready_to_localize/Python
This tutorial requires at least the contents of the Pozyx Ready to Localize kit. It demonstrates the positioning capabilities
of the Pozyx device both locally and remotely. Follow the steps to correctly set up your environment in the link, change the
parameters and upload this sketch. Watch the coordinates change as you move your device around!
"""
from time import sleep

import pypozyx
from pypozyx import (PozyxConstants, Coordinates, POZYX_SUCCESS, PozyxRegisters, version,
                     DeviceCoordinates, PozyxSerial, get_first_pozyx_serial_port, SingleRegister, Quaternion)
from pythonosc.udp_client import SimpleUDPClient

from pypozyx.tools.version_check import perform_latest_version_check
from pypozyx.structures.device import DeviceList


import rclpy
from std_msgs.msg import String
from nav_msgs.msg import Odometry
# from geometry_msgs.msg import Quaternion
from visualization_msgs.msg import Marker, MarkerArray
import paho.mqtt.client as mqtt
import ssl
import json
import pprint
import datetime


class MultitagPositioning(object):
    """Continuously performs multitag positioning"""

    def __init__(self, pozyx, osc_udp_client, tag_ids, anchors, algorithm=PozyxConstants.POSITIONING_ALGORITHM_UWB_ONLY,
                 dimension=PozyxConstants.DIMENSION_3D, height=1000, node=None):
        self.pozyx = pozyx
        self.osc_udp_client = osc_udp_client

        self.tag_ids = tag_ids
        self.anchors = anchors
        self.algorithm = algorithm
        self.dimension = dimension
        self.height = height
        self.node = node
        self.count_log = 0

    def setup(self):
        """Sets up the Pozyx for positioning by calibrating its anchor list."""
        print("------------POZYX MULTITAG POSITIONING V{} -------------".format(version))
        print("")
        print(" - System will manually calibrate the tags")
        print("")
        print(" - System will then auto start positioning")
        print("")
        if None in self.tag_ids:
            for device_id in self.tag_ids:
                self.pozyx.printDeviceInfo(device_id)
        else:
            for device_id in [None] + self.tag_ids:
                self.pozyx.printDeviceInfo(device_id)
        print("")
        print("------------POZYX MULTITAG POSITIONING V{} -------------".format(version))
        print("")

        self.setAnchorsManual(save_to_flash=False)

        self.printPublishAnchorConfiguration()

    def loop(self):
        """Performs positioning and prints the results."""
        for tag_id in self.tag_ids:
            position = Coordinates()
            status = self.pozyx.doPositioning(
                position, self.dimension, self.height, self.algorithm, remote_id=tag_id)
            quat = Quaternion()
            status &= self.pozyx.getNormalizedQuaternion(quat, tag_id)
            if status == POZYX_SUCCESS:
                self.printPublishPosition(position, tag_id, quat)
            else:
                self.printPublishErrorCode("positioning", tag_id)

    def printPublishPosition(self, position, network_id, quat):
        """Prints the Pozyx's position and possibly sends it as a OSC packet"""
        global pub, markers_pub, anchors
        if network_id is None:
            network_id = 0
        s = "POS ID: {}, x(mm): {}, y(mm): {}, z(mm): {}".format("0x%0.4x" % network_id,
                                                                 position.x, position.y, position.z)

        if "0x6e04" == ("0x%0.4x"%network_id):
            print(s)
            odom = Odometry()
            odom.header.stamp = self.node.get_clock().now().to_msg()
            odom.header.frame_id = "pozyx"
            # odom.child_frame_id = "pozyx"
            odom.pose.pose.position.x = position.x * 0.001
            odom.pose.pose.position.y = position.y * 0.001
            odom.pose.pose.position.z = position.z * 0.001
            

            odom.pose.pose.orientation.x = quat.x
            odom.pose.pose.orientation.y = quat.y
            odom.pose.pose.orientation.z = quat.z
            odom.pose.pose.orientation.w = quat.w
            
            
            # row = 0  # 行
            # col = 8  # 列
            # height_l = "200cm"

            # with open(f"/home/common/pozyx_0824_2D_{height_l}_{row}_{col}.csv", mode='a') as f:
            #     f.write(f'{row}, {col}, {datetime.datetime.now().isoformat()}, {position.x*0.001}, {position.y*0.001}, {position.z*0.001}, {quat.x}, {quat.y}, {quat.z}, {quat.w}\n')
            # print(f"{self.count_log} datas")
            # self.count_log += 1

            # odom.pose.cov ariance = [
            #     0.14408971883333066, 0.0, 0.0, 0.0, 0.0, 0.0,\
            #     0.0, 0.06944361656077998, 0.0, 0.0, 0.0, 0.0,\
            #     0.0, 0.0, 0.0, 0.0, 0.0, 0.0,\
            #     0.0, 0.0, 0.0, 0.0, 0.0, 0.0,\
            #     0.0, 0.0, 0.0, 0.0, 0.0, 0.0,\
            #     0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            #     ]
            odom.pose.covariance = [
                1.0, 0.0, 0.0, 0.0, 0.0, 0.0,\
                0.0, 1.0, 0.0, 0.0, 0.0, 0.0,\
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0,\
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0,\
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0,\
                0.0, 0.0, 0.0, 0.0, 0.0, 0.03,
                ]
            pub.publish(odom)
            self.publishMarkerArray(anchors)

        if self.osc_udp_client is not None:
            self.osc_udp_client.send_message(
                "/position", [network_id, position.x, position.y, position.z])

    def publishMarkerArray(self, anchors):
        """
        Rviz上でPozyxアンカーの位置を表示する。

        Parameters
        ----------
        anchors : list<Anchor>

        """
        # for tag_id in self.tag_ids:
        #     list_size = SingleRegister()  
        #     self.pozyx.getNumberOfAnchors(list_size)  
        #     print(list_size)
        #     anchor_list = DeviceList(list_size=4)
        #     self.pozyx.getPositioningAnchorIds(anchor_list)  # , remote_id=tag_id)
        #     print(f'anchors for tag{"0x%0.4x" % tag_id} : {anchor_list}')


        markerArray = MarkerArray()
        m_id = 0
        for a in anchors:
            marker = Marker()
            marker.header.frame_id = "/pozyx"
            marker.header.stamp = self.node.get_clock().now().to_msg()
            marker.ns = "pozyx_local"  # namespace
            marker.id = m_id
            m_id += 1

            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.lifetime.sec = 100
            marker.scale.x = 0.3
            marker.scale.y = 0.3
            marker.scale.z = 0.3
            marker.pose.position.x = a.pos.x / 1000
            marker.pose.position.y = a.pos.y / 1000
            marker.pose.position.z = a.pos.z / 1000
            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = 0.0
            marker.pose.orientation.w = 1.0
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.color.a = 1.0
            markerArray.markers.append(marker)

            marker_text = Marker()
            marker_text.header.frame_id = "/pozyx"
            marker_text.header.stamp = self.node.get_clock().now().to_msg()
            marker_text.ns = "pozyx_anchors_id"
            marker_text.id = m_id
            m_id += 1

            marker_text.type = Marker.TEXT_VIEW_FACING
            marker_text.action = Marker.ADD
            marker_text.lifetime.sec = 100
            marker_text.scale.x = 0.3
            marker_text.scale.y = 0.3
            marker_text.scale.z = 0.3
            marker_text.pose.position.x = a.pos.x / 1000
            marker_text.pose.position.y = a.pos.y / 1000
            marker_text.pose.position.z = a.pos.z / 1000 + 1.0
            marker_text.pose.orientation.x = 0.0
            marker_text.pose.orientation.y = 0.0
            marker_text.pose.orientation.z = 0.0
            marker_text.pose.orientation.w = 1.0
            marker_text.color.r = 1.0
            marker_text.color.g = 1.0
            marker_text.color.b = 1.0
            marker_text.color.a = 1.0
            marker_text.text = "0x%0.4x" % a.network_id
            markerArray.markers.append(marker_text)

        markers_pub.publish(markerArray)

    def setAnchorsManual(self, save_to_flash=False):
        """Adds the manually measured anchors to the Pozyx's device list one for one."""
        for tag_id in self.tag_ids:
            status = self.pozyx.clearDevices(tag_id)
            for anchor in self.anchors:
                status &= self.pozyx.addDevice(anchor, tag_id)
            if len(self.anchors) > 4:
                status &= self.pozyx.setSelectionOfAnchors(PozyxConstants.ANCHOR_SELECT_MANUAL, len(self.anchors),
                                                           remote_id=tag_id)
            # enable these if you want to save the configuration to the devices.
            if save_to_flash:
                self.pozyx.saveAnchorIds(tag_id)
                self.pozyx.saveRegisters([PozyxRegisters.POSITIONING_NUMBER_OF_ANCHORS], tag_id)

            self.printPublishConfigurationResult(status, tag_id)

    def printPublishConfigurationResult(self, status, tag_id):
        """Prints the configuration explicit result, prints and publishes error if one occurs"""
        if tag_id is None:
            tag_id = 0
        if status == POZYX_SUCCESS:
            print("Configuration of tag %s: success" % tag_id)
        else:
            self.printPublishErrorCode("configuration", tag_id)


    def printPublishErrorCode(self, operation, network_id):
        """Prints the Pozyx's error and possibly sends it as a OSC packet"""
        error_code = SingleRegister()
        status = self.pozyx.getErrorCode(error_code, network_id)
        if network_id is None:
            network_id = 0
        if status == POZYX_SUCCESS:
            print("Error %s on ID %s, %s" %
                  (operation, "0x%0.4x" % network_id, self.pozyx.getErrorMessage(error_code)))
            if self.osc_udp_client is not None:
                self.osc_udp_client.send_message(
                    "/error_%s" % operation, [network_id, error_code[0]])
        else:
            # should only happen when not being able to communicate with a remote Pozyx.
            self.pozyx.getErrorCode(error_code)
            print("Error % s, local error code %s" % (operation, str(error_code)))
            if self.osc_udp_client is not None:
                self.osc_udp_client.send_message("/error_%s" % operation, [0, error_code[0]])

    def printPublishAnchorConfiguration(self):
        for anchor in self.anchors:
            print("ANCHOR,0x%0.4x,%s" % (anchor.network_id, str(anchor.pos)))
            if self.osc_udp_client is not None:
                self.osc_udp_client.send_message(
                    "/anchor", [anchor.network_id, anchor.pos.x, anchor.pos.y, anchor.pos.z])
                sleep(0.025)

def main(args=None):
    global pub, markers_pub, anchors
    rclpy.init(args=args)

    node = rclpy.create_node('qurianaPozyx')
    pub = node.create_publisher(Odometry, "odometry/pozyx", 1000)
    markers_pub = node.create_publisher(MarkerArray, "odometry/pozyx/markers", 10)

    # Check for the latest PyPozyx version. Skip if this takes too long or is not needed by setting to False.
    check_pypozyx_version = True
    if check_pypozyx_version:
        perform_latest_version_check()

    # shortcut to not have to find out the port yourself.
    serial_port = '/dev/ttyACM0' # get_first_pozyx_serial_port()
    if serial_port is None:
        print("No Pozyx connected. Check your USB cable or your driver!")
        quit()

    # enable to send position data through OSC
    use_processing = False

    # configure if you want to route OSC to outside your localhost. Networking knowledge is required.
    # ip = "127.0.0.1"
    # network_port = 8888


    # IDs of the tags to position, add None to position the local tag as well.
    tag_ids = [0x6e04] # [None, 0x6e04]

    # necessary data for calibration
    # anchors 9/30
    anchors = [
        ## DeviceCoordinates(0x6023, 1, Coordinates(-13563, -8838, 475)),  # ROOM
        # DeviceCoordinates(0x6e23, 1, Coordinates( -3327, -8849, 475)),  # ROOM
        # DeviceCoordinates(0x6e49, 1, Coordinates( -3077, -2959, 475)),  # ROOM
        # DeviceCoordinates(0x6e58, 1, Coordinates( -7238, -3510, 475)),  # ROOM
        # DeviceCoordinates(0x6050, 1, Coordinates( -9214, -9102, 475)),  # ROOM
        DeviceCoordinates(0x6037, 1, Coordinates( -2223, -7211, 475)),  # COR1
        DeviceCoordinates(0x6e08, 1, Coordinates(  -216, -7322, 475)),  # COR1
        # DeviceCoordinates(0x605b, 1, Coordinates( -1992,   243, 475)),  # COR1
        # DeviceCoordinates(0x6e30, 1, Coordinates(     0,     0, 475)),  # COR1 # COR2
        DeviceCoordinates(0x605b, 1, Coordinates( -2109,   -2683, 475)),  # COR1_CHANGE
        DeviceCoordinates(0x6e30, 1, Coordinates(  -112,   -2707, 475)),  # COR1_CHANGE        
        # DeviceCoordinates(0x6e7c, 1, Coordinates(   -59, -2126, 475)),  # COR2
        # DeviceCoordinates(0x6044, 1, Coordinates(-14124,   922, 475)),  # COR2 # COR3
        # DeviceCoordinates(0x6e22, 1, Coordinates(-14224,  -686, 475)),  # COR2 # COR3
        # DeviceCoordinates(0x6e39, 1, Coordinates(-20836,  -929, 475)),  # COR3
        # DeviceCoordinates(0x6e69, 1, Coordinates(-20902,   879, 475)),  # COR3
        # DeviceCoordinates(0x6e5c, 1, Coordinates(-18693,  1072, 475)),  # COR4
    ]

    """
    anchors = [
        DeviceCoordinates(0x6023, 1, Coordinates(12995, 17049, 475)),
        DeviceCoordinates(0x6037, 1, Coordinates(24229, 19294, 475)),
        DeviceCoordinates(0x6050, 1, Coordinates(17352, 17024, 475)),
        DeviceCoordinates(0x605b, 1, Coordinates(24052, 26749, 475)),
        DeviceCoordinates(0x6e08, 1, Coordinates(26239, 19293, 475)),
        DeviceCoordinates(0x6e23, 1, Coordinates(23195, 17997, 475)),
        DeviceCoordinates(0x6e30, 1, Coordinates(26055, 26616, 475)),
        DeviceCoordinates(0x6e49, 1, Coordinates(23144, 23493, 475)),
        DeviceCoordinates(0x6e58, 1, Coordinates(19019, 22715, 475)),
        DeviceCoordinates(0x6e7c, 1, Coordinates(26112, 24490, 475)),
    ]
    """

    """
    anchors = [
        DeviceCoordinates(0x6023, 1, Coordinates(-6109, 5305, 2000)),# Coordinates(12320, 23300, 2000)),  # 928 new
               DeviceCoordinates(0x6050, 1, Coordinates(-5239, -8444, 2000)),# Coordinates(14320, 9100, 2000)),  # 928 new 
               DeviceCoordinates(0x6e58, 1, Coordinates(-647, 5866, 2000)),# Coordinates(17861, 23738, 2000)),  # 928 new 
               DeviceCoordinates(0x6e49, 1, Coordinates(5735, 2233, 2000)),# Coordinates(24000, 20500, 2000)),  # 928 new
               DeviceCoordinates(0x6e23, 1, Coordinates(5410, -8742, 2000)),# Coordinates(24400, 9500, 2000)),  # 928 new 
        #        DeviceCoordinates(0x6e11, 1, Coordinates(2609, -4647, 2000)),# Coordinates(24400, 9500, 2000)),  # 928 new
            #    DeviceCoordinates(0x6023, 1, Coordinates(12320, 23300, 2000)),  # 928
            #    DeviceCoordinates(0x6050, 1, Coordinates(14320, 9100, 2000)),  # 928 
            #    DeviceCoordinates(0x6e58, 1, Coordinates(17861, 23738, 2000)),  # 928
            #    DeviceCoordinates(0x6e49, 1, Coordinates(24000, 20500, 2000)),  # 928
            #    DeviceCoordinates(0x6e23, 1, Coordinates(24400, 9500, 2000)),  # 928
            #    DeviceCoordinates(0x6e11, 1, Coordinates(24400, 9500, 2000)),  # 928 add
            #    DeviceCoordinates(0x6e08, 1, Coordinates(26200, 21400, 2000)),  # first area
            #    DeviceCoordinates(0x605b, 1, Coordinates(24200, 19300, 2000)),  # first area
            #    DeviceCoordinates(0x6037, 1, Coordinates(23900, 26600, 2000)),  # first-second area
            #    DeviceCoordinates(0x6e30, 1, Coordinates(23800, 24100, 2000)),  # first-second area
            #    DeviceCoordinates(0x6e7c, 1, Coordinates(26100, 26600, 2000)),  # first-second area
            #    DeviceCoordinates(0x6e39, 1, Coordinates(11800, 24300, 2000)),
            #    DeviceCoordinates(0x6044, 1, Coordinates(76400, 25500, 2000)),
            #    DeviceCoordinates(0x6e5c, 1, Coordinates(12300, 25900, 2000)),
            #    DeviceCoordinates(0x6e22, 1, Coordinates(7420, 25500, 2000)),
            #    DeviceCoordinates(0x6e73, 1, Coordinates(8700, 8840, 2000)),
            #    DeviceCoordinates(0x6e31, 1, Coordinates(6620, 8630, 2000)),
            #    DeviceCoordinates(0x6e69, 1, Coordinates(5300, 24200, 2000)),
               ]
    """

    # positioning algorithm to use, other is PozyxConstants.POSITIONING_ALGORITHM_TRACKING
    algorithm = PozyxConstants.POSITIONING_ALGORITHM_UWB_ONLY
    # algorithm = PozyxConstants.POSITIONING_ALGORITHM_TRACKING
    # positioning dimension. Others are PozyxConstants.DIMENSION_2D, PozyxConstants.DIMENSION_2_5D
    dimension = PozyxConstants.DIMENSION_2D # PozyxConstants.DIMENSION_2_5D
    # height of device, required in 2.5D positioning
    height = 475 # 500

    osc_udp_client = None
    if use_processing:
        osc_udp_client = SimpleUDPClient(ip, network_port)

    pozyx = PozyxSerial(serial_port)
    pozyx.clearDevices(0x6e04)
    if pozyx.doDiscovery(discovery_type=2, remote_id=0x6e04) == POZYX_SUCCESS:
        pozyx.printDeviceList(0x6e04)

    print(len(anchors))
    r = MultitagPositioning(pozyx, osc_udp_client, tag_ids, anchors,
                            algorithm, dimension, height, node)
    #try:
    r.setup()
    #except:
    #    print("eror")


    #pub = rospy.Publisher('chatter', String, queue_size=10)
    #r = rospy.Rate(100) # 20hz

    r.publishMarkerArray(anchors)
    while rclpy.ok():
        r.loop()
        # rclpy.spin_once(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':

    main()