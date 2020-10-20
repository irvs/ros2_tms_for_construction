import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray

from pypozyx import (PozyxSerial, PozyxConstants, Coordinates, PozyxRegisters, version, DeviceCoordinates,
                     SingleRegister, DeviceRange, POZYX_SUCCESS, get_first_pozyx_serial_port, Quaternion)

from pypozyx.tools.version_check import perform_latest_version_check
import time


class RangeDebugger(Node):

    def __init__(self):
        super().__init__("range_debugger")
        self.range_pub = self.create_publisher(String, "range", 10)
        self.position_pub = self.create_publisher(Odometry, "odometry/pozyx", 10)
        self.markers_pub = self.create_publisher(MarkerArray, "odometry/pozyx/markers", 10)
        # serial port setting
        serial_port = "/dev/ttyACM0"
        seiral_port = get_first_pozyx_serial_port()
        if serial_port is None:
            print("No Pozyx connected. CHeck your USB cable or your driver!")
            quit()

        self.pozyx = PozyxSerial(serial_port)

        # remote and destination
        # But sorry, just 1 tag is useable.
        # "None" is setting for use USB-connected tag, "0xXX"(tag id) is to use remote tag. 
        self.tag_ids = [None] # 0x6e04]  # TODO: To use multiple tags

        self.ranging_protocol = PozyxConstants.RANGE_PROTOCOL_PRECISION
        self.range_timer_ = self.create_timer(
            0.1, self.range_callback
        )

        self.anchors = [
            # DeviceCoordinates(0x605b, 1, Coordinates(   0, 0, 0)),  # test
            # DeviceCoordinates(0x603b, 1, Coordinates( 800, 0, 0)),  # test
            # DeviceCoordinates(0x6023, 1, Coordinates(-13563, -8838, 475)),  # ROOM
            # DeviceCoordinates(0x6e23, 1, Coordinates( -3327, -8849, 475)),  # ROOM
            # DeviceCoordinates(0x6e49, 1, Coordinates( -3077, -2959, 475)),  # ROOM
            ## DeviceCoordinates(0x6e58, 1, Coordinates( -7238, -3510, 475)),  # ROOM
            # DeviceCoordinates(0x6050, 1, Coordinates( -9214, -9102, 475)),  # ROOM
            DeviceCoordinates(0x6037, 1, Coordinates( -2199, -5889, 475)),  # COR1_C2
            DeviceCoordinates(0x6e08, 1, Coordinates(  -239, -7321, 475)),  # COR1_C2
            DeviceCoordinates(0x605b, 1, Coordinates( -2130,   -2630, 475)),  # COR1_C2
            DeviceCoordinates(0x6e30, 1, Coordinates(  -178,   -4168, 475)),  # COR1_C2
            DeviceCoordinates(0x6e7c, 1, Coordinates(   -59, -2126, 475)),  # COR2
            DeviceCoordinates(0x6044, 1, Coordinates(-14124,   922, 475)),  # COR2 # COR3
            DeviceCoordinates(0x6e22, 1, Coordinates(-14224,  -686, 475)),  # COR2 # COR3
        ]

        self.algorithm = PozyxConstants.POSITIONING_ALGORITHM_UWB_ONLY
        self.dimension = PozyxConstants.DIMENSION_2D
        self.height = 475

        self.periodically_i = 0
        self.setup()


    def setup(self):
        self.setAnchorsManual()
        for anchor in self.anchors:
            self.get_logger().info("ANCHOR,0x%0.4x,%s" % (anchor.network_id, str(anchor.pos)))
    
    def setAnchorsManual(self):
        """Adds the manually measured anchors to the Pozyx's device list one for one."""
        for tag_id in self.tag_ids:
            status = self.pozyx.clearDevices(tag_id)
            for anchor in self.anchors:
                status &= self.pozyx.addDevice(anchor, tag_id)
            if len(self.anchors) > 4:
                # !!! AUTO Test from 10/19
                status &= self.pozyx.setSelectionOfAnchors(PozyxConstants.ANCHOR_SELECT_AUTO, len(self.anchors),
                                                           remote_id=tag_id)
            self.printPublishConfigurationResult(status, tag_id)

    def printPublishConfigurationResult(self, status, tag_id):
        """Prints the configuration explicit result, prints and publishes error if one occurs"""
        if tag_id is None:
            tag_id = 0
        if status == POZYX_SUCCESS:
            self.get_logger().info("Configuration of tag %s: success" % tag_id)
        else:
            self.printPublishErrorCode("configuration", tag_id)


    def printPublishErrorCode(self, operation, network_id):
        """Prints the Pozyx's error and possibly sends it as a OSC packet"""
        error_code = SingleRegister()
        status = self.pozyx.getErrorCode(error_code, network_id)
        if network_id is None:
            network_id = 0
        if status == POZYX_SUCCESS:
            self.get_logger().error("Error %s on ID %s, %s" %
                  (operation, "0x%0.4x" % network_id, self.pozyx.getErrorMessage(error_code)))
        else:
            # should only happen when not being able to communicate with a remote Pozyx.
            self.pozyx.getErrorCode(error_code)
            self.get_logger().error("Error % s, local error code %s" % (operation, str(error_code)))

    def range_callback(self):
        """Do ranging periodically, and publish visualizasion_msg MarkerArray"""
        s = time.time()
        self.periodically_i += 1
        if self.periodically_i % 10 == 0:
            self.doRanging()
        m = time.time()
        self.doPositioning()
        e = time.time()
        # sr = SingleRegister(0, 1)
        for tag_id in self.tag_ids:
            # self.pozyx.getNumberOfAnchors(sr, remote_id=tag_id)
            self.get_logger().info(f"doRanging: {m-s}, doPositioning: {e-m}")

    def doRanging(self):
        for tag_id in self.tag_ids:
            for anchor in self.anchors:
                device_range = DeviceRange()
                status = self.pozyx.doRanging(
                    anchor.network_id, device_range, tag_id
                )
                if status == POZYX_SUCCESS:
                    self.publishMarkerArray(device_range.distance, anchor)
                    # self.get_logger().info(f"{device_range.distance}")
                else:
                    error_code = SingleRegister()
                    status = self.pozyx.getErrorCode(error_code)
                    if status == POZYX_SUCCESS:
                        self.get_logger().error("ERROR Ranging, local %s" %
                            self.pozyx.getErrorMessage(error_code))
                    else:
                        self.get_logger().error("ERROR Ranging, couldn't retrieve local error")
       

    def doPositioning(self):
        for tag_id in self.tag_ids:
            position = Coordinates()
            status = self.pozyx.doPositioning(position, self.dimension, self.height, self.algorithm, remote_id=tag_id)
            quat = Quaternion()
            # status &= self.pozyx.getNormalizedQuaternion(quat, tag_id)
            quat.w = 1.0
            if status == POZYX_SUCCESS:
                self.printPublishPosition(position, tag_id, quat)
            else:
                self.printPublishErrorCode("positioning", tag_id)
    
    def printPublishPosition(self, position, network_id, quat):
        if network_id is None:
            network_id = 0
        
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = "pozyx"
        odom.pose.pose.position.x = position.x * 0.001
        odom.pose.pose.position.y = position.y * 0.001
        if self.dimension == PozyxConstants.DIMENSION_3D:
            odom.pose.pose.position.z = position.z * 0.001
        else:
            odom.pose.pose.position.z = float(self.height) / 1000
        odom.pose.pose.orientation.x = quat.x
        odom.pose.pose.orientation.y = quat.y
        odom.pose.pose.orientation.z = quat.z
        odom.pose.pose.orientation.w = quat.w
        odom.pose.covariance = [
            1.0, 0.0, 0.0, 0.0, 0.0, 0.0,\
            0.0, 1.0, 0.0, 0.0, 0.0, 0.0,\
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,\
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,\
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,\
            0.0, 0.0, 0.0, 0.0, 0.0, 0.03,
            ]
        self.position_pub.publish(odom)
    

    def publishMarkerArray(self, distance, anchor):
        """
        Visualization pozyx anchors for Rviz2

        Parameters
        ----------
        distance : float
        """

        is_3D_estimation = (self.dimension == PozyxConstants.DIMENSION_3D)

        pos_x = anchor.pos.x
        pos_y = anchor.pos.y
        pos_z = anchor.pos.z

        markerArray = MarkerArray()
        m_id = anchor.network_id
        
        # marker of pozyx pos
        marker_pos = Marker()
        marker_pos.header.frame_id = "/pozyx"
        marker_pos.header.stamp = self.get_clock().now().to_msg()
        marker_pos.ns = "pozyx_pos"  # namespace
        marker_pos.id = m_id
        m_id += 1

        marker_pos.type = Marker.CUBE
        marker_pos.action = Marker.ADD
        marker_pos.lifetime.sec = 1
        marker_pos.scale.x = 0.07
        marker_pos.scale.y = 0.07
        marker_pos.scale.z = 0.02
        marker_pos.pose.position.x = pos_x / 1000
        marker_pos.pose.position.y = pos_y / 1000
        marker_pos.pose.position.z = pos_z / 1000
        marker_pos.pose.orientation.x = 0.0
        marker_pos.pose.orientation.y = 0.0
        marker_pos.pose.orientation.z = 0.0
        marker_pos.pose.orientation.w = 1.0
        marker_pos.color.r = 0.0
        marker_pos.color.g = 1.0
        marker_pos.color.b = 0.0
        marker_pos.color.a = 1.0
        markerArray.markers.append(marker_pos)

        # marker of pozyx distance
        marker_pos = Marker()
        marker_pos.header.frame_id = "/pozyx"
        marker_pos.header.stamp = self.get_clock().now().to_msg()
        marker_pos.ns = "pozyx_distance"  # namespace
        marker_pos.id = m_id
        m_id += 1
        
        if is_3D_estimation:
            marker_pos.type = Marker.SPHERE
            marker_pos.scale.z = float(distance) * 2 / 1000
        else:
            marker_pos.type =Marker.CYLINDER
            marker_pos.scale.z = 0.001
        
        marker_pos.action = Marker.ADD
        marker_pos.lifetime.sec = 1
        marker_pos.scale.x = float(distance) * 2 / 1000
        marker_pos.scale.y = float(distance) * 2 / 1000
        marker_pos.pose.position.x = pos_x / 1000
        marker_pos.pose.position.y = pos_y / 1000
        marker_pos.pose.position.z = pos_z / 1000
        marker_pos.pose.orientation.x = 0.0
        marker_pos.pose.orientation.y = 0.0
        marker_pos.pose.orientation.z = 0.0
        marker_pos.pose.orientation.w = 1.0
        marker_pos.color.r = 0.0
        marker_pos.color.g = 0.5
        marker_pos.color.b = 0.5
        marker_pos.color.a = 0.1
        markerArray.markers.append(marker_pos)

        # marker of pozyx label
        marker_pos = Marker()
        marker_pos.header.frame_id = "/pozyx"
        marker_pos.header.stamp = self.get_clock().now().to_msg()
        marker_pos.ns = "pozyx_distance_label"  # namespace
        marker_pos.id = m_id
        m_id += 1
        
        marker_pos.type = Marker.TEXT_VIEW_FACING
        marker_pos.action = Marker.ADD
        marker_pos.lifetime.sec = 1
        marker_pos.scale.x = 1.0
        marker_pos.scale.y = 1.0
        marker_pos.scale.z = 0.2
        marker_pos.pose.position.x = pos_x / 1000
        marker_pos.pose.position.y = pos_y / 1000
        marker_pos.pose.position.z = pos_z / 1000 + 0.5
        marker_pos.pose.orientation.x = 0.0
        marker_pos.pose.orientation.y = 0.0
        marker_pos.pose.orientation.z = 0.0
        marker_pos.pose.orientation.w = 1.0
        marker_pos.color.r = 1.0
        marker_pos.color.g = 1.0
        marker_pos.color.b = 1.0
        marker_pos.color.a = 1.0
        marker_pos.text = f"{float(distance / 1000):.2f}\n{hex(anchor.network_id)}"

        markerArray.markers.append(marker_pos)

        # Publish markers!
        self.markers_pub.publish(markerArray)




def main(args=None):
    rclpy.init(args=args)

    range_debugger = RangeDebugger()
    rclpy.spin(range_debugger)
    # if rclpy.ok():
    #     range_debugger.range_callback()

    range_debugger.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()