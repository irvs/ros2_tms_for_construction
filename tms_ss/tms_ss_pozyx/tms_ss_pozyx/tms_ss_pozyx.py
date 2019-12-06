from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from nav_msgs.msg import Odometry
import rclpy

import paho.mqtt as mqtt
import json
from rx.subject import Subject


HOST_IPADDRESS = "192.168.4.68"
PORT = 1883


class TmsSsPozyx(Node):
    def __init__(self):
        super().__init__('tms_ss_pozyx')
        self.pozyx_pos_publisher_ = self.create_publisher(Odometry, 'odometry/pozyx', 10)
        self.pozyx_client = PozyxClient()
        self.pozyx_client.stream.subscribe(self.pozyx_position_publish)
        #self.timer = self.create_timer(timer_period, self.publish_pozyx_position_callback)
    
    def pozyx_position_publish(self, stream):
        pozyx_position_msg = Odometry()
         stream_dict = json.loads(stream)[0]
        if stream_dict["tagId"] == str(int(self.tagId, 16)):
            print(stream_dict)
            try:
                position = stream_dict["data"]["coordinates"]
                q = stream_dict["data"]["tagData"]["quaternion"]
            except:
                pass
            else:
                pozyx_position_msg.header.frame_id = "map"
                pozyx_position_msg.child_frame_id  = "odom"
                pozyx_position_msg.header.stamp = rclpy.time.now()

                pozyx_position_msg.pose.pose.position.x = position["x"]
                pozyx_position_msg.pose.pose.position.y = position["y"]
                pozyx_position_msg.pose.pose.position.z = position["z"]

                pozyx_position_msg.pose.pose.orientation.x = q["x"]
                pozyx_position_msg.pose.pose.orientation.y = q["y"]
                pozyx_position_msg.pose.pose.orientation.z = q["z"]
                pozyx_position_msg.pose.pose.orientation.w = q["w"]

                 self.pozyx_pos_publisher_.publish(pozyx_position_msg)



class PozyxClient():

    def __init__(self):
        host = HOST_IPADDRESS
        port = PORT
        topic = "tags"

        self.stream =  Subject()
        self.client = mqtt.Client()
        # set callbacks
        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message
        self.client.on_subscribe = self.on_subscribe
        self.client.connect(host, port=port)
        self.client.subscribe(topic)

        # Loop for initialization
        self.client.loop_forever()
        # self.client.loop()
    
    def on_connect(self, client, userdata, flags, rc):
        print(mqtt.connack_string(rc))

    def on_message(self, client, userdata, msg):
        print("Positioning update:", msg.payload.decode())
        stream.on_next(msg.payload.decode())
    
    def on_subscribe(self, client, userdata, mid, granted_qos):
        print("subscribed to topic!")


def main(args=None):
    rclpy.init(args=args)
    
    tms_ss_pozyx = TmsSsPozyx()
    rclpy.spin(tms_ss_pozyx)

    tms_ss_pozyx.destroy_node()
    rclpy.shutdown()


if __name__=='__main__':
    main()