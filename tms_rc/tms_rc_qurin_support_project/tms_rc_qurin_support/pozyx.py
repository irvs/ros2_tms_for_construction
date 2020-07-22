## pozyx.py
# pozyx creatorからデータを取得，
# EKFノードにOdometryトピックを発行．

"""pozyxのデータをローカルMQTTで受け取り
ROS Topicとして再度発行する
"""
import rclpy
from std_msgs.msg import String
from nav_msgs.msg import Odometry
import paho.mqtt.client as mqtt
import ssl
import json
import pprint

# Pozyx Mqtt client settings
host = "localhost"
port = 1883
topic = "tags"
tagId = u"28164"  # 28164 = 0x6e04

def on_connect(client, userdata, flags, rc):
    print(mqtt.connack_string(rc))

# callback triggered by a new Pozyx data packet
def on_message(client, userdata, msg):
    global pub
    data_str = msg.payload.decode()
    datas = json.loads(data_str)
    # pprint.pprint(datas)
    for d in datas:
        #if d[u"success"] and d[u"tagId"] == tagId:
        pos = d[u"data"][u"coordinates"]
        odom = Odometry()
        # odom.header.stamp = rospy.Time.now() 
        
        odom.header.frame_id = "map"
        odom.child_frame_id = "base_footprint"
        odom.pose.pose.position.x = pos[u"x"] * 0.001
        odom.pose.pose.position.y = pos[u"y"] * 0.001
        odom.pose.pose.position.z = pos[u"z"] * 0.001
        
        ori = d[u"data"][u"tagData"][u"quaternion"]
        odom.pose.pose.orientation.x = ori[u"x"]
        odom.pose.pose.orientation.y = ori[u"y"]
        odom.pose.pose.orientation.z = ori[u"z"]
        odom.pose.pose.orientation.w = ori[u"w"]
        pub.publish(odom)



def on_subscribe(client, userdata, mid, granted_qos):
    print("Subscribed to topic!")


def main(args=None):
    global pub
    rclpy.init(args=args)

    node = rclpy.create_node('qurianaPozyx')
    pub = node.create_publisher(Odometry, "odometry/pozyx", 1000)

    client = mqtt.Client()

    # set callbacks
    client.on_connect = on_connect
    client.on_message = on_message
    client.on_subscribe = on_subscribe
    client.connect(host, port=port)
    client.subscribe(topic)

    #pub = rospy.Publisher('chatter', String, queue_size=10)
    #r = rospy.Rate(100) # 20hz
    while rclpy.ok():
        client.loop(timeout=0.5)
        # rclpy.spin_once(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()