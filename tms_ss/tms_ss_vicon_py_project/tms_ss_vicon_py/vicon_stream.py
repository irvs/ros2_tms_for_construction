# This package use Vicon """"UDP""""" stream

import json
import math
import socket
from datetime import datetime

import rclpy
from rclpy.node import Node, Clock

from std_msgs.msg import String
from tms_msg_db.msg import Tmsdb
from tms_msg_db.msg import TmsdbStamped
from tms_msg_db.srv import TmsdbGetData

import struct


def vicon_read(s):
    """Socket通信をする
    args:
        s :  通信を行うsocket.socket()のオブジェクト
    """

    data, addr = s.recvfrom(1024)

    # print(f"message: {data}\nfrom: {addr}")
    frame_number = int.from_bytes(data[0:4], 'little')
    
    items_in_block = int(data[4])

    item_id = int(data[5])
    item_data_size = int.from_bytes(data[6:8], 'little')
    item_name = data[8:32].decode('utf-8')
    # trans_x = int.from_bytes(data[32:40], 'little')
    print(items_in_block)
    print("data length : {}".format(len(data)))
    # trans_x = struct.unpack('d', data[32:40])
    # trans_y = int.from_bytes(data[40:48], 'big')
    # trans_z = int.from_bytes(data[48:56], 'big')
    # rot_x = int.from_bytes(data[56:64], 'big')
    # rot_y = int.from_bytes(data[64:72], 'big')
    # rot_z = int.from_bytes(data[72:80], 'big')
    trans_x = struct.unpack('<d', data[32:40])[0]/1000
    # print(type(trans_x[0]))
    trans_y = struct.unpack('<d', data[40:48])[0]/1000
    trans_z = struct.unpack('<d', data[48:56])[0]/1000
    rot_x = struct.unpack('<d', data[56:64])[0]
    rot_y = struct.unpack('<d', data[64:72])[0]
    rot_z = struct.unpack('<d', data[72:80])[0]

    # item_id_2 = int(data[80])
    # item_data_size_2 = int.from_bytes(data[81:83], 'little')
    # item_name_2 = data[83:107].decode('utf-8')
    # trans_x_2 = struct.unpack('<d', data[107:115])[0]
    # trans_y_2 = struct.unpack('<d', data[115:123])[0]
    # trans_z_2 = struct.unpack('<d', data[123:131])[0]
    # rot_x_2 = struct.unpack('<d', data[131:139])[0]
    # rot_y_2 = struct.unpack('<d', data[139:147])[0]
    # rot_z_2 = struct.unpack('<d', data[147:155])[0]
    
    
    print(item_name, " ", trans_x, " ", trans_y, " ", trans_z,)

    # print(item_name_2, " ", trans_x_2, " ", trans_y_2, " ", trans_z_2,)


def main(args=None):
    global node, roll, pitch, temp, rate, wave
    rclpy.init(args=args)


    node = rclpy.create_node('vicon_stream')

    publisher = node.create_publisher(TmsdbStamped, 'tms_db_data', 1000)
    i = 0
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:  # UDP
        s.bind(('', 51001))  # IPv4アドレス, PORT番号
        print("vicon_stream ready  ... ")
        while rclpy.ok():
            vicon_read(s)  # socketから各種センサ値を取得

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()