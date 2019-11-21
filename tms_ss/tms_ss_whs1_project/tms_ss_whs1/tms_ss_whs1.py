# This is ROS2 version of ros_tms/tms_ss/tms_ss_whs1/src/main.cpp
# whs1_client(windows)を起動しておく。
# https://github.com/irvs/whs1_client/Debug/whs1_client.exe
# whs1_clientからデータを受取り、DBに格納。

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

PEAK = 700
MIN_INTERVAL = 300

temp = 0.0  # 温度
rate = 0  # 心拍数
p_rate = 0  # 1frame前の心拍数
msec = 0  # 周期
p_msec = 0  # 1frame前の周期
wave = [0] * 100  # 波形（要素数100)
roll = 0.0
pitch = 0.0
last_peak_time = -1

count = 0
def whs1_read(s):
    """Socket通信をする
    args:
        s :  通信を行うsocket.socket()のオブジェクト
    """
    global temp, rate, p_rate, msec, p_msec, hakei, roll, pitch, count
    global last_peak_time

    data, addr = s.recvfrom(1024)

    wave[count%100] = int.from_bytes(data[0:2], 'little')
    msec = int.from_bytes(data[2:4], "little")
    temp = int.from_bytes(data[6:8], "little") * 0.01

    # 各方向加速度からroll, pitchを計算
    acc_x = int.from_bytes(data[4:6], "little") * 0.01
    acc_y = int.from_bytes(data[10:12], "little") * 0.01
    acc_z = int.from_bytes(data[8:10], "little") * 0.01
    g = math.sqrt(acc_x*acc_x + acc_y*acc_y + acc_z*acc_z)
    if acc_y != 0:
        roll = math.asin(-acc_x / g)
        pitch = math.atan(acc_x / acc_y)
    
    ## 周期を計算
    interval = msec - last_peak_time
    while(interval < 0):
        interval += 60000

    if wave[count%100] > PEAK and interval > MIN_INTERVAL:
        if last_peak_time == -1:
            last_peak_time = msec
        else:
            p_rate = rate
            rate = 1000.0 / interval * 60.0
            # print(rate)
            if rate < 30:
                rate = 0
            elif rate > 200:
                rate = p_rate
            last_peak_time = msec
        
    count += 1
        
def db_write(pub, state):
    """tms_db_writerにトピックを送る（結果、DBにデータを書き込む)
    args:
        pub: node.publisher型。tms_db_writerへのトピック送信用
        state: int型。1なら測定中、0なら測定停止中
    """
    global node, roll, pitch, temp, rate, wave
    now_time = datetime.now().isoformat()

    send_data = {"temp": temp, "rate" : rate, "wave" : wave}

    db_msg = TmsdbStamped()
    db_msg.header.frame_id = "/world"
    # TODO: db_msgのヘッダーのstampがわからなかったので書いていない
    # clock = Clock()
    # db_msg.header.stamp = clock.now()
    
    tmp_data = Tmsdb()
    tmp_data.time = now_time
    tmp_data.name = "whs1_mybeat"
    tmp_data.id = 3021
    tmp_data.place = 5001
    tmp_data.sensor = 3021
    tmp_data.state = state
    tmp_data.rr = roll
    tmp_data.rp = pitch
    tmp_data.ry = 0.0

    tmp_data.note = json.dumps(send_data)
    db_msg.tmsdb = [tmp_data]

    pub.publish(db_msg)
    


def main(args=None):
    global node, roll, pitch, temp, rate, wave
    rclpy.init(args=args)


    node = rclpy.create_node('tms_ss_whs1')

    publisher = node.create_publisher(TmsdbStamped, 'tms_db_data', 1000)
    i = 0
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:  # UDP
        s.bind(('192.168.4.119', 65001))  # IPv4アドレス, PORT番号
        print("tms_ss_whs1 ready  ... ")
        while rclpy.ok():
            whs1_read(s)  # socketから各種センサ値を取得
            print(f"temp: {temp}, rate: {rate}")

            if i % 10 == 0:
                db_write(publisher, 1)  # 10回に一回、DBに格納
            i += 1
        db_write(publisher, 0)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()