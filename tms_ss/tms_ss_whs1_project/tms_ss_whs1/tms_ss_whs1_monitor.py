# tms_ss_whs1の出力をモニタリングする
# tms_db_readerを同時に起動しておく

from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
import rclpy
from tms_msg_db.msg import Tmsdb
from tms_msg_db.srv import TmsdbGetData

import numpy as np
import  matplotlib.pyplot as plt
import pymongo
import json

TMSDB_ID = 3021  # database request "id" for whs1
TMSDB_SENSOR = 3021  # database request "sensor" for whs1

class TmsSsWhs1Monitor(Node):

    def __init__(self):
        super().__init__('tms_ss_whs1_monitor')

        self.cli_dbreader = self.create_client(TmsdbGetData, 'tms_db_reader')
        while not self.cli_dbreader.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service "tms_db_reader" not available, waiting again...')

        self.cb_group = ReentrantCallbackGroup()
        timer_period = 1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback, callback_group=self.cb_group)
        self.get_logger().info('tms_ss_whs1_monitor ready...')

    async def timer_callback(self):
        rate = await self.get_whs1_heartrate()
        print(f"rate: {rate}")

    async def get_whs1_heartrate(self):
        """Whs1の心拍数を取得する

        tms_ss_whs1/src/main.cppは、tmsdbのnoteにjsonとして各種データを保存しているので、
        noteをjson.loadsで辞書型に変更して読みだす。
        """
        rate = -1  # Error Number
        data_tmsdb = Tmsdb()
        data_tmsdb.id = TMSDB_ID
        data_tmsdb.sensor = TMSDB_SENSOR

        res = await self.call_dbreader(data_tmsdb)  
        if len(res) == 0:
            return rate
        note = res[0].note

        whs1_params = json.loads(note)
        rate = whs1_params["rate"]
        wave = whs1_params["wave"]

        # draw graph
        plt.cla()
        plt.title("tms_ss_whs1 Monitor")
        plt.ylim(0,1023)
        fig = plt.plot(wave)
        plt.text(0, 850, "{0:.2f} bpm".format(rate), fontsize=32, color="green")
        plt.pause(0.01)

        return rate


    async def call_dbreader(self,data):
        """[tms_db_reader] DBからデータを読み取る
        """
        req = TmsdbGetData.Request()
        req.tmsdb = data
        self.future_dbreader  = self.cli_dbreader.call_async(req)

        await self.future_dbreader

        if self.future_dbreader.result() is not None:
            res = self.future_dbreader.result().tmsdb
            return res
        else:
            self.get_logger().info('Service "tms_db_reader" call failed %r' % (self.future_dbreader.exception(),))


def main(args=None):
    rclpy.init(args=args)

    plt.ion()

    monitor = TmsSsWhs1Monitor()

    rclpy.spin(monitor)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    monitor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()