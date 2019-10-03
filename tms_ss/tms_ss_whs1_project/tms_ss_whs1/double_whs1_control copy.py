# doubleをwhs1(心拍センサ)でコントロールする

from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
import rclpy
from tms_msg_db.msg import Tmsdb
from tms_msg_db.srv import TmsdbGetData
from geometry_msgs.msg import PoseStamped

import pymongo
import json

TMSDB_ID = 3021  # database request "id" for whs1
TMSDB_SENSOR = 3021  # database request "sensor" for whs1

class DoubleWhs1Control(Node):

    def __init__(self):
        super().__init__('double_whs1_control')
        self.publish_flag = False

        self.publisher = self.create_publisher(PoseStamped, "/move_base_simple/goal", 10)

        self.cli_dbreader = self.create_client(TmsdbGetData, 'tms_db_reader')
        while not self.cli_dbreader.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service "tms_db_reader" not available, waiting again...')

        self.cb_group = ReentrantCallbackGroup()
        timer_period = 1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback, callback_group=self.cb_group)
        self.get_logger().info('double_whs1_control ready...')

    async def timer_callback(self):
        rate = await self.get_whs1_heartrate()
        print(f"rate: {rate}")
        
        if rate > 140 and not self.publish_flag:
            self.publisher.publish(self.setGoalPoseStamped())
            self.get_logger().info('[HeartRate Warning] Robot moves to you !')
            self.publish_flag = True

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

    def setGoalPoseStamped(self):
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = "map"
        pose_stamped.pose.position.x = 4.0516
        pose_stamped.pose.position.y = -1.5177
        pose_stamped.pose.position.z = -0.0053
        pose_stamped.pose.orientation.x = 0.0
        pose_stamped.pose.orientation.y = 0.0
        pose_stamped.pose.orientation.z = -0.38268343
        pose_stamped.pose.orientation.w = -0.92387953

        return pose_stamped

def main(args=None):
    rclpy.init(args=args)


    double_whs1_control = DoubleWhs1Control()

    rclpy.spin(double_whs1_control)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    double_whs1_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()