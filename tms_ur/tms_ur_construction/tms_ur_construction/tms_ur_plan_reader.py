# Copyright 2023, IRVS Laboratory, Kyushu University, Japan.
# 
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
# 
#     http://www.apache.org/licenses/LICENSE-2.0
# 
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from functools import partial
from time import sleep
import rclpy
from rclpy.node import Node

from trajectory_msgs.msg import JointTrajectory
from nav_msgs.msg import Path
from diagnostic_msgs.msg import KeyValue
from tms_msg_db.srv import TmsdbGetData

import tms_db_manager.tms_db_util as db_util


NODE_NAME = "tms_ur_plan_reader"
DATA_ID = 3012#2012
DATA_TYPE = "parameter"


class TmsUrPlanReader(Node):
    """Get construction vehicle's Odometry data from tms_db_reader."""

    def __init__(self):
        super().__init__(NODE_NAME)

        # Declare parameters
        self.declare_parameter("latest", False)
        #self.declare_parameter("machine_name", "machine_name")

        # Get parameters
        self.latest: bool = (
            self.get_parameter("latest").get_parameter_value().bool_value
        )

        self.subscription = self.create_subscription(
            KeyValue,
            'planwritten',
            self.writer_callback,
            10)
        self.subscription

        self.pathpublisher_ = self.create_publisher(Path, "~/output/plan", 10)

        self.jointpublisher_ = self.create_publisher(JointTrajectory, "~/output/joint_plan", 10)

        self.cli = self.create_client(TmsdbGetData, "tms_db_reader")
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("service not available, waiting again...")

    
    def writer_callback(self, msg):
        self.machine_name = msg.key
        self.record_name = msg.value#path_plan or joint_plan
        self.send_request()

    def send_request(self):
        """
        Send request to tms_db_reader to get Odometry data.
        """
        self.req = TmsdbGetData.Request()
        self.req.type = DATA_TYPE
        self.req.id = DATA_ID
        self.req.latest_only = self.latest
        #self.req.param_type = "plan"
        self.req.param_type = self.record_name
        self.req.name = self.machine_name
        self.req.recordnames = [self.record_name]

        future = self.cli.call_async(self.req)
        future.add_done_callback(partial(self.callback_response))
        self.get_logger().info("send request to read plan")

    def callback_response(self, future):
        """
        Set response from tms_db_reader.
        """
        try:
            self.res = future.result()
            self.tmsdbs = self.res.tmsdbs
            self.publish_plan()
        except:
            return
        
    def publish_plan(self) -> None:
        if(self.tmsdbs[0].pathplan.poses != []):
            msg: Path = self.tmsdbs[0].pathplan
            self.pathpublisher_.publish(msg)
            self.get_logger().info("published path plan")
        elif(self.tmsdbs[0].jointplan != ""):
            msg: JointTrajectory = self.tmsdbs[0].jointplan
            self.jointpublisher_.publish(msg)
            self.get_logger().info("published joint trajectory plan")




def main(args=None):
    rclpy.init(args=args)

    tms_ur_plan_reader = TmsUrPlanReader()
    rclpy.spin(tms_ur_plan_reader)

    tms_ur_plan_reader.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
