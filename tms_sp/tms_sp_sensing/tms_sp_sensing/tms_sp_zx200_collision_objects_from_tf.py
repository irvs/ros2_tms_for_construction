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

from datetime import datetime
import json
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Pose
from std_msgs.msg import String
from pymongo import MongoClient
from sensing_msgs.msg import Zx200CollisionObjectsIc120

import tms_db_manager.tms_db_util as db_util

import tf2_ros
from tf2_geometry_msgs import do_transform_pose
from geometry_msgs.msg import TransformStamped
import tf2_py as tf2

MONGODB_IPADDRESS = '127.0.0.1'
MONGODB_PORTNUMBER = 27017

DATA_ID = 10000
DATA_TYPE = 'parameter' 
DATA_NAME = 'data_name'

class UpdateDB_Parameter(Node):
    def __init__(self):
        super().__init__("tms_sp_zx200_collison_objects_from_tf")
        
        # パラメータの宣言と取得
        self.declare_parameter('robot_name', 'zx200')
        self.robot_name = self.get_parameter('robot_name').get_parameter_value().string_value
        
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # 定期的にtfを取得してDBを更新するタイマー
        self.timer = self.create_timer(1.0, self.update_db_from_tf)
        
        self.get_logger().info(f"Tracking robot: {self.robot_name}")
    
    def update_db_from_tf(self) -> None:
        try:
            # ロボットのtfを取得 (robot名_tf/base_link)
            transform = self.tf_buffer.lookup_transform(
                'map',  # 基準フレーム
                f'{self.robot_name}_tf/base_link',  # ターゲットフレーム
                rclpy.time.Time()
            )
            
            # tfから位置・姿勢情報を抽出
            x = transform.transform.translation.x
            y = transform.transform.translation.y
            z = transform.transform.translation.z
            qx = transform.transform.rotation.x
            qy = transform.transform.rotation.y
            qz = transform.transform.rotation.z
            qw = transform.transform.rotation.w
            
            # データベースを更新
            self.update_database(x, y, z, qx, qy, qz, qw)
            
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            self.get_logger().warn(f"Failed to get transform for {self.robot_name}: {e}")
    
    def update_database(self, x: float, y: float, z: float, qx: float, qy: float, qz: float, qw: float) -> None:
        client = MongoClient(MONGODB_IPADDRESS, MONGODB_PORTNUMBER)
        db = client['rostmsdb']
        collection = db['parameter']
        
        # record_nameで既存のレコードを検索
        record_name = f"collision_object_{self.robot_name}"
        query = {"record_name": record_name}
        
        # 位置情報のみを更新
        update_parameter_info = {"x": x, "y": y, "z": z, "qx": qx, "qy": qy, "qz": qz, "qw": qw}
        update_query = {"$set": update_parameter_info}
        
        # レコードが存在しない場合は新規作成、存在する場合は位置のみ更新
        result = collection.update_one(query, update_query, upsert=True)
        
        if result.upserted_id:
            # 新規作成時は基本情報も設定
            collection.update_one(
                {"_id": result.upserted_id},
                {"$set": {
                    "model_name": "zx200", 
                    "type": "dynamic"
                }}
            )
            self.get_logger().info(f"Created new record for {self.robot_name}")
        else:
            self.get_logger().debug(f"Updated position for {self.robot_name}")

def main(args=None):
    rclpy.init(args=args)
    update_db_parameter = UpdateDB_Parameter()
    rclpy.spin(update_db_parameter)
    update_db_parameter.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()