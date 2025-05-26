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

from pymongo import MongoClient
import json
import pymongo

import rclpy
from rclpy.node import Node

import tms_db_manager.tms_db_util as db_util
from tms_msg_db.msg import TmsdbQuery

# MongoDBに接続（ローカルホストのデフォルトのポートに接続する例）
client = MongoClient('mongodb://localhost:27017/')
# 使用するデータベースとコレクションの指定
db = client['rostmsdb']  # データベース名
collection = db['parameters']  # コレクション名


class TmsDbWriterQuery(Node):
    """Write data to MongoDB."""
    

    def __init__(self):
        super().__init__('tms_db_writer_query')
        

        # サブスクリプション作成
        self.subscription = self.create_subscription(
            TmsdbQuery,
            "tms_db_param_data",
            self.db_write_callback,
            10
        )
    



    def db_write_callback(self, msg: TmsdbQuery) -> None:
        """
        Store data.

        Parameters
        ----------
        msg : Tmsdb
            An instance of a ROS2 custom message to store data.
        """
        if (msg.type=="pose"):
            document = {
            'model_name': msg.vehicle_name,
            'x': msg.add_path.pose.position.x,
            'y': msg.add_path.pose.position.y,
            'qw': msg.add_path.pose.orientation.w,
            'qx': msg.add_path.pose.orientation.x,
            'qy': msg.add_path.pose.orientation.y,
            'qz': msg.add_path.pose.orientation.z,
            'record_name': msg.record_name#,
            }

            try:
                result = collection.insert_one(document)
                print('Inserted document ID:', result.inserted_id)
            except Exception as e:
                print('Error inserting document:', e)

        if(msg.type=="joint"):
            if len(msg.add_joints.position) > 0:
                document = {
                'model_name': msg.vehicle_name,
                'record_name': msg.record_name,
                'type': 'static',
                'trajectory' : 1,
                'swing_joint': msg.add_joints.position[0],
                'boom_joint': msg.add_joints.position[1],
                'arm_joint': msg.add_joints.position[2],
                'bucket_joint': msg.add_joints.position[3],
                'bucket_end_joint': msg.add_joints.position[4]
                }
            else:
                self.get_logger().warn("Received JointState message with empty 'position' array.")

            try:
                result = collection.insert_one(document)
                print('Inserted document ID:', result.inserted_id)
            except Exception as e:
                print('Error inserting document:', e)


        if (msg.type=="pose_query"):
            ##################
            # 既存のドキュメントを指定するための条件
          #  query = {"model_name": "ic120", "record_name": "test_PATH"}
            # 更新するためのクエリ条件（車両名と記録名）
            query = {"model_name": msg.vehicle_name, "record_name": msg.record_name}
            # 新しい数値を追加する
          #  new_data = {
          #      "x": -34.5,  # 新しい x 値
          #      "y": 45.0,   # 新しい y 値
          #      "z": 1.0,    # 新しい z 値
          #      "qx": 0.5,   # 新しい qx 値
          #      "qy": 0.6,   # 新しい qy 値
          #      "qz": 0.7,   # 新しい qz 値
          #      "qw": 0.8    # 新しい qw 値
          #  }
            # 新しいデータを取り出す（msgから必要な情報を抽出）
            new_data = {
                "x": [msg.add_path.pose.position.x],
                "y": [msg.add_path.pose.position.y],
                "z": [msg.add_path.pose.position.z],
                "qx": [msg.add_path.pose.orientation.x],
                "qy": [msg.add_path.pose.orientation.y],
                "qz": [msg.add_path.pose.orientation.z],
                "qw": [msg.add_path.pose.orientation.w]
            }
            # 更新操作（配列に新しい値を追加）
         #   update = {
         #       "$push": {
         #           "x": new_data["x"],
         #           "y": new_data["y"],
         #           "z": new_data["z"],
         #           "qx": new_data["qx"],
         #           "qy": new_data["qy"],
         #           "qz": new_data["qz"],
         #           "qw": new_data["qw"]
         #       }
         #   }
            # 更新操作（配列に新しい値を追加）
            update = {
                "$push": {
                    "x": new_data["x"],
                    "y": new_data["y"],
                    "z": new_data["z"],
                    "qx": new_data["qx"],
                    "qy": new_data["qy"],
                    "qz": new_data["qz"],
                    "qw": new_data["qw"]
                }
            }
            # ドキュメントの更新
            result = collection.update_one(query, update)

            # 結果を表示
            if result.modified_count > 0:
                self.get_logger().info("Document updated successfully.")
            else:
                self.get_logger().info("Document not found or not modified.")
        






def main(args=None):
    rclpy.init(args=args)

    node = TmsDbWriterQuery()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
