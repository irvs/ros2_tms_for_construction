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
from sensing_msgs.msg import Flgs

import tms_db_manager.tms_db_util as db_util

MONGODB_IPADDRESS = '127.0.0.1'
MONGODB_PORTNUMBER = 27017

DATA_ID = 10000
DATA_TYPE = 'parameter' 
DATA_NAME = 'data_name'

class UpdateDB_Parameter(Node):
    def __init__(self):
        super().__init__("tms_sp_flgs")
        self.subscription = self.create_subscription(
            Flgs,
            '/Flgs',
            self.update_db_parameter,
            10) 
    
    def update_db_parameter(self, msg: Flgs) -> None:
        client = MongoClient(MONGODB_IPADDRESS, MONGODB_PORTNUMBER)
        db = client['rostmsdb']
        collection = db['parameter']
        query = {"type" : "dynamic", "record_name": msg.record_name}
        parameter_info = collection.find_one(query)
        update_parameter_info = {}
        msg_fields_and_types = msg.get_fields_and_field_types()
        msg_fields = list(msg_fields_and_types.keys())

        # ログ用のリスト
        updated_fields = []
        kept_fields = []

        # msgに含まれているフィールドのみを更新対象とする
        for field in msg_fields:
            if field == 'record_name' or field == 'keep_val_flgs':
                continue  # record_nameとkeep_val_flgsは更新対象外
            
            # parameter_infoに対応するキーがあるかチェック
            matching_key = None
            for key in parameter_info.keys():
                if key.lower() == field.lower():
                    matching_key = key
                    break
            
            if matching_key and matching_key not in ["_id", "model_name", "type", "record_name"]:
                if matching_key in msg.keep_val_flgs:
                    # keep_val_flgsに含まれている場合は既存の値を保持
                    old_value = parameter_info[matching_key]
                    update_parameter_info[matching_key] = old_value
                    kept_fields.append(f"{matching_key}: {old_value}")
                else:
                    # msgの値で更新
                    new_value = getattr(msg, field)
                    old_value = parameter_info[matching_key]
                    update_parameter_info[matching_key] = new_value
                    updated_fields.append(f"{matching_key}: {old_value} -> {new_value}")

        update_query = {"$set": update_parameter_info}
        result = collection.update_one(query, update_query)
        
        # 更新結果のログ出力
        if result.matched_count > 0:
            log_msg = f"Successfully updated parameters for record_name: {msg.record_name}"
            if updated_fields:
                log_msg += f", Updated: [{', '.join(updated_fields)}]"
            if kept_fields:
                log_msg += f", Kept: [{', '.join(kept_fields)}]"
            self.get_logger().info(log_msg)
        else:
            self.get_logger().warn(f"No matching record found for record_name: {msg.record_name}")

def main(args=None):
    rclpy.init(args=args)
    update_db_parameter = UpdateDB_Parameter()
    rclpy.spin(update_db_parameter)
    update_db_parameter.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()