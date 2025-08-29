from datetime import datetime
import math
import rclpy
from rclpy.node import Node

from pymongo import MongoClient
import tf2_ros
from tf2_ros import TransformException
from geometry_msgs.msg import TransformStamped

MONGODB_IPADDRESS = '127.0.0.1'
MONGODB_PORTNUMBER = 27017

class CalculateRobotPositions(Node):
    def __init__(self):
        super().__init__("calculate_robot_positions")
        
        # パラメータの宣言と取得
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('robot_names', ['robot1', 'robot2'])
        self.declare_parameter('record_names', ['SAMPLE_BLACKBOARD_robot1', 'SAMPLE_BLACKBOARD_robot2'])
        
        self.base_frame = self.get_parameter('base_frame').get_parameter_value().string_value
        self.robot_names = self.get_parameter('robot_names').get_parameter_value().string_array_value
        self.record_names = self.get_parameter('record_names').get_parameter_value().string_array_value
        
        if len(self.robot_names) != len(self.record_names):
            raise ValueError("robot_names と record_names の長さが一致していません。")

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        self.timer = self.create_timer(1.0, self.calculate_positions)
        self.get_logger().info(f"Tracking robots: {self.robot_names} relative to {self.base_frame}")
    
    def calculate_positions(self):
        client = MongoClient(MONGODB_IPADDRESS, MONGODB_PORTNUMBER)
        db = client['rostmsdb']
        collection = db['parameter']
        
        for robot_name, record_name in zip(self.robot_names, self.record_names):
            try:
                # データベースから record_name と CURRENT_LOCATION が一致するロボットを取得
                query = {
                    "record_name": record_name,
                    "CURRENT_LOCATION": {"$in": [0, "0"]}
                }
                robot_data = collection.find_one(query)
                
                if not robot_data:
                    self.get_logger().info(f"No active robot found for {record_name}")
                    continue
                
                # --- ロボットフレーム → 基準フレーム の変換を取得 ---
                transform_local = self.tf_buffer.lookup_transform(
                    f"{robot_name}/base_link",    # ロボット基準
                    self.base_frame,              # 基準フレーム
                    rclpy.time.Time()
                )

                # ロボットから見た基準フレームの位置
                x_local = transform_local.transform.translation.x
                y_local = transform_local.transform.translation.y

                yaw_local = math.atan2(y_local, x_local)

                # おしりを向けたい → 180度反転
                yaw_tail = yaw_local + math.pi

                # [-π, π] の範囲に正規化（任意）
                yaw_tail = math.atan2(math.sin(yaw_tail), math.cos(yaw_tail))

                self.get_logger().info(
                    f"[Local] {robot_name} sees {self.base_frame} at: "
                    f"x={x_local:.2f}, y={y_local:.2f}, yaw={math.degrees(yaw_tail):.2f} deg"
                )

                # ここで update_swing_angle を呼ぶ
                self.update_swing_angle(yaw_tail)

            except (TransformException, Exception) as e:
                self.get_logger().warn(f"Failed to process robot {robot_name}: {e}")

    def update_swing_angle(self, swing_angle: float):
        """
        データベースのロボット情報にtarget_angleを更新する関数。
        record_nameがSWING_ANGLE_LEFTで、typeがdynamic以外の場合は更新を禁止する。
        """
        client = MongoClient(MONGODB_IPADDRESS, MONGODB_PORTNUMBER)
        db = client['rostmsdb']
        collection = db['parameter']

        for robot_name in self.robot_names:
            try:
                # データベースからロボット情報を取得
                query = {
                    "record_name": "SWING_ANGLE_LEFT"
                }
                robot_data = collection.find_one(query)

                if not robot_data:
                    self.get_logger().warn(f"Robot {robot_name} with record_name 'SWING_ANGLE_LEFT' not found in database.")
                    continue

                # typeがdynamic以外の場合は更新禁止
                if robot_data.get("type") != "dynamic":
                    self.get_logger().warn(f"Robot {robot_name} has type '{robot_data.get('type')}', update forbidden.")
                    continue

                # target_angleを更新（ここで yaw_local が渡される）
                collection.update_one(query, {"$set": {"target_angle": swing_angle}})
                self.get_logger().info(f"Updated target_angle for {robot_name} to {swing_angle:.3f} rad.")

            except Exception as e:
                self.get_logger().error(f"Failed to update target_angle for {robot_name}: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = CalculateRobotPositions()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
