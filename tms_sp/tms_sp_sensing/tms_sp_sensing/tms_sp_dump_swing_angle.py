import math
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException

from pymongo import MongoClient
from pymongo.errors import PyMongoError, ServerSelectionTimeoutError
import tf2_ros
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException, TransformException

MONGODB_IPADDRESS = '127.0.0.1'
MONGODB_PORTNUMBER = 27017
MONGODB_TIMEOUT_MS = 2000

SWING_ANGLE_RECORD_NAME = 'loading_pose_vessel'

class CalculateRobotPositions(Node):
    def __init__(self):
        super().__init__("calculate_robot_positions")

        # パラメータの宣言と取得
        self.declare_parameter('base_frame', 'base_link_3d')  # バックホウ (ZX200) のフレーム
        self.declare_parameter('excavator_record_name', 'SAMPLE_BLACKBOARD_zx200')  # LOADING_DUMP を持つレコード

        self.base_frame = self.get_parameter('base_frame').get_parameter_value().string_value
        self.excavator_record_name = self.get_parameter('excavator_record_name').get_parameter_value().string_value

        # MongoClient はコネクションプールを持つため、ノードで1つだけ生成して使い回す
        self.mongo_client = MongoClient(
            MONGODB_IPADDRESS, MONGODB_PORTNUMBER,
            serverSelectionTimeoutMS=MONGODB_TIMEOUT_MS
        )
        self.collection = self.mongo_client['rostmsdb']['parameter']

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.timer = self.create_timer(1.0, self.calculate_positions)
        self.get_logger().info(
            f"Tracking LOADING_DUMP of {self.excavator_record_name} relative to {self.base_frame}"
        )

    def destroy_node(self):
        self.mongo_client.close()
        super().destroy_node()

    def calculate_positions(self):
        # バックホウのレコードから積込対象のダンプ名を取得
        try:
            excavator_data = self.collection.find_one({"record_name": self.excavator_record_name})
        except ServerSelectionTimeoutError as e:
            self.get_logger().error(f"Cannot connect to MongoDB: {e}")
            return
        except PyMongoError as e:
            self.get_logger().error(f"Failed to query {self.excavator_record_name}: {e}")
            return

        if not excavator_data:
            self.get_logger().warn(f"Record '{self.excavator_record_name}' not found in database.")
            return

        loading_dump = excavator_data.get("LOADING_DUMP")
        if not isinstance(loading_dump, str) or not loading_dump.strip():
            # 積込を受けるダンプがいない（LOADING_DUMP == ''）
            self.get_logger().info(
                f"No loading dump in {self.excavator_record_name}", throttle_duration_sec=10.0
            )
            return

        # LOADING_DUMP はダンプのレコード名（SAMPLE_BLACKBOARD_mst110cr など）。
        # TF ネームスペースには、そのレコードの model_name（mst110cr, mst2200vd など）を使う
        dump_record_name = loading_dump.strip()
        try:
            dump_data = self.collection.find_one({"record_name": dump_record_name}, {"model_name": 1})
        except PyMongoError as e:
            self.get_logger().error(f"Failed to query {dump_record_name}: {e}")
            return
        robot_name = dump_data.get("model_name") if dump_data else None
        if not isinstance(robot_name, str) or not robot_name:
            self.get_logger().warn(
                f"model_name of LOADING_DUMP record '{dump_record_name}' not found.", throttle_duration_sec=10.0
            )
            return

        # --- ロボットフレーム → 基準フレーム の変換を取得 ---
        try:
            transform_local = self.tf_buffer.lookup_transform(
                f"{robot_name}/base_link",    # ロボット基準
                self.base_frame,              # 基準フレーム
                rclpy.time.Time()
            )
        except LookupException as e:
            self.get_logger().warn(f"TF not available yet ({robot_name}/base_link <- {self.base_frame}): {e}")
            return
        except ConnectivityException as e:
            self.get_logger().warn(f"TF tree not connected ({robot_name}/base_link <- {self.base_frame}): {e}")
            return
        except ExtrapolationException as e:
            self.get_logger().warn(f"TF extrapolation failed ({robot_name}/base_link <- {self.base_frame}): {e}")
            return
        except TransformException as e:
            self.get_logger().warn(f"Failed to lookup TF for {robot_name}: {e}")
            return

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

        self.update_swing_angle(robot_name, yaw_tail)

    def update_swing_angle(self, robot_name: str, swing_angle: float):
        """
        データベースの loading_pose_vessel の target_angle を更新する関数。
        ダンプの種類を問わず同じレコードを使用する。
        typeがdynamic以外の場合は更新を禁止する。
        """
        query = {"record_name": SWING_ANGLE_RECORD_NAME}
        try:
            record = self.collection.find_one(query)

            if not record:
                self.get_logger().warn(f"Record '{SWING_ANGLE_RECORD_NAME}' not found in database.")
                return

            # typeがdynamic以外の場合は更新禁止
            if record.get("type") != "dynamic":
                self.get_logger().warn(
                    f"Record '{SWING_ANGLE_RECORD_NAME}' has type '{record.get('type')}', update forbidden."
                )
                return

            result = self.collection.update_one(query, {"$set": {"target_angle": swing_angle}})
            if result.matched_count == 0:
                self.get_logger().warn(f"Record '{SWING_ANGLE_RECORD_NAME}' disappeared before update.")
                return

            self.get_logger().info(
                f"Updated target_angle of '{SWING_ANGLE_RECORD_NAME}' to {swing_angle:.3f} rad (from {robot_name})."
            )

        except ServerSelectionTimeoutError as e:
            self.get_logger().error(f"Cannot connect to MongoDB: {e}")
        except PyMongoError as e:
            self.get_logger().error(f"Failed to update target_angle of '{SWING_ANGLE_RECORD_NAME}': {e}")

def main(args=None):
    rclpy.init(args=args)
    node = CalculateRobotPositions()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
