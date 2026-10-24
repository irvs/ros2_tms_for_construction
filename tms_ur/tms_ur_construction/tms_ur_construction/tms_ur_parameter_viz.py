#!/usr/bin/env python3

import json
import math

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray

from tms_msg_db.srv import TmsdbGetData


NODE_NAME = "tms_ur_parameter_import_publisher"

DATA_ID = 3012
DATA_TYPE = "parameter"

SUMMARY_RECORD_NAME = (
    "parameter_import_summary_20261023T013934824101"
)


class TmsUrParameterImportPublisher(Node):

    def __init__(self):
        super().__init__(NODE_NAME)

        # ==================================================
        # Parameters
        # ==================================================

        self.declare_parameter("latest", False)

        # ブロックの高さ。DBのblock_sizeにzがない場合に使用する。
        self.declare_parameter("block_height", 0.1)

        self.latest = self.get_parameter(
            "latest"
        ).value

        self.block_height = float(
            self.get_parameter("block_height").value
        )

        # ==================================================
        # Publishers
        # ==================================================

        # 関節角度データ
        self.joint_pub = self.create_publisher(
            String,
            "/joint_angle_params",
            10
        )

        # 建機位置データ
        self.machinery_pub = self.create_publisher(
            String,
            "/machinery_position_params",
            10
        )

        # ブロック形状の可視化データ
        self.block_pub = self.create_publisher(
            MarkerArray,
            "/block_position_params",
            10
        )

        # ブロックの元データ
        # soil_volumeなど、MarkerArrayでは表現できない情報も保持する。
        self.block_json_pub = self.create_publisher(
            String,
            "/block_position_params_json",
            10
        )

        # ==================================================
        # Database client
        # ==================================================

        self.cli = self.create_client(
            TmsdbGetData,
            "tms_db_param_reader"
        )

        while not self.cli.wait_for_service(timeout_sec=0.5):
            self.get_logger().info(
                "Waiting for tms_db_param_reader..."
            )

        # ==================================================
        # Summary
        # ==================================================

        self.summary_received = False

        self.joint_angle_param_names = []
        self.machinery_position_param_names = []
        self.block_position_param_names = []

        # ==================================================
        # Record queue
        # ==================================================

        self.record_queue = []

        self.request_in_progress = False

        # ==================================================
        # Data lists
        # ==================================================

        # 関節角度データ
        # [
        #     {
        #         "record_name": "...",
        #         "data": {...}
        #     },
        #     ...
        # ]
        self.joint_angle_records = []

        # 建機位置データ
        self.machinery_position_records = []

        # ブロック位置データ
        # DBから取得した元のJSONデータをそのまま保存する。
        self.block_position_records = []

        # すべてのデータをpublishしたか
        self.published = False

        # ==================================================
        # Timer
        # ==================================================

        self.timer = self.create_timer(
            0.2,
            self.process_next_record
        )

        self.get_logger().info(
            "Parameter import publisher started."
        )

    # ======================================================
    # Request DB record
    # ======================================================

    def request_record(self, record_name, category):

        req = TmsdbGetData.Request()

        req.type = DATA_TYPE
        req.id = DATA_ID
        req.latest_only = self.latest
        req.flgorparam = "param"

        req.name = record_name
        req.recordnames = [record_name]

        self.request_in_progress = True

        self.get_logger().info(
            f"Requesting [{category}]: {record_name}"
        )

        future = self.cli.call_async(req)

        future.add_done_callback(
            lambda f: self.callback_response(
                f,
                record_name,
                category
            )
        )

    # ======================================================
    # Process records sequentially
    # ======================================================

    def process_next_record(self):

        if self.request_in_progress:
            return

        # 最初にサマリーレコードを取得する。
        if not self.summary_received:

            self.request_record(
                SUMMARY_RECORD_NAME,
                "summary"
            )

            return

        # 全レコードの取得が終了したらpublishする。
        if not self.record_queue:

            if not self.published:
                self.publish_all()

            return

        record_name, category = self.record_queue.pop(0)

        self.request_record(
            record_name,
            category
        )

    # ======================================================
    # DB response
    # ======================================================

    def callback_response(
        self,
        future,
        record_name,
        category
    ):

        self.request_in_progress = False

        try:
            res = future.result()

        except Exception as e:

            self.get_logger().error(
                f"DB request failed: {record_name}: {e}"
            )

            return

        if res is None or not res.tmsdbs:

            self.get_logger().warning(
                f"No DB data: {record_name}"
            )

            return

        try:
            data = json.loads(res.tmsdbs[0].msg)

        except (json.JSONDecodeError, TypeError) as e:

            self.get_logger().error(
                f"JSON parse failed: {record_name}: {e}"
            )

            return

        # --------------------------------------------------
        # Summary
        # --------------------------------------------------

        if category == "summary":

            self.joint_angle_param_names = data.get(
                "joint_angle_param_names", []
            )

            self.machinery_position_param_names = data.get(
                "machinery_position_param_names", []
            )

            self.block_position_param_names = data.get(
                "block_position_param_names", []
            )

            # レコード取得キューを作成する。
            self.record_queue = []

            for name in self.joint_angle_param_names:
                self.record_queue.append(
                    (name, "joint_angle")
                )

            for name in self.machinery_position_param_names:
                self.record_queue.append(
                    (name, "machinery_position")
                )

            for name in self.block_position_param_names:
                self.record_queue.append(
                    (name, "block_position")
                )

            self.summary_received = True

            self.get_logger().info(
                f"Joint angle records: "
                f"{len(self.joint_angle_param_names)}"
            )

            self.get_logger().info(
                f"Machinery position records: "
                f"{len(self.machinery_position_param_names)}"
            )

            self.get_logger().info(
                f"Block position records: "
                f"{len(self.block_position_param_names)}"
            )

            self.get_logger().info(
                f"Total records: {len(self.record_queue)}"
            )

            return

        # --------------------------------------------------
        # Joint angle data
        # --------------------------------------------------

        if category == "joint_angle":

            self.joint_angle_records.append({
                "record_name": record_name,
                "data": data
            })

        # --------------------------------------------------
        # Machinery position data
        # --------------------------------------------------

        elif category == "machinery_position":

            self.machinery_position_records.append({
                "record_name": record_name,
                "data": data
            })

        # --------------------------------------------------
        # Block position data
        # --------------------------------------------------

        elif category == "block_position":

            # Marker変換前の元データを保存する。
            # block_center等がないレコードもここには保存する。
            self.block_position_records.append({
                "record_name": record_name,
                "data": data
            })

        self.get_logger().info(
            f"Received [{category}]: {record_name}"
        )

    # ======================================================
    # Convert block data to MarkerArray
    # ======================================================

    def create_block_marker_array(self, block_records):

        marker_array = MarkerArray()

        stamp = self.get_clock().now().to_msg()

        for index, record in enumerate(block_records):

            record_name = record["record_name"]
            data = record["data"]

            # --------------------------------------------------
            # Check required fields
            # --------------------------------------------------

            required_fields = (
                "block_center",
                "block_size",
                "block_vector"
            )

            if not all(field in data for field in required_fields):

                self.get_logger().warning(
                    f"Skipping Marker conversion for "
                    f"[{record_name}]: "
                    f"block_center, block_size, or "
                    f"block_vector is missing."
                )

                continue

            try:
                center = data["block_center"]
                size = data["block_size"]
                vector = data["block_vector"]

                center_x = float(center["x"])
                center_y = float(center["y"])
                center_z = float(center["z"])

                size_x = float(size["x"])
                size_y = float(size["y"])

                vector_x = float(vector["x"])
                vector_y = float(vector["y"])

                # DBに高さがあればそれを使う。
                # なければblock_heightパラメータを使う。
                size_z = float(
                    size.get("z", self.block_height)
                )

                if size_x <= 0.0 or size_y <= 0.0 or size_z <= 0.0:
                    self.get_logger().warning(
                        f"Skipping Marker for [{record_name}]: "
                        f"block dimensions must be positive."
                    )
                    continue

                # block_vectorの方向からyawを計算する。
                yaw = math.atan2(vector_y, vector_x)

                # ==================================================
                # Block cube marker
                # ==================================================

                marker = Marker()

                marker.header.frame_id = "map"
                marker.header.stamp = stamp

                marker.ns = "block_position"
                marker.id = index * 2

                marker.type = Marker.CUBE
                marker.action = Marker.ADD

                # 中心位置
                marker.pose.position.x = center_x
                marker.pose.position.y = center_y
                marker.pose.position.z = center_z

                # yawからクォータニオンを設定する。
                marker.pose.orientation.x = 0.0
                marker.pose.orientation.y = 0.0
                marker.pose.orientation.z = math.sin(yaw / 2.0)
                marker.pose.orientation.w = math.cos(yaw / 2.0)

                # ブロックサイズ
                marker.scale.x = size_x
                marker.scale.y = size_y
                marker.scale.z = size_z

                # 表示色
                marker.color.r = 0.2
                marker.color.g = 0.7
                marker.color.b = 0.3
                marker.color.a = 0.8

                marker_array.markers.append(marker)

                # ==================================================
                # Block name text marker
                # ==================================================

                text_marker = Marker()

                text_marker.header.frame_id = "map"
                text_marker.header.stamp = stamp

                text_marker.ns = "block_position_labels"
                text_marker.id = index * 2 + 1

                text_marker.type = Marker.TEXT_VIEW_FACING
                text_marker.action = Marker.ADD

                # ブロックの少し上に名前を表示する。
                text_marker.pose.position.x = center_x
                text_marker.pose.position.y = center_y
                text_marker.pose.position.z = (
                    center_z + size_z / 2.0 + 0.2
                )

                text_marker.pose.orientation.x = 0.0
                text_marker.pose.orientation.y = 0.0
                text_marker.pose.orientation.z = 0.0
                text_marker.pose.orientation.w = 1.0

                text_marker.scale.z = 0.3

                text_marker.color.r = 1.0
                text_marker.color.g = 1.0
                text_marker.color.b = 1.0
                text_marker.color.a = 1.0

                text_marker.text = record_name

                marker_array.markers.append(text_marker)

            except (KeyError, TypeError, ValueError) as e:

                self.get_logger().error(
                    f"Invalid block data [{record_name}]: {e}"
                )

                continue

        return marker_array

    # ======================================================
    # Publish all lists
    # ======================================================

    def publish_all(self):

        # --------------------------------------------------
        # Joint angle records
        # --------------------------------------------------

        joint_msg = String()

        joint_msg.data = json.dumps(
            self.joint_angle_records,
            ensure_ascii=False
        )

        self.joint_pub.publish(joint_msg)

        # --------------------------------------------------
        # Machinery position records
        # --------------------------------------------------

        machinery_msg = String()

        machinery_msg.data = json.dumps(
            self.machinery_position_records,
            ensure_ascii=False
        )

        self.machinery_pub.publish(machinery_msg)

        # --------------------------------------------------
        # Block position records: original JSON
        # --------------------------------------------------

        block_json_msg = String()

        block_json_msg.data = json.dumps(
            self.block_position_records,
            ensure_ascii=False
        )

        self.block_json_pub.publish(block_json_msg)

        # --------------------------------------------------
        # Block position records: MarkerArray
        # --------------------------------------------------

        marker_array = self.create_block_marker_array(
            self.block_position_records
        )

        self.block_pub.publish(marker_array)

        # --------------------------------------------------
        # Complete
        # --------------------------------------------------

        self.published = True

        self.get_logger().info(
            "========================================"
        )

        self.get_logger().info(
            f"Published joint angle records: "
            f"{len(self.joint_angle_records)}"
        )

        self.get_logger().info(
            f"Published machinery position records: "
            f"{len(self.machinery_position_records)}"
        )

        self.get_logger().info(
            f"Published block JSON records: "
            f"{len(self.block_position_records)}"
        )

        self.get_logger().info(
            f"Published block markers: "
            f"{len(marker_array.markers)}"
        )

        self.get_logger().info(
            "All parameter lists published."
        )

        self.get_logger().info(
            "========================================"
        )


def main(args=None):

    rclpy.init(args=args)

    node = TmsUrParameterImportPublisher()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()