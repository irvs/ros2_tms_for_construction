#!/usr/bin/env python3

import json
import math

import rclpy
from rclpy.node import Node

from std_msgs.msg import String, ColorRGBA
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray

from tms_msg_db.srv import TmsdbGetData


NODE_NAME = "tms_ur_parameter_import_publisher"

DATA_ID = 3012
DATA_TYPE = "parameter"


class TmsUrParameterImportPublisher(Node):

    def __init__(self):
        super().__init__(NODE_NAME)

        # ==================================================
        # Parameters
        # ==================================================

        self.declare_parameter("latest", False)

        # ブロックの高さ。DBのblock_sizeにzがない場合に使用する。
        self.declare_parameter("block_height", 0.1)

        # データベースの再読み出し周期（秒）
        self.declare_parameter("refresh_period", 30.0)

        self.latest = self.get_parameter(
            "latest"
        ).value

        self.block_height = float(
            self.get_parameter("block_height").value
        )

        self.refresh_period = float(
            self.get_parameter("refresh_period").value
        )

        if self.refresh_period <= 0.0:
            raise ValueError("refresh_period must be greater than 0")

        # ==================================================
        # Publishers
        # ==================================================

        # 関節角度データ
        self.joint_pub = self.create_publisher(
            String,
            "/joint_angle_params",
            10
        )

        # 建機位置データの可視化（WaypointVisualizerと同様にSPHERE_LISTでpublish）
        self.machinery_pub = self.create_publisher(
            Marker,
            "/machinery_position_params",
            10
        )

        # 建機位置データの元JSONも必要に応じて利用できるよう別トピックでpublish
        self.machinery_json_pub = self.create_publisher(
            String,
            "/machinery_position_params_json",
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

        # GeoFenceの境界線
        self.geo_fence_pub = self.create_publisher(
            Marker,
            "/geo_fence_params",
            10
        )

        # GeoFenceの元データ
        self.geo_fence_json_pub = self.create_publisher(
            String,
            "/geo_fence_params_json",
            10
        )

        # 接続線の可視化用
        self.machinery_connection_pub = self.create_publisher(
            Marker,
            "/machinery_connection_params",
            10
        )

        # 接続関係のJSONデータ用
        self.machinery_connection_json_pub = self.create_publisher(
            String,
            "/machinery_connection_params_json",
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
        self.machinery_connection_records = []

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
        # GeoFenceデータ
        self.geo_fence_records = []

        # すべてのデータをpublishしたか
        self.published = False


        # 再読み出し処理中か
        self.refresh_in_progress = False

        # 次回の更新開始時刻
        self.next_refresh_time = self.get_clock().now()

        # ==================================================
        # Timer
        # ==================================================

        self.timer = self.create_timer(
            0.2,
            self.process_next_record
        )

        # self.get_logger().info(
        #     "Parameter import publisher started."
        # )

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

        # self.get_logger().info(
        #     f"Requesting [{category}]: {record_name}"
        # )

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

        # DBリクエスト中は次の処理を開始しない
        if self.request_in_progress:
            return

        # --------------------------------------------------
        # 前回の取得が完了している場合、次回の更新時刻を確認
        # --------------------------------------------------
        if self.published:
            now = self.get_clock().now()

            if now < self.next_refresh_time:
                return

            self.get_logger().info(
                "Starting periodic database refresh."
            )

            self.start_refresh()

        # --------------------------------------------------
        # サマリーを取得していない場合
        # --------------------------------------------------
        if not self.summary_received:
            self.request_record(
                "__LATEST_PARAMETER_IMPORT_SUMMARY__",
                "summary"
            )
            return

        # --------------------------------------------------
        # 各レコードを順番に取得
        # --------------------------------------------------
        if self.record_queue:
            record_name, category = self.record_queue.pop(0)

            self.request_record(
                record_name,
                category
            )
            return

        # --------------------------------------------------
        # 全レコードの取得が完了したらpublish
        # --------------------------------------------------
        if not self.published:
            self.publish_all()

            self.published = True
            self.refresh_in_progress = False

            self.next_refresh_time = (
                self.get_clock().now()
                + rclpy.duration.Duration(
                    seconds=self.refresh_period
                )
            )

            self.get_logger().info(
                f"Database refresh completed. "
                f"Joint angle records: "
                f"{len(self.joint_angle_records)}, "
                f"Machinery position records: "
                f"{len(self.machinery_position_records)}, "
                f"Block position records: "
                f"{len(self.block_position_records)}, "
                f"Block markers: "
                f"{len(self.create_block_marker_array(self.block_position_records).markers)}"
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
            
            # GeoFenceを追加
            self.record_queue.append(
                ("geo_fence", "geo_fence")
            )

            connection_names = data.get(
                "machinery_connection_param_names", []
            )

            for record_name in connection_names:
                self.record_queue.append(
                    (record_name, "machinery_connection")
                )

            self.summary_received = True

            # self.get_logger().info(
            #     f"Joint angle records: "
            #     f"{len(self.joint_angle_param_names)}"
            # )

            # self.get_logger().info(
            #     f"Machinery position records: "
            #     f"{len(self.machinery_position_param_names)}"
            # )

            # self.get_logger().info(
            #     f"Block position records: "
            #     f"{len(self.block_position_param_names)}"
            # )

            # self.get_logger().info(
            #     f"Total records: {len(self.record_queue)}"
            # )

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

        elif category == "geo_fence":

            self.geo_fence_records.append({
                "record_name": record_name,
                "data": data
            })

        elif category == "machinery_connection":

            self.machinery_connection_records.append({
                "record_name": record_name,
                "data": data
            })

        # self.get_logger().info(
        #     f"Received [{category}]: {record_name}"
        # )

    # ======================================================
    # Convert machinery positions to a Marker (SPHERE_LIST)
    # ======================================================

    def create_machinery_marker(self, machinery_records):
        marker = Marker()

        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "machinery_positions"
        marker.id = 0
        marker.type = Marker.SPHERE_LIST
        marker.action = Marker.ADD

        marker.scale.x = 1.0
        marker.scale.y = 1.0
        marker.scale.z = 1.0

        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        marker.pose.orientation.w = 1.0

        for record in machinery_records:
            record_name = record.get("record_name", "")
            data = record.get("data", {})

            try:
                # x, y, z が配列なら、各インデックスの座標をすべて使用する。
                # 数値（スカラー）なら、1点分の座標として扱う。
                def as_coordinate_list(value, default=None):
                    if value is None:
                        return default
                    if isinstance(value, list):
                        return value
                    return [value]

                x_values = as_coordinate_list(data.get("x"))
                y_values = as_coordinate_list(data.get("y"))
                z_values = as_coordinate_list(data.get("z", 0.0))

                if not x_values or not y_values:
                    raise ValueError("x or y is empty/missing")

                # zがない、または配列の要素数が足りない場合は0.0を使用する。
                # xとyは対応するインデックス同士で組み合わせる。
                point_count = min(len(x_values), len(y_values))

                if len(x_values) != len(y_values):
                    self.get_logger().warning(
                        f"[{record_name}] x/y lengths differ "
                        f"({len(x_values)}/{len(y_values)}); "
                        f"using {point_count} paired points."
                    )

                for i in range(point_count):
                    x = float(x_values[i])
                    y = float(y_values[i])

                    if i < len(z_values):
                        z = float(z_values[i])
                    else:
                        z = 0.0

                    
                    # NaN や Infinity を含む座標は送信しない
                    if not all(math.isfinite(v) for v in (x, y, z)):
                        self.get_logger().warning(
                            f"Skipping non-finite point [{record_name}] "
                            f"index={i}: x={x}, y={y}, z={z}"
                        )
                        continue

                    marker.points.append(Point(x=x, y=y, z=z))

                    point_color = ColorRGBA()
                    point_color.r = 1.0
                    point_color.g = 0.0
                    point_color.b = 0.0
                    point_color.a = 1.0
                    marker.colors.append(point_color)

            except (KeyError, IndexError, TypeError, ValueError) as e:
                self.get_logger().warning(
                    f"Skipping machinery marker [{record_name}]: {e}"
                )

        return marker

    # ======================================================
    # Convert block data to MarkerArray
    # ======================================================

    def create_block_marker_array(self, block_records):

        marker_array = MarkerArray()
        stamp = self.get_clock().now().to_msg()

        for index, record in enumerate(block_records):

            record_name = record["record_name"]
            data = record["data"]

            required_fields = (
                "block_center",
                "block_size",
                "block_vector"
            )

            if not all(field in data for field in required_fields):
                self.get_logger().warning(
                    f"Skipping Marker conversion for [{record_name}]: "
                    "block_center, block_size, or block_vector is missing."
                )
                continue

            try:
                center = data["block_center"]
                size = data["block_size"]
                vector = data["block_vector"]

                # block_centerがリストなら先頭要素を使用する
                if isinstance(center, list):
                    if not center:
                        raise ValueError(
                            "block_center is an empty list"
                        )
                    center = center[0]

                if not isinstance(center, dict):
                    raise ValueError(
                        "block_center must be a dict or a list of dicts"
                    )

                center_x = float(center["x"])
                center_y = float(center["y"])
                center_z = float(center["z"])

                size_x = float(size["x"])
                size_y = float(size["y"])
                size_z = float(
                    size.get("z", self.block_height)
                )

                vector_x = float(vector["x"])
                vector_y = float(vector["y"])

                if size_x <= 0.0 or size_y <= 0.0 or size_z <= 0.0:
                    raise ValueError(
                        "block dimensions must be positive"
                    )

                # block_vectorを正規化する
                vector_length = math.hypot(vector_x, vector_y)

                if vector_length == 0.0:
                    raise ValueError(
                        "block_vector must not be zero"
                    )

                # block_vectorをX方向として使用する
                axis_x = vector_x
                axis_y = vector_y

                # block_vectorに直交する水平面上のY方向
                axis_y_perp_x = -vector_y
                axis_y_perp_y = vector_x

                # block_centerを底面の角として扱う
                # 底面の4頂点
                p0 = (center_x, center_y, center_z)

                p1 = (
                    center_x + axis_x * size_x,
                    center_y + axis_y * size_x,
                    center_z
                )

                p2 = (
                    center_x + axis_y_perp_x * size_y,
                    center_y + axis_y_perp_y * size_y,
                    center_z
                )

                p3 = (
                    center_x + axis_x * size_x
                    + axis_y_perp_x * size_y,
                    center_y + axis_y * size_x
                    + axis_y_perp_y * size_y,
                    center_z
                )

                # 通常のブロックは従来どおりCUBEで描画
                # excavation_loading_paramsは傾斜形状で描画
                is_excavation = (
                    record_name == "excavation_loading_params"
                    or "block_angle" in data
                )

                if is_excavation:
                    block_angle = float(data["block_angle"])

                    # 0 < block_angle <= 180度の範囲を想定
                    if not (0.0 < block_angle <= math.pi):
                        raise ValueError(
                            "block_angle must be in (0, pi]"
                        )

                    # block_angleが90度の場合はずれを0とする
                    if math.isclose(
                        block_angle,
                        math.pi / 2.0,
                        abs_tol=1e-6
                    ):
                        offset = 0.0
                    else:
                        tan_angle = math.tan(block_angle)

                        if abs(tan_angle) < 1e-9:
                            raise ValueError(
                                "block_angle produces an invalid offset"
                            )

                        offset = size_z / tan_angle

                    # 上面をblock_vectorの反対方向へずらす
                    shift_x = -vector_x * offset
                    shift_y = -vector_y * offset

                    # 上面の4頂点
                    p4 = (
                        p0[0] + shift_x,
                        p0[1] + shift_y,
                        center_z + size_z
                    )

                    p5 = (
                        p1[0] + shift_x,
                        p1[1] + shift_y,
                        center_z + size_z
                    )

                    p6 = (
                        p2[0] + shift_x,
                        p2[1] + shift_y,
                        center_z + size_z
                    )

                    p7 = (
                        p3[0] + shift_x,
                        p3[1] + shift_y,
                        center_z + size_z
                    )

                    # TRIANGLE_LISTで6面を構築する
                    marker = Marker()
                    marker.header.frame_id = "map"
                    marker.header.stamp = stamp
                    marker.ns = "block_position"
                    marker.id = index * 2
                    marker.type = Marker.TRIANGLE_LIST
                    marker.action = Marker.ADD

                    # TRIANGLE_LISTのscaleを設定する
                    marker.scale.x = 1.0
                    marker.scale.y = 1.0
                    marker.scale.z = 1.0

                    marker.pose.orientation.w = 1.0

                    # 頂点座標はmap座標で計算済みなので、
                    # Markerのpose.positionは原点にする
                    marker.pose.position.x = 0.0
                    marker.pose.position.y = 0.0
                    marker.pose.position.z = 0.0

                    # 各面を2つの三角形で構成
                    faces = [
                        # 底面
                        (p0, p2, p1),
                        (p1, p2, p3),

                        # 上面
                        (p4, p5, p6),
                        (p5, p7, p6),

                        # X方向の側面
                        (p0, p1, p4),
                        (p1, p5, p4),

                        (p2, p6, p3),
                        (p3, p6, p7),

                        # Y方向の側面
                        (p0, p4, p2),
                        (p2, p4, p6),

                        (p1, p3, p5),
                        (p3, p7, p5),
                    ]

                    for triangle in faces:
                        for vertex in triangle:
                            marker.points.append(
                                Point(
                                    x=float(vertex[0]),
                                    y=float(vertex[1]),
                                    z=float(vertex[2])
                                )
                            )

                    marker.color.r = 0.2
                    marker.color.g = 0.7
                    marker.color.b = 0.3
                    marker.color.a = 0.8

                    marker_array.markers.append(marker)

                else:
                    # 通常のブロックはCUBEを使用
                    yaw = math.atan2(vector_y, vector_x)

                    marker = Marker()
                    marker.header.frame_id = "map"
                    marker.header.stamp = stamp
                    marker.ns = "block_position"
                    marker.id = index * 2
                    marker.type = Marker.CUBE
                    marker.action = Marker.ADD

                    # 通常ブロックではblock_centerを中心として扱う
                    marker.pose.position.x = center_x
                    marker.pose.position.y = center_y
                    marker.pose.position.z = center_z

                    marker.pose.orientation.x = 0.0
                    marker.pose.orientation.y = 0.0
                    marker.pose.orientation.z = math.sin(yaw / 2.0)
                    marker.pose.orientation.w = math.cos(yaw / 2.0)

                    marker.scale.x = size_x
                    marker.scale.y = size_y
                    marker.scale.z = size_z

                    marker.color.r = 0.2
                    marker.color.g = 0.7
                    marker.color.b = 0.3
                    marker.color.a = 0.8

                    marker_array.markers.append(marker)

                # ブロック名を表示するテキストMarker
                text_marker = Marker()
                text_marker.header.frame_id = "map"
                text_marker.header.stamp = stamp
                text_marker.ns = "block_position_labels"
                text_marker.id = index * 2 + 1
                text_marker.type = Marker.TEXT_VIEW_FACING
                text_marker.action = Marker.ADD

                # ブロックの中央付近に名前を表示
                text_marker.pose.position.x = (
                    center_x
                    + axis_x * size_x / 2.0
                    + axis_y_perp_x * size_y / 2.0
                )
                text_marker.pose.position.y = (
                    center_y
                    + axis_y * size_x / 2.0
                    + axis_y_perp_y * size_y / 2.0
                )
                text_marker.pose.position.z = center_z + size_z + 0.2

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

    def create_geo_fence_marker(self, geo_fence_records):

        marker = Marker()

        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()

        marker.ns = "geo_fence"
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD

        # 線の太さ
        marker.scale.x = 0.2

        # 線の色：黄色
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        marker.pose.orientation.w = 1.0

        for record in geo_fence_records:

            data = record.get("data", {})
            record_name = record.get("record_name", "")

            try:
                coordinates = data["coordinates"]

                # GeoJSON形式に合わせて、座標配列を処理する。
                # 今回のデータでは coordinates[0] が境界線の座標列。
                if not coordinates or not coordinates[0]:
                    raise ValueError("coordinates is empty")

                points = coordinates[0]

                for point in points:
                    marker.points.append(
                        Point(
                            x=float(point["x"]),
                            y=float(point["y"]),
                            z=float(point.get("z", 0.0))
                        )
                    )

            except (KeyError, TypeError, ValueError, IndexError) as e:

                self.get_logger().error(
                    f"Invalid GeoFence data [{record_name}]: {e}"
                )

        return marker

    def create_machinery_connection_marker(self, records):
        marker = Marker()

        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()

        marker.ns = "machinery_connections"
        marker.id = 0
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD

        # 線の太さ
        marker.scale.x = 0.15

        # 黄色
        # オレンジレッド（機械位置の赤色と区別しやすい色）
        marker.color.r = 1.0
        marker.color.g = 0.25
        marker.color.b = 0.0
        marker.color.a = 1.0

        # ノードID -> 座標
        node_positions = {}

        for record in records:
            data = record.get("data", {})

            node_id = data.get("node_id")
            position = data.get("position")

            if node_id is None or not isinstance(position, dict):
                continue

            try:
                node_positions[str(node_id)] = Point(
                    x=float(position["x"]),
                    y=float(position["y"]),
                    z=float(position.get("z", 0.0))
                )
            except (KeyError, TypeError, ValueError):
                self.get_logger().warning(
                    f"Invalid node position: {node_id}"
                )

        # 同じ接続を重複して描画しないための集合
        drawn_edges = set()

        for record in records:
            data = record.get("data", {})

            source_id = data.get("node_id")
            connect_nodes = data.get("connect_node", [])

            if source_id is None or not isinstance(connect_nodes, list):
                continue

            source_id = str(source_id)

            if source_id not in node_positions:
                continue

            for target_id in connect_nodes:
                target_id = str(target_id)

                if target_id not in node_positions:
                    self.get_logger().warning(
                        f"Connection target not found: "
                        f"{source_id} -> {target_id}"
                    )
                    continue

                if source_id == target_id:
                    continue

                # A -> B と B -> A は同じ線として扱う
                edge = tuple(sorted((source_id, target_id)))

                if edge in drawn_edges:
                    continue

                drawn_edges.add(edge)

                # LINE_LISTでは、2点ずつ追加すると1本の線になる
                marker.points.append(
                    node_positions[source_id]
                )
                marker.points.append(
                    node_positions[target_id]
                )

        return marker


    def start_refresh(self):

        # 新しい読み出し処理を開始
        self.summary_received = False
        self.published = False
        self.refresh_in_progress = True

        self.record_queue.clear()

        # 古いデータが新しいデータに混ざらないように初期化
        self.joint_angle_records.clear()
        self.machinery_position_records.clear()
        self.block_position_records.clear()
        self.geo_fence_records.clear()

        self.joint_angle_param_names.clear()
        self.machinery_position_param_names.clear()
        self.block_position_param_names.clear()
        self.machinery_connection_records.clear()

        # self.get_logger().info(
        #     "Cleared cached records. Reloading summary."
        # )

        # 次のタイマー処理でサマリー取得を開始する


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
        # Machinery position records: Marker (SPHERE_LIST)
        # --------------------------------------------------

        machinery_marker = self.create_machinery_marker(
            self.machinery_position_records
        )
        self.machinery_pub.publish(machinery_marker)

        # 元の建機位置データもJSONとして別トピックにpublishする。
        machinery_json_msg = String()
        machinery_json_msg.data = json.dumps(
            self.machinery_position_records,
            ensure_ascii=False
        )
        self.machinery_json_pub.publish(machinery_json_msg)

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
        # GeoFence: JSON
        # --------------------------------------------------

        geo_fence_json_msg = String()

        geo_fence_json_msg.data = json.dumps(
            self.geo_fence_records,
            ensure_ascii=False
        )

        self.geo_fence_json_pub.publish(
            geo_fence_json_msg
        )

        # --------------------------------------------------
        # GeoFence: Marker
        # --------------------------------------------------

        geo_fence_marker = self.create_geo_fence_marker(
            self.geo_fence_records
        )

        self.geo_fence_pub.publish(
            geo_fence_marker
        )

        # ------------------------------------------
        # Machinery connection visualization
        # ------------------------------------------
        connection_marker = self.create_machinery_connection_marker(
            self.machinery_connection_records
        )

        # MarkerとしてPublish
        self.machinery_connection_pub.publish(connection_marker)

        # StringとしてJSONをPublish
        connection_json_msg = String()
        connection_json_msg.data = json.dumps(
            self.machinery_connection_records,
            ensure_ascii=False
        )

        self.machinery_connection_json_pub.publish(
            connection_json_msg
        )

        # self.get_logger().info(
        #     f"Published machinery connections: "
        #     f"{len(connection_marker.points) // 2}"
        # )

        # self.get_logger().info(
        #     f"Published machinery connection JSON records: "
        #     f"{len(self.machinery_connection_records)}"
        # )

        # self.get_logger().info(
        #     f"Published GeoFence records: "
        #     f"{len(self.geo_fence_records)}"
        # )

        # self.get_logger().info(
        #     f"Published GeoFence points: "
        #     f"{len(geo_fence_marker.points)}"
        # )

        # --------------------------------------------------
        # Complete
        # --------------------------------------------------

        self.published = True

        # self.get_logger().info(
        #     "========================================"
        # )

        # self.get_logger().info(
        #     f"Published joint angle records: "
        #     f"{len(self.joint_angle_records)}"
        # )

        # self.get_logger().info(
        #     f"Published machinery position records: "
        #     f"{len(self.machinery_position_records)}"
        # )

        # self.get_logger().info(
        #     f"Published machinery position markers: "
        #     f"{len(machinery_marker.points)} points"
        # )

        # self.get_logger().info(
        #     f"Published block JSON records: "
        #     f"{len(self.block_position_records)}"
        # )

        # self.get_logger().info(
        #     f"Published block markers: "
        #     f"{len(marker_array.markers)}"
        # )

        # self.get_logger().info(
        #     "All parameter lists published."
        # )

        # self.get_logger().info(
        #     "========================================"
        # )


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