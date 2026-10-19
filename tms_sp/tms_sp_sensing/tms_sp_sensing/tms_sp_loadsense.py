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

"""ダンプトラックの積載量を監視し、しきい値を超えたらexcavation_loading_params(バックホウ用)とSAMPLE_BLACKBOARD_mst110cr(ダンプ用)のSENSING_LOADED_FLGをtrueにするノード.

/load_volume は荷台のLiDARから計算されるため、積み込み直後は砂埃によって
外れ値が多く含まれ、値が落ち着くまで5-10秒ほどかかる.
そのため以下の2段構えでフィルタリングする.

1. 時間窓メディアンフィルタ
   直近 filter_window_sec 秒の値の中央値を積載量とする.
   移動平均と異なり、砂埃による突発的な外れ値に引きずられない.
2. 保持時間付きしきい値判定(ヒステリシス付き)
   フィルタ後の値が load_threshold を threshold_hold_sec 秒間連続で超えた場合のみ
   「積載完了」と判定する. 一度判定した後は load_threshold - threshold_hysteresis を
   下回るまで再判定しない(放土後に再び積み込むサイクルに対応).
"""

from collections import deque
from statistics import median
from typing import Optional

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from pymongo import MongoClient

from mst110cr_loadsense.msg import FloatStamped

EXCAVATION_RECORD_NAME = 'excavation_loading_params'   # バックホウ制御用
DUMP_RECORD_NAME = 'SAMPLE_BLACKBOARD_mst110cr'        # ダンプ制御用(model_nameで台数を区別)


class TmsSpLoadsense(Node):
    def __init__(self):
        super().__init__('tms_sp_loadsense')

        self.declare_parameter('machine_name', '')
        self.declare_parameter('flag_name', 'SENSING_LOADED_FLG')
        self.declare_parameter('load_threshold', 3.0)           # [m^3]
        self.declare_parameter('threshold_hysteresis', 0.5)     # [m^3]
        self.declare_parameter('threshold_hold_sec', 2.0)       # [s]
        self.declare_parameter('filter_window_sec', 5.0)        # [s]
        self.declare_parameter('filter_min_samples', 5)
        self.declare_parameter('volume_change_log_threshold', 0.5)  # [m^3]
        self.declare_parameter('mongodb_ip', '127.0.0.1')
        self.declare_parameter('mongodb_port', 27017)

        self.machine_name = self.get_parameter('machine_name').get_parameter_value().string_value.strip('/')
        self.flag_name = self.get_parameter('flag_name').get_parameter_value().string_value
        self.load_threshold = self.get_parameter('load_threshold').get_parameter_value().double_value
        self.threshold_hysteresis = self.get_parameter('threshold_hysteresis').get_parameter_value().double_value
        self.threshold_hold_sec = self.get_parameter('threshold_hold_sec').get_parameter_value().double_value
        self.filter_window_sec = self.get_parameter('filter_window_sec').get_parameter_value().double_value
        self.filter_min_samples = self.get_parameter('filter_min_samples').get_parameter_value().integer_value
        self.volume_change_log_threshold = \
            self.get_parameter('volume_change_log_threshold').get_parameter_value().double_value
        mongodb_ip = self.get_parameter('mongodb_ip').get_parameter_value().string_value
        mongodb_port = self.get_parameter('mongodb_port').get_parameter_value().integer_value

        self.client = MongoClient(mongodb_ip, mongodb_port)
        self.collection = self.client['rostmsdb']['parameter']
        # 書き換え対象: (表示名, 検索クエリ)
        self.targets = [
            (EXCAVATION_RECORD_NAME,
             {'type': 'dynamic', 'record_name': EXCAVATION_RECORD_NAME}),
        ]
        if self.machine_name:
            self.targets.append(
                (f'{DUMP_RECORD_NAME}[{self.machine_name}]',
                 {'type': 'dynamic', 'record_name': DUMP_RECORD_NAME, 'model_name': self.machine_name}))
        else:
            self.get_logger().warn(
                f'machine_name is empty. {DUMP_RECORD_NAME} will not be updated '
                f'because it is selected by model_name.')

        # (受信時刻[s], 積載量) の時間窓バッファ
        self.samples = deque()
        self.last_logged_volume: Optional[float] = None
        self.above_since: Optional[float] = None
        self.load_completed = False

        topic_name = f'/{self.machine_name}/load_volume' if self.machine_name else '/load_volume'
        self.subscription = self.create_subscription(
            FloatStamped,
            topic_name,
            self.load_volume_callback,
            10)

        self.get_logger().info(
            f'tms_sp_loadsense started: topic={topic_name}, flag={self.flag_name}, '
            f'targets={[name for name, _ in self.targets]}, '
            f'threshold={self.load_threshold:.3f} m^3, window={self.filter_window_sec:.1f} s, '
            f'hold={self.threshold_hold_sec:.1f} s')
        self.ensure_flag_exists()

    def ensure_flag_exists(self) -> None:
        """対象レコードにフラグが無ければfalseで追加する(既存の値は変更しない)."""
        for name, query in self.targets:
            try:
                if self.collection.find_one(query) is None:
                    self.get_logger().warn(f'No matching record found: {name}')
                    continue
                result = self.collection.update_one(
                    {**query, self.flag_name: {'$exists': False}},
                    {'$set': {self.flag_name: False}})
                if result.modified_count > 0:
                    self.get_logger().info(f'Added {name}.{self.flag_name} = False')
            except Exception as e:
                self.get_logger().error(f'Failed to ensure {name}.{self.flag_name}: {e}')

    def load_volume_callback(self, msg: FloatStamped) -> None:
        now = self.get_clock().now().nanoseconds * 1e-9
        self.samples.append((now, float(msg.data)))
        while self.samples and now - self.samples[0][0] > self.filter_window_sec:
            self.samples.popleft()

        if len(self.samples) < self.filter_min_samples:
            return
        filtered_volume = median(v for _, v in self.samples)

        if self.last_logged_volume is None or \
                abs(filtered_volume - self.last_logged_volume) >= self.volume_change_log_threshold:
            prev = 'None' if self.last_logged_volume is None else f'{self.last_logged_volume:.3f}'
            self.get_logger().info(
                f'Load volume changed: {prev} -> {filtered_volume:.3f} m^3 '
                f'(raw: {msg.data:.3f} m^3)')
            self.last_logged_volume = filtered_volume

        self.judge_threshold(filtered_volume, now)

    def judge_threshold(self, filtered_volume: float, now: float) -> None:
        if self.load_completed:
            if filtered_volume < self.load_threshold - self.threshold_hysteresis:
                self.load_completed = False
                self.above_since = None
                self.get_logger().info(
                    f'Load volume dropped below {self.load_threshold - self.threshold_hysteresis:.3f} m^3. '
                    f'Ready for next loading.')
            return

        if filtered_volume <= self.load_threshold:
            self.above_since = None
            return

        if self.above_since is None:
            self.above_since = now
        if now - self.above_since >= self.threshold_hold_sec:
            self.load_completed = True
            self.set_flag_true(filtered_volume)

    def set_flag_true(self, filtered_volume: float) -> None:
        self.get_logger().info(
            f'Load volume {filtered_volume:.3f} m^3 exceeded threshold {self.load_threshold:.3f} m^3.')
        for name, query in self.targets:
            try:
                parameter_info = self.collection.find_one(query)
                if parameter_info is None:
                    self.get_logger().warn(f'No matching record found: {name}')
                    continue
                old_value = parameter_info.get(self.flag_name)
                if old_value is True:
                    self.get_logger().info(f'{name}.{self.flag_name} is already True.')
                    continue
                self.collection.update_one(query, {'$set': {self.flag_name: True}})
            except Exception as e:
                self.get_logger().error(f'Failed to update {name}.{self.flag_name}: {e}')
                continue
            self.get_logger().info(f'Updated {name}.{self.flag_name}: {old_value} -> True')


def main(args=None):
    rclpy.init(args=args)
    node = TmsSpLoadsense()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
