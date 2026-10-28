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

"""ダンプの積載量を監視し、ダンプ自身のレコード(SAMPLE_BLACKBOARD_<machine_name>)のSENSING_LOADED_FLGを更新するノード.

/load_volume は荷台のLiDARから計算されるため、積み込み直後は砂埃によって
外れ値が多く含まれ、値が落ち着くまで5-10秒ほどかかる.
そのため以下の2段構えでフィルタリングする.

1. 時間窓メディアンフィルタ
   直近 filter_window_sec 秒の値の中央値を積載量 V_f とする.
   移動平均と異なり、砂埃による突発的な外れ値に引きずられない.
2. 保持時間付きしきい値判定(ヒステリシス付き)
   V_f がしきい値を threshold_hold_sec 秒間連続で超えた場合のみ「積載完了(true)」と判定する.
   一度判定した後は しきい値 - threshold_hysteresis を下回るまで true を保ち、
   下回ったら false に戻す(放土場で荷台を空けたとき).

しきい値は、目標積載量から、まだ V_f に反映されていない土の量(見込み)を引いた値にする.
バックホウは放土して初期姿勢に戻った直後にこのフラグを読むため、その時点で入れた1杯は V_f に反映されていない.
見込みが無いと、積み終わっているのに「足りない」と判断して1杯多く積んでしまう.

    しきい値 = V_target - lookahead_volume
    true     : V_f >= しきい値 が threshold_hold_sec 秒続いたとき
    false    : V_f < しきい値 - threshold_hysteresis

    V_target         : ダンプのレコードの TargetSoilVolume (無い場合は load_threshold)
    lookahead_volume : まだ V_f に反映されていない土の量 [m^3]. 目安は1杯分. 0 にすると見込みなし

フラグは値が変わったときだけ書き込む. 起動後に最初に判定できたときは、レコードの値に関わらず1回書き込む.
"""

from collections import deque
from statistics import median
from typing import Optional

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from pymongo import MongoClient

from mst110cr_loadsense.msg import FloatStamped

DUMP_RECORD_PREFIX = 'SAMPLE_BLACKBOARD_'   # ダンプ自身のレコード: SAMPLE_BLACKBOARD_<machine_name>


class TmsSpLoadsense(Node):
    def __init__(self):
        super().__init__('tms_sp_loadsense')

        self.declare_parameter('machine_name', '')
        self.declare_parameter('record_name', '')                # 空なら SAMPLE_BLACKBOARD_<machine_name>
        self.declare_parameter('flag_name', 'SENSING_LOADED_FLG')
        self.declare_parameter('target_volume_field', 'TargetSoilVolume')
        self.declare_parameter('target_reload_sec', 5.0)        # [s] TargetSoilVolume を読み直す周期
        self.declare_parameter('load_threshold', 3.0)           # [m^3] TargetSoilVolume が無いときの目標積載量
        self.declare_parameter('lookahead_volume', 0.5)         # [m^3] V_f に反映されていない土の見込み(目安は1杯分)
        self.declare_parameter('threshold_hysteresis', 0.5)     # [m^3]
        self.declare_parameter('threshold_hold_sec', 2.0)       # [s]
        self.declare_parameter('filter_window_sec', 5.0)        # [s]
        self.declare_parameter('filter_min_samples', 5)
        self.declare_parameter('volume_change_log_threshold', 0.5)  # [m^3]
        self.declare_parameter('mongodb_ip', '127.0.0.1')
        self.declare_parameter('mongodb_port', 27017)
        self.declare_parameter('mongodb_database', 'rostmsdb')

        self.machine_name = self.get_parameter('machine_name').get_parameter_value().string_value.strip('/')
        record_name = self.get_parameter('record_name').get_parameter_value().string_value
        self.record_name = record_name if record_name else DUMP_RECORD_PREFIX + self.machine_name
        self.flag_name = self.get_parameter('flag_name').get_parameter_value().string_value
        self.target_volume_field = self.get_parameter('target_volume_field').get_parameter_value().string_value
        self.target_reload_sec = self.get_parameter('target_reload_sec').get_parameter_value().double_value
        self.load_threshold = self.get_parameter('load_threshold').get_parameter_value().double_value
        self.lookahead_volume = self.get_parameter('lookahead_volume').get_parameter_value().double_value
        self.threshold_hysteresis = self.get_parameter('threshold_hysteresis').get_parameter_value().double_value
        self.threshold_hold_sec = self.get_parameter('threshold_hold_sec').get_parameter_value().double_value
        self.filter_window_sec = self.get_parameter('filter_window_sec').get_parameter_value().double_value
        self.filter_min_samples = self.get_parameter('filter_min_samples').get_parameter_value().integer_value
        self.volume_change_log_threshold = \
            self.get_parameter('volume_change_log_threshold').get_parameter_value().double_value
        mongodb_ip = self.get_parameter('mongodb_ip').get_parameter_value().string_value
        mongodb_port = self.get_parameter('mongodb_port').get_parameter_value().integer_value
        mongodb_database = self.get_parameter('mongodb_database').get_parameter_value().string_value

        if not self.machine_name:
            self.get_logger().warn('machine_name is empty. Subscribing /load_volume.')

        self.client = MongoClient(mongodb_ip, mongodb_port)
        self.collection = self.client[mongodb_database]['parameter']
        self.query = {'record_name': self.record_name}

        # (受信時刻[s], 積載量) の時間窓バッファ
        self.samples = deque()
        self.last_logged_volume: Optional[float] = None
        self.above_since: Optional[float] = None
        self.load_completed = False
        self.written_flag: Optional[bool] = None    # 最後に書き込んだ値(None: まだ書いていない)

        # 目標積載量(TargetSoilVolume)のキャッシュ
        self.target_volume = self.load_threshold
        self.target_from_record: Optional[bool] = None   # None: まだ読んでいない
        self.target_read_at: Optional[float] = None

        topic_name = f'/{self.machine_name}/load_volume' if self.machine_name else '/load_volume'
        self.subscription = self.create_subscription(
            FloatStamped,
            topic_name,
            self.load_volume_callback,
            10)

        self.ensure_flag_exists()
        self.update_target_volume(self.get_clock().now().nanoseconds * 1e-9)
        self.get_logger().info(
            f'tms_sp_loadsense started: topic={topic_name}, record={self.record_name}, flag={self.flag_name}, '
            f'threshold={self.threshold():.3f} m^3 (target={self.target_volume:.3f}, '
            f'lookahead={self.lookahead_volume:.3f}), '
            f'window={self.filter_window_sec:.1f} s, hold={self.threshold_hold_sec:.1f} s')

    def threshold(self) -> float:
        """true にするしきい値 V_target - lookahead_volume."""
        return self.target_volume - self.lookahead_volume

    def ensure_flag_exists(self) -> None:
        """レコードにフラグが無ければfalseで追加する(既存の値は変更しない)."""
        try:
            if self.collection.find_one(self.query) is None:
                self.get_logger().warn(f'No matching record found: {self.record_name}')
                return
            result = self.collection.update_one(
                {**self.query, self.flag_name: {'$exists': False}},
                {'$set': {self.flag_name: False}})
            if result.modified_count > 0:
                self.get_logger().info(f'Added {self.record_name}.{self.flag_name} = False')
        except Exception as e:
            self.get_logger().error(f'Failed to ensure {self.record_name}.{self.flag_name}: {e}')

    def update_target_volume(self, now: float) -> None:
        """ダンプのレコードから TargetSoilVolume を読む. 無い場合は load_threshold を使う."""
        if self.target_read_at is not None and now - self.target_read_at < self.target_reload_sec:
            return
        self.target_read_at = now
        try:
            parameter_info = self.collection.find_one(self.query, {self.target_volume_field: 1})
        except Exception as e:
            self.get_logger().error(f'Failed to read {self.record_name}.{self.target_volume_field}: {e}')
            return
        value = parameter_info.get(self.target_volume_field) if parameter_info else None
        if isinstance(value, (int, float)) and not isinstance(value, bool):
            changed = self.target_from_record is not True or float(value) != self.target_volume
            self.target_volume = float(value)
            self.target_from_record = True
            if changed:
                self.get_logger().info(
                    f'Target volume: {self.record_name}.{self.target_volume_field} = {self.target_volume:.3f} m^3 '
                    f'(threshold {self.threshold():.3f} m^3)')
        else:
            if self.target_from_record is not False:   # 初回、または読めていた値が無くなったときだけ警告
                self.get_logger().warn(
                    f'{self.record_name}.{self.target_volume_field} is not found. '
                    f'Using load_threshold = {self.load_threshold:.3f} m^3.')
            self.target_volume = self.load_threshold
            self.target_from_record = False

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

        self.update_target_volume(now)
        self.judge_threshold(filtered_volume, now)
        self.write_flag(self.load_completed, filtered_volume)

    def judge_threshold(self, filtered_volume: float, now: float) -> None:
        threshold = self.threshold()
        if self.load_completed:
            if filtered_volume < threshold - self.threshold_hysteresis:
                self.load_completed = False
                self.above_since = None
                self.get_logger().info(
                    f'Load volume dropped below {threshold - self.threshold_hysteresis:.3f} m^3. '
                    f'Ready for next loading.')
            return

        if filtered_volume < threshold:
            self.above_since = None
            return

        if self.above_since is None:
            self.above_since = now
        if now - self.above_since >= self.threshold_hold_sec:
            self.load_completed = True
            self.get_logger().info(
                f'Load volume {filtered_volume:.3f} m^3 reached threshold {threshold:.3f} m^3 '
                f'(target {self.target_volume:.3f} m^3).')

    def write_flag(self, value: bool, filtered_volume: float) -> None:
        """フラグの値が変わったときだけ書き込む."""
        if self.written_flag == value:
            return
        try:
            result = self.collection.update_one(self.query, {'$set': {self.flag_name: value}})
        except Exception as e:
            self.get_logger().error(f'Failed to update {self.record_name}.{self.flag_name}: {e}')
            return
        if result.matched_count == 0:
            self.get_logger().warn(f'No matching record found: {self.record_name}')
            return
        self.get_logger().info(
            f'Updated {self.record_name}.{self.flag_name} -> {value} (volume {filtered_volume:.3f} m^3)')
        self.written_flag = value


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
