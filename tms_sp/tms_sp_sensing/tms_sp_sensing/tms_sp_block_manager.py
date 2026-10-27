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

"""掘削積込(excavation_loading)のブロック管理ノード.

バックホウの BT から NEXT_BLOCK_FLG=true で依頼を受けたら、ブロック番号を1つ進め、
そのブロックでの行き先を実行管理用のレコード(SAMPLE_BLACKBOARD_zx200)に書き込む.
次のブロックへ進むかどうかは判断しない(BT に頼まれたときに数えて書くだけ).

    current_block_index : i = 現在の値 + 1 (初期化時の -1 からは 0)
    excavate_position   : excavate_position_{i+1}  (バックホウの行き先のレコード名)
    loading_position    : loading_position_{i+1}   (ダンプの行き先のレコード名)
    LAST_BLOCK_FLG      : i がブロック定義(excavation_loading_params の block_center)の最後か
    NEXT_BLOCK_FLG      : false に戻す(処理完了の合図)

これらは1回の更新でまとめて書き込む. MongoDB では1つのドキュメントへの更新は一括で反映されるので、
BT が NEXT_BLOCK_FLG=false を読んだ時点で、行き先は必ず新しいブロックのものになっている.

MongoDB がレプリカセットではなく change stream を使えないため、NEXT_BLOCK_FLG は poll_period_sec ごとに読む.
"""

from typing import Optional

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from pymongo import MongoClient


class TmsSpBlockManager(Node):
    def __init__(self):
        super().__init__('tms_sp_block_manager')

        self.declare_parameter('state_record_name', 'SAMPLE_BLACKBOARD_zx200')         # 実行管理用のレコード
        self.declare_parameter('params_record_name', 'excavation_loading_params')      # tms_if_input の出力
        self.declare_parameter('block_list_field', 'block_center')                    # ブロックの数を数えるフィールド
        self.declare_parameter('excavate_position_prefix', 'excavate_position_')
        self.declare_parameter('loading_position_prefix', 'loading_position_')
        self.declare_parameter('poll_period_sec', 0.1)
        self.declare_parameter('mongodb_ip', '127.0.0.1')
        self.declare_parameter('mongodb_port', 27017)
        self.declare_parameter('mongodb_database', 'rostmsdb')

        self.state_record_name = self.get_parameter('state_record_name').get_parameter_value().string_value
        self.params_record_name = self.get_parameter('params_record_name').get_parameter_value().string_value
        self.block_list_field = self.get_parameter('block_list_field').get_parameter_value().string_value
        self.excavate_position_prefix = \
            self.get_parameter('excavate_position_prefix').get_parameter_value().string_value
        self.loading_position_prefix = self.get_parameter('loading_position_prefix').get_parameter_value().string_value
        poll_period_sec = self.get_parameter('poll_period_sec').get_parameter_value().double_value
        mongodb_ip = self.get_parameter('mongodb_ip').get_parameter_value().string_value
        mongodb_port = self.get_parameter('mongodb_port').get_parameter_value().integer_value
        mongodb_database = self.get_parameter('mongodb_database').get_parameter_value().string_value

        self.client = MongoClient(mongodb_ip, mongodb_port)
        self.collection = self.client[mongodb_database]['parameter']
        self.state_query = {'record_name': self.state_record_name}
        self.last_error: Optional[str] = None   # 同じエラーを毎周期出さないため

        self.timer = self.create_timer(poll_period_sec, self.poll)
        self.get_logger().info(
            f'tms_sp_block_manager started: state={self.state_record_name}, params={self.params_record_name}, '
            f'blocks={self.block_count()}, poll={poll_period_sec:.2f} s')

    def block_count(self) -> Optional[int]:
        """ブロック定義からブロックの数を数える."""
        params = self.collection.find_one({'record_name': self.params_record_name}, {self.block_list_field: 1})
        if params is None or not isinstance(params.get(self.block_list_field), list):
            return None
        return len(params[self.block_list_field])

    def error_once(self, message: str) -> None:
        if message != self.last_error:
            self.get_logger().error(message)
            self.last_error = message

    def poll(self) -> None:
        try:
            state = self.collection.find_one(self.state_query)
        except Exception as e:
            self.error_once(f'Failed to read {self.state_record_name}: {e}')
            return
        if state is None:
            self.error_once(f'No matching record found: {self.state_record_name}')
            return
        if state.get('NEXT_BLOCK_FLG') is not True:
            return
        self.advance(state)

    def advance(self, state: dict) -> None:
        """ブロック番号を1つ進め、行き先を書き込んでから NEXT_BLOCK_FLG を false に戻す."""
        current = state.get('current_block_index')
        if not isinstance(current, (int, float)) or isinstance(current, bool):
            self.error_once(f'{self.state_record_name}.current_block_index is invalid: {current!r}')
            return
        count = self.block_count()
        if count is None:
            self.error_once(f'{self.params_record_name}.{self.block_list_field} is not found.')
            return

        index = int(current) + 1
        if index >= count:
            # 最後のブロックの後は進めない. NEXT_BLOCK_FLG を true のまま残し、BT が先へ進まないようにする.
            self.error_once(
                f'Requested block {index} but there are only {count} blocks. NEXT_BLOCK_FLG is left true.')
            return

        excavate_position = f'{self.excavate_position_prefix}{index + 1}'
        loading_position = f'{self.loading_position_prefix}{index + 1}'
        for record in (excavate_position, loading_position):
            if self.collection.find_one({'record_name': record}, {'_id': 1}) is None:
                self.get_logger().warn(f'Position record not found: {record}')

        update = {
            'current_block_index': index,
            'excavate_position': excavate_position,
            'loading_position': loading_position,
            'LAST_BLOCK_FLG': index == count - 1,
            'NEXT_BLOCK_FLG': False,
        }
        try:
            # NEXT_BLOCK_FLG=true のときだけ更新する(他から false に戻されていたら何もしない)
            result = self.collection.update_one({**self.state_query, 'NEXT_BLOCK_FLG': True}, {'$set': update})
        except Exception as e:
            self.error_once(f'Failed to update {self.state_record_name}: {e}')
            return
        if result.modified_count == 0:
            return
        self.last_error = None
        self.get_logger().info(
            f'Block {int(current)} -> {index} / {count - 1}: excavate_position={excavate_position}, '
            f'loading_position={loading_position}, LAST_BLOCK_FLG={update["LAST_BLOCK_FLG"]}')


def main(args=None):
    rclpy.init(args=args)
    node = TmsSpBlockManager()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
