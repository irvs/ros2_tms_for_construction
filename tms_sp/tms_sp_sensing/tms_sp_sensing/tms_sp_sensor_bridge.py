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

"""トピックとデータベースの値をそのまま受け渡すセンサブリッジ.

判断や加工はしない. 対応関係は YAML(config_file)で与える.

topic_to_db : トピックの data を、レコードのフィールドへそのまま書き込む.
    - 値が変わったときだけ書く(最初に受け取ったときは必ず1回書く).
    - timeout_sec を与えた場合、その時間トピックが届かなければ timeout_value を書き、警告を出す.
      (起動してから一度も届かない場合も同じ.) 定期的に出されるトピックにだけ設定する.
db_to_topic : レコードのフィールドの値を、トピックの data としてそのまま出す.
    - 値が変わったらすぐ出し、変わらなくても republish_sec ごとに出し直す.
    - QoS は reliable + transient_local(後から起動したノードも最新の値を受け取れる).

扱う型は std_msgs の data を1つ持つ型(Bool, Int32, Int64, Float32, Float64, String など).
人が手でフラグを変えたいときは、該当するトピックに ros2 topic pub で値を出せば、そのまま書き込まれる.
"""

import os
from typing import Any, Optional

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from rosidl_runtime_py.utilities import get_message
from pymongo import MongoClient

DEFAULT_CONFIG = os.path.join(
    get_package_share_directory('tms_sp_sensing'), 'config', 'excavation_loading_sensor_bridge.yaml')

INT_TYPES = ('int8', 'int16', 'int32', 'int64', 'uint8', 'uint16', 'uint32', 'uint64')
FLOAT_TYPES = ('float', 'double')


def convert(value: Any, field_type: str) -> Any:
    """データベースの値を、メッセージの data の型に合わせる."""
    if field_type == 'boolean':
        return bool(value)
    if field_type in INT_TYPES:
        return int(value)
    if field_type in FLOAT_TYPES:
        return float(value)
    if field_type == 'string':
        return str(value)
    raise ValueError(f'Unsupported data type: {field_type}')


class TopicToDb:
    """トピック → データベースの対応 1 つ分."""

    def __init__(self, node: 'TmsSpSensorBridge', conf: dict):
        self.node = node
        self.topic = conf['topic']
        self.record_name = conf['record_name']
        self.field = conf['field']
        self.timeout_sec: Optional[float] = conf.get('timeout_sec')
        self.timeout_value = conf.get('timeout_value')
        self.last_written: Any = None
        self.has_written = False
        self.last_received = node.now()
        self.timed_out = False
        msg_type = get_message(conf['type'])
        node.create_subscription(msg_type, self.topic, self.callback, 10)

    def callback(self, msg) -> None:
        self.last_received = self.node.now()
        if self.timed_out:
            self.timed_out = False
            self.node.get_logger().info(f'{self.topic} is received again.')
        value = msg.data
        if isinstance(value, float):
            value = float(value)
        self.write(value)

    def check_timeout(self, now: float) -> None:
        if self.timeout_sec is None or self.timed_out:
            return
        if now - self.last_received >= self.timeout_sec:
            self.timed_out = True
            self.node.get_logger().warn(
                f'{self.topic} has not been received for {self.timeout_sec:.1f} s. '
                f'Writing {self.record_name}.{self.field} = {self.timeout_value!r}.')
            self.write(self.timeout_value)

    def write(self, value: Any) -> None:
        if self.has_written and value == self.last_written and type(value) is type(self.last_written):
            return
        if self.node.write_field(self.record_name, self.field, value):
            self.last_written = value
            self.has_written = True


class DbToTopic:
    """データベース → トピックの対応 1 つ分."""

    QOS = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
                     durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)

    def __init__(self, node: 'TmsSpSensorBridge', conf: dict):
        self.node = node
        self.topic = conf['topic']
        self.record_name = conf['record_name']
        self.field = conf['field']
        self.republish_sec: float = conf.get('republish_sec', 1.0)
        self.msg_type = get_message(conf['type'])
        self.data_type = self.msg_type.get_fields_and_field_types()['data']
        self.publisher = node.create_publisher(self.msg_type, self.topic, self.QOS)
        self.last_value: Any = None
        self.last_published: Optional[float] = None
        self.missing = False

    def poll(self, now: float) -> None:
        value = self.node.read_field(self.record_name, self.field)
        if value is None:
            if not self.missing:
                self.missing = True
                self.node.get_logger().warn(f'{self.record_name}.{self.field} is not found. {self.topic} is not published.')
            return
        self.missing = False
        try:
            value = convert(value, self.data_type)
        except (TypeError, ValueError) as e:
            self.node.get_logger().error(f'Cannot publish {self.record_name}.{self.field}={value!r} to {self.topic}: {e}')
            return
        changed = value != self.last_value
        if changed or self.last_published is None or now - self.last_published >= self.republish_sec:
            msg = self.msg_type()
            msg.data = value
            self.publisher.publish(msg)
            self.last_published = now
            if changed:
                self.node.get_logger().info(f'Publish {self.topic} = {value!r} ({self.record_name}.{self.field})')
                self.last_value = value


class TmsSpSensorBridge(Node):
    def __init__(self):
        super().__init__('tms_sp_sensor_bridge')

        self.declare_parameter('config_file', DEFAULT_CONFIG)
        config_file = self.get_parameter('config_file').get_parameter_value().string_value
        with open(config_file) as f:
            config = yaml.safe_load(f)

        mongodb = config.get('mongodb', {})
        self.client = MongoClient(mongodb.get('ip', '127.0.0.1'), mongodb.get('port', 27017))
        self.collection = self.client[mongodb.get('database', 'rostmsdb')]['parameter']

        self.topic_to_db = [TopicToDb(self, c) for c in config.get('topic_to_db') or []]
        self.db_to_topic = [DbToTopic(self, c) for c in config.get('db_to_topic') or []]

        self.timer = self.create_timer(config.get('poll_period_sec', 0.1), self.poll)
        self.get_logger().info(
            f'tms_sp_sensor_bridge started: config={config_file}, '
            f'topic_to_db={[m.topic for m in self.topic_to_db]}, db_to_topic={[m.topic for m in self.db_to_topic]}')

    def now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def poll(self) -> None:
        now = self.now()
        for m in self.topic_to_db:
            m.check_timeout(now)
        for m in self.db_to_topic:
            m.poll(now)

    def read_field(self, record_name: str, field: str) -> Any:
        try:
            doc = self.collection.find_one({'record_name': record_name}, {field: 1})
        except Exception as e:
            self.get_logger().error(f'Failed to read {record_name}.{field}: {e}')
            return None
        return doc.get(field) if doc else None

    def write_field(self, record_name: str, field: str, value: Any) -> bool:
        try:
            result = self.collection.update_one({'record_name': record_name}, {'$set': {field: value}})
        except Exception as e:
            self.get_logger().error(f'Failed to update {record_name}.{field}: {e}')
            return False
        if result.matched_count == 0:
            self.get_logger().warn(f'No matching record found: {record_name}')
            return False
        self.get_logger().info(f'Updated {record_name}.{field} -> {value!r}')
        return True


def main(args=None):
    rclpy.init(args=args)
    node = TmsSpSensorBridge()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
