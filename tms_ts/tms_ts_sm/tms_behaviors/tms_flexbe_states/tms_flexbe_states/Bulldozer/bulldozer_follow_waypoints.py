#!/usr/bin/env python3
import rclpy
from rclpy.action import ActionClient

from flexbe_core import EventState, Logger
from tms_msg_ts.action import SimpleConnectorHFSMBTH


class BulldozerFollowWaypoints(EventState):
    '''
    Navigate the crawler dummp along the waypoints.

    -- model_name: Model name to control from this such as "ic120".
    -- task_id (int): ID of the task to send

    <= received: Action succeeded
    <= aborted: Action aborted by server
    <= no_connection: Failed to connect to action server
    <= data_error: Communication or result error
    '''

    def __init__(self, model_name, record_name):
        super().__init__(outcomes=['received', 'aborted', 'no_connection', 'data_error'],
                         output_keys=['data'])
        self._task_id = 1 # Sample。DBに正規・非正規Subtask Nodes用のコレクションの2種を用意し、正規のコレクションにprimitivesを格納。タスクIDをそれに合わせて修正
        self._model_name = model_name
        self._record_name = record_name 
        self._action_topic = model_name + '/SimpleConnectionHFSMBTH'
        self._node = rclpy.create_node('bulldozer_follow_waypoints_client')
        self._client = ActionClient(self._node, SimpleConnectorHFSMBTH, self._action_topic)
        self._goal_handle = None
        self._connected = True
        self._received = False
        self._result = None


    def on_enter(self, userdata):
        self._received = False
        self._result = None
        if not self._client.wait_for_server(timeout_sec=3.0):
            Logger.logwarn(f'[{self._action_topic}] Action server not available')
            self._connected = False
            return
        goal_msg = SimpleConnectorHFSMBTH.Goal()
        goal_msg.task_id = self._task_id
        goal_msg.model_name = self._model_name
        goal_msg.record_name = self._record_name
        Logger.loginfo(f'Sending goal: task_id={goal_msg.task_id}')
        Logger.loginfo(f'Sending goal: model_name={goal_msg.model_name}, record_name={goal_msg.record_name}')
        send_goal_future = self._client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self._goal_response_callback)


    def _goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            Logger.logwarn('Goal rejected by server.')
            self._connected = False
            return
        Logger.loginfo('Goal accepted, waiting for result...')
        self._goal_handle = goal_handle
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self._get_result_callback)


    def _get_result_callback(self, future):
        result = future.result().result
        self._result = result
        self._received = True
        if result.result_code == 0:
            Logger.loginfo('Action succeeded (RESULT_OK)')
        elif result.result_code == 1:
            Logger.logwarn('Action failed (RESULT_FAILED)')
        elif result.result_code == 2:
            Logger.logwarn('Action aborted (RESULT_ABORTED)')
        else:
            Logger.logwarn(f'Unknown result code: {result.result_code}')


    def execute(self, userdata):
        rclpy.spin_once(self._node, timeout_sec=0.1)
        if not self._connected:
            return 'no_connection'
        if self._received and self._result is not None:
            if self._result.result_code == 0:
                userdata.data = self._result
                return 'received'
            elif self._result.result_code == 2:
                return 'aborted'
            else:
                return 'data_error'
        return None


    def on_exit(self, userdata):
        if self._node is not None:
            self._node.destroy_node()
