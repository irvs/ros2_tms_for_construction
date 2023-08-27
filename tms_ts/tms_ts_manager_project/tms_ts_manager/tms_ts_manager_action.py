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

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from tms_msg_ts.action import TsDoSubtask, TsDoTask, TsReq
import json
import re
from tms_msg_db.srv import TmsdbGetTask
import time

class TaskNode(Node):   #タスクを実行するノード(受け取ったサブタスクツリーによって，再帰的にTaskNodeを生成する)
    _task_num = 0
    tasks = []
    def __init__(self, task_tree, name_prefix=""):
        self.node_type = task_tree[0]
        self.task_tree = task_tree
        if name_prefix != "":
            name_prefix += "_"
        self.name = f"{self.node_type}{TaskNode._task_num}"
        TaskNode._task_num += 1
        TaskNode.tasks.append(self)
        self.cb_group = ReentrantCallbackGroup()
        self.child_task_nodes = []
        self.child_clients = []
        self.goal_handles = []  # Goal Handle
    
        super().__init__(self.name)
        self.action_server = ActionServer(
            self,
            TsDoTask,
            f"{self.name}",
            self.execute_callback,
            callback_group=self.cb_group,
            cancel_callback=self.cancel_callback)
        
        self.get_logger().info('Instantiate')

    async def execute_callback(self, goal_handle):
        result = TsDoTask.Result()
        if self.node_type == "serial":
            result = await self.serial(goal_handle)
        elif self.node_type == "parallel":
            result = await self.parallel(goal_handle)
        else:  # subtask
            result = await self.subtask(goal_handle)
        return result

    def cancel_callback(self, goal_handle):
        for goal_handle in self.goal_handles:
            goal_handle.cancel_goal_async()
        return CancelResponse.ACCEPT

    async def serial(self, goal_handle):   #child task 1を実行した後，child task 2を実行する
        global executor
        # child task 1   
        child_task_node1 = TaskNode(self.task_tree[1], name_prefix=self.name)
        executor.add_node(child_task_node1)
        self.child_task_nodes.append(child_task_node1)
        self.get_logger().info(f"Generate {child_task_node1.name}")

        ## execute
        self.client = ActionClient(self, TsDoTask, child_task_node1.name, callback_group=ReentrantCallbackGroup())
        print(f"subtask1: {self.client}")
        self.child_clients.append(self.client)
        self.client.wait_for_server()
        goal = TsDoTask.Goal()
        self.get_logger().info(f"start {child_task_node1.name} send_goal_async")
        self.goal_handle_client = await self.client.send_goal_async(goal)
        self.get_logger().info(f"end {child_task_node1.name} send_goal_async")
        self.goal_handles.append(self.goal_handle_client)
        if not self.goal_handle_client.accepted:
            goal_handle.abort()
            result = TsDoTask.Result()
            result.message = "Goal Reject"
            return result
        self.get_logger().info(f"start {child_task_node1.name} get_result_async")
        self.future_result = await self.goal_handle_client.get_result_async()
        self.get_logger().info(f"end {child_task_node1.name} get_result_async")
        if not self.future_result.result.message == "Success":
            goal_handle.abort()
            result = TsDoTask.Result()
            result.message = "Abort"
            return result
        self.get_logger().info(f"Execute {child_task_node1.name}")
        self.goal_handles.pop()
        self.get_logger().info(f"Destroy {child_task_node1.name}")
        
        # cancelされていないか確認する
        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
            result = TsDoTask.Result()
            result.message = "Canceled"
            self.get_logger().info('goal canceled')
            return result
        
        # child task 2
        child_task_node2 = TaskNode(self.task_tree[2], name_prefix=self.name)
        executor.add_node(child_task_node2)
        self.child_task_nodes.append(child_task_node2)
        self.get_logger().info(f"Generate {child_task_node2.name}")

        ## execute
        self.client = ActionClient(self, TsDoTask, child_task_node2.name, callback_group=ReentrantCallbackGroup())
        print(f"subtask2: {self.client}")
        self.child_clients.append(self.client)
        self.client.wait_for_server()
        goal = TsDoTask.Goal()
        self.get_logger().info(f"start {child_task_node2.name} send_goal_async")
        self.goal_handle_client = await self.client.send_goal_async(goal)
        self.get_logger().info(f"end {child_task_node2.name} send_goal_async")
        self.goal_handles.append(self.goal_handle_client)
        if not self.goal_handle_client.accepted:
            self.get_logger().info("subtask2 goal reject")
            goal_handle.abort()
            result = TsDoTask.Result()
            result.message = "Goal Reject"
            return result
        self.get_logger().info("subtask2 goal accept")
        self.get_logger().info(f"start {child_task_node2.name} get_result_async")
        self.future_result = await self.goal_handle_client.get_result_async()
        self.get_logger().info(f"end {child_task_node2.name} get_result_async")
        if not self.future_result.result.message == "Success":
            goal_handle.abort()
            result = TsDoTask.Result()
            result.message = "Abort"
            return result
        self.get_logger().info(f"Execute {child_task_node2.name}")
        self.get_logger().info(f"Destroy {child_task_node2.name}")
        goal_handle.succeed()
        result = TsDoTask.Result()
        result.message = "Success"
        return result

    async def parallel(self, goal_handle):
        """child task 1とchild task 2を並列に実行する
        """
        global executor
        # generate
        child_task_node1 = TaskNode(self.task_tree[1], name_prefix=self.name)
        self.child_task_nodes.append(child_task_node1)
        executor.add_node(child_task_node1)
        child_task_node2 = TaskNode(self.task_tree[2], name_prefix=self.name)
        self.child_task_nodes.append(child_task_node2)
        executor.add_node(child_task_node2)

        time.sleep(1.0)

        # send goal
        self.client1 = ActionClient(self, TsDoTask, child_task_node1.name, callback_group=ReentrantCallbackGroup())
        self.child_clients.append(self.client1)
        self.client1.wait_for_server()
        goal = TsDoTask.Goal()
        self.goal_handle_future1 = self.client1.send_goal_async(goal)
        self.client2 = ActionClient(self, TsDoTask, child_task_node2.name, callback_group=ReentrantCallbackGroup())
        self.child_clients.append(self.client2)
        self.client2.wait_for_server()
        goal = TsDoTask.Goal()
        self.goal_handle_future1 = self.client2.send_goal_async(goal)

        # await send_goal_async
        self.goal_handle_client1 = await self.goal_handle_future1
        self.goal_handles.append(self.goal_handle_client1)
        self.goal_handle_client2 = await self.goal_handle_future2
        self.goal_handles.append(self.goal_handle_client2)
        if not self.goal_handle_client1.accepted:
            self.get_logger().info("subtask1 goal reject")
            goal_handle.abort()
            result = TsDoTask.Result()
            result.message = "Goal Reject"
            return result
        self.get_logger().info("subtask1 goal accept")
        if not self.goal_handle_client2.accepted:
            self.get_logger().info("subtask2 goal reject")
            goal_handle.abort()
            result = TsDoTask.Result()
            result.message = "Goal Reject"
            return result
        self.get_logger().info("subtask2 goal accept")
        
        # get result
        self.future_result1 = self.goal_handle_client1.get_result_async()
        self.future_result2 = self.goal_handle_client2.get_result_async()
        self.future_result = await self.future_result1
        if not self.future_result.done():
            goal_handle.abort()
            result = TsDoTask.Result()
            result.message = "Abort"
            return result
        self.future_result = await self.future_result2
        if not self.future_result.done():
            goal_handle.abort()
            result = TsDoTask.Result()
            result.message = "Abort"
            return result
        self.child_task_nodes.pop()
        self.child_clients.pop()
        self.goal_handles.pop()
        self.child_task_nodes.pop()
        self.child_clients.pop()
        self.goal_handles.pop()

        goal_handle.succeed()
        result = TsDoTask.Result()
        result.message = "Success"
        return result
    
    async def subtask(self, goal_handle):
        """サブタスクを実行する
        """
        command = self.task_tree[1]
        self.subtask_client = ActionClient(self, TsDoSubtask, "subtask_node_" + str(command[0]), callback_group=ReentrantCallbackGroup())
        self.child_clients.append(self.subtask_client)
        print(self.subtask_client)
      
        goal_msg = TsDoSubtask.Goal()
        if len(command) >= 2:
            self.get_logger().info(f'service call "subtask_node_{command[0]} args: {command[1]}"')
            goal_msg.arg_json = command[1]
            self.get_logger().info(f'command:{command[1]}')
        else:
            self.get_logger().info(f'service call "subtask_node_{command[0]}"')
            goal_msg.arg_json = "{}"

        self.goal_handle = await self.subtask_client.send_goal_async(goal_msg)
        self.goal_handles.append(self.goal_handle)
        if not self.goal_handle.accepted:
            self.get_logger().info("goal rejected")
            goal_handle.abort()
            result = TsDoTask.Result()
            result.message = "Goal Rejected"
            return result
        
        self.get_logger().info("goal accept")
        self.future_result = await self.goal_handle.get_result_async()
        result = TsDoTask.Result()
        if True:
            self.get_logger().info(self.future_result.result.message)
            goal_handle.succeed()
            result.message = "Success"
        else:
            goal_handle.abort()
            result.message = self.future_result.result.message
        self.goal_handles.pop()
        time.sleep(0.1)
        return result


class TaskSchedulerManager(Node):   #TaskNodeを管理するマネージャノード
    
    def __init__(self):
        self.task_node_list = []
        self.cb_group = ReentrantCallbackGroup()
        self.name = 'task_scheduler_manager'
        self.goal_handles = []
        super().__init__('task_scheduler_manager')

        self.action_server = ActionServer(
            self,
            TsReq,
            "tms_ts_master",
            self.execute_callback,
            callback_group=self.cb_group,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback
        )

    def destroy_node(self):
        self.action_server.destroy()
        super().destroy_node()
    

    def goal_callback(self, goal_request):   #タスク実行要求をACCEPT or REJECT
        self.get_logger().info('Received Task Request')
        return GoalResponse.ACCEPT

    def cancel_callback(self,goal_handle):   #タスク停止要求をACCEPT or REJECT
        self.get_logger().info('Received Cancel Task')
        for goal_h in self.goal_handles:
            try:
                goal_h.cancel_goal_async()
            except rclpy.handle.InvalidHandle as e:
                print(e)
        return CancelResponse.ACCEPT
    
    async def execute_callback(self, goal_handle):   #タスク処理
        global executor
        task_id = goal_handle.request.task_id
        data_str = goal_handle.request.arg_json
        if data_str != '':
            self.arg_data = json.loads(data_str)
        else:
            self.arg_data = {}

        # syntax analyze
        task_tree = await self.convert_task_to_subtasks(task_id, self.arg_data)
        if task_tree == []:   #構文木が[ ]のときにエラーを戻り値にとる部分
            goal_handle.abort()
            result = TsReq.Result()
            result.message = "Syntax Error"
            return result        
        
        # generate task_node
        task_node = TaskNode(task_tree)
        self.task_node_list.append(task_node)
        executor.add_node(self.task_node_list[len(self.task_node_list) - 1])  # added last added task

        ## execute
        client = ActionClient(self, TsDoTask, task_node.name, callback_group=ReentrantCallbackGroup())
        client.wait_for_server()
        goal = TsDoTask.Goal()
        goal_handle_client = await client.send_goal_async(goal)
        self.goal_handles.append(goal_handle_client)
        if not goal_handle_client.accepted:
            self.get_logger().info("goal reject")
            goal_handle.abort()
            result = TsReq.Result()
            result.message = "Goal Reject"
            return result
        self.get_logger().info("goal accept")
        self.future_result = await goal_handle_client.get_result_async()
        #if not self.future_result.done():
        if True:
            goal_handle.abort()
            result = TsReq.Result()
            result.message = "Abort"
            return result

        # return result
        goal_handle.succeed()
        result = TsReq.Result()
        result.message = "Success"
        task_node.destroy_node()
        return result

    async def convert_task_to_subtasks(self, task_id, arg_data):
        def func(m):
            arg_str = m.groups()[0] # m.groups()はマッチした部分を文字列として取得
            args = arg_str.split('.')
            answer = arg_data.copy()
            self.get_logger().info(f"answer: {answer}")
            for arg in args:
                answer = answer.get(arg, {})
                self.get_logger().info(f"answer: {answer}")
            if answer == {}:
                self._is_valid_subtask_replace = False
                return '"ARGUMENT ERROR"'
            else:
                return str(answer)

        subtask_list = []
        task = await self.get_task(task_id) 
        subtask_str = task['etcdata']
        subtask_raw_list = re.findall(r'[0-9]+\$\{.*?\}|[0-9]+|\+|\|', subtask_str)
        self.get_logger().info(f"subtask_raw_list: {subtask_raw_list}")

        self._is_valid_subtask_replace = True
        for subtask_raw in subtask_raw_list:
            subtask = subtask_raw.split("$")
            generated_subtask = []
            for elem in subtask:
                elem = re.sub(r"\((.*?)\)",func,elem)
                self.get_logger().info(f"elem: {elem}")
                generated_subtask.append(elem)
                self.get_logger().info(f"generated_subtask: {generated_subtask}")
            subtask_list.append(generated_subtask)

        print(f"subtask_list: {subtask_list}")
        if self._is_valid_subtask_replace == False:
            print(f"[{self.name}] >> argument error!")
            return []
        
        # 構文解析
        _stack = []
        for s in subtask_list:
            if s == ['+']:
                pre1 = _stack.pop(-1)
                pre2 = _stack.pop(-1)
                _stack.append(['serial', pre2, pre1])
            elif s == ['|']:
                pre1 = _stack.pop(-1)
                pre2 = _stack.pop(-1)
                _stack.append(['parallel', pre2, pre1])
            else:
                _stack.append(['subtask', s])
        
        syntax_tree = _stack
        print(f"[{self.name}] >> analyze task")
        print(syntax_tree)
        if len(syntax_tree) != 1:
            print(f"[{self.name}] >> syntax error!")
            return []

        return syntax_tree[0]
    
    async def get_task(self, task_id):
        # Get task object
        self.cli_dbreader = self.create_client(TmsdbGetTask, 'tms_db_reader_task', callback_group=self.cb_group)
        while not self.cli_dbreader.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service "tms_db_reader_task" not available, waiting again...')
            
        req = TmsdbGetTask.Request()
        req.type = 'default'
        req.id = task_id
        
        self.future_dbreader  = self.cli_dbreader.call_async(req)
        await self.future_dbreader
        if self.future_dbreader.result() is not None:
            res = self.future_dbreader.result().task
            return json.loads(res)
        else:
            self.get_logger().info('Service "tms_db_reader_task" call failed %r' % (self.future_dbreader.exception(),))


def main(args=None):
    global executor
    rclpy.init(args=args)
    try:
        executor = MultiThreadedExecutor(num_threads=999)
        task_scheduler_manager = TaskSchedulerManager()
        executor.add_node(task_scheduler_manager)
        try:
            executor.spin()
        finally:
            executor.shutdown()
            task_scheduler_manager.destroy_node()
    finally:
        rclpy.shutdown()

if __name__=='__main__':
    main()