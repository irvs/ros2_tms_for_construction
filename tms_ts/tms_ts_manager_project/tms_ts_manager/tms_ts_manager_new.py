import rclpy
import rclpy.qos
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from tms_msg_ts.action import TsReq, TsDoTask, TsDoSubtask
from tms_msg_db.srv import TmsdbGetData

import json
import re
from pprint import pprint


class TaskNode(Node):
    """タスクを実行するノード
    """
    _count = 0
    def __init__(self, task_tree):
        """コンストラクタ
        """
        self.cb_group = ReentrantCallbackGroup()
        self.name = f'tasknode_{TaskNode._count}'
        self.task_tree = task_tree
        self.child_tasknodes = {}
        self.goal_handle_clients = {}
        self.subtask_goalhandle = None
        super().__init__(self.name)
        TaskNode._count += 1

        self.action_server = ActionServer(self, TsDoTask, self.name, self.execute_callback,
        cancel_callback=self.cancel_callback,
        callback_group=self.cb_group)
    
    def destroy_node(self):
        global executor
        self.get_logger().warning('destroy')
        executor.remove_node(self)
        for child in self.child_tasknodes.values():
            try:
                child.destroy_node()
            except rclpy.handle.InvalidHandle as e:
                print(f"DestroyNode error: {e}")
        
        super().destroy_node()  # 注意
        # 2020-01-30現在，このsuper().destroy_node()を実行すればrclpy.handle.InvalidHandleが起こってしまうが
        # rqt_graphやros2 node listなどの表示からこのノードが消える

    def cancel_callback(self, goal_handle):
        self.get_logger().warning(f"Canceled, {goal_handle.status}")
        self._cancel_timer = self.create_timer(0.0, self._cancel, callback_group=self.cb_group)
        if not goal_handle.is_cancel_requested:
            return CancelResponse.ACCEPT
        else:
            return CancelResponse.REJECT

    async def _cancel(self):
        self._cancel_timer.cancel()
        if self.subtask_goalhandle is not None:
            future_sub_gh = await self.subtask_goalhandle.cancel_goal_async()
        for goal_handle_client in  self.goal_handle_clients.values():
            future = await goal_handle_client.cancel_goal_async()

    async def execute_callback(self, goal_handle):
        result = TsDoTask.Result()
        self.get_logger().warning(f"{self.task_tree}")
        result.message = await self.execute(goal_handle, self.task_tree)
        if result.message == "Success":
            goal_handle.succeed()
        else:
            goal_handle.abort()
        return result
    
    async def execute(self, goal_handle, task_tree):
        if task_tree == []:
            return "Success"
        tree_type = task_tree[0]
        msg = ""
        if goal_handle.is_cancel_requested:
            return "Canceled"
        elif tree_type == "serial":
            msg = await self.serial(goal_handle, task_tree)
        elif tree_type == "parallel":
            msg = await self.parallel(goal_handle, task_tree)
        elif tree_type == "subtask":
            msg = await self.subtask(goal_handle, task_tree)
        else:
            self.get_logger().warning(f"Error type of subtask tree: {tree_type}")

        return msg

    async def serial(self, goal_handle, task_tree):
        #print(f'serial: {task_tree[1]} {task_tree[2]}')
        print("serial")
        msg1 = await self.execute(goal_handle, task_tree[1])
        if goal_handle.is_cancel_requested:
            return "Canceled"
        if msg1 != "Success":
            return msg1
        msg2 = await self.execute(goal_handle, task_tree[2])

        if msg2 == "Success":
            return "Success"
        else:
            return msg2

    async def parallel(self, goal_handle, task_tree):
        #print(f'parallel: {task_tree[1]} {task_tree[2]}')
        print("parallel")
        goal_handle_client, child_tasknode = await self.create_tasknode(task_tree[1])
        self.child_tasknodes[id(child_tasknode)] = child_tasknode
        self.goal_handle_clients[goal_handle_client.goal_id.uuid.tostring()] = goal_handle_client
        print(self.goal_handle_clients)
        msg2 = await self.execute(goal_handle, task_tree[2])
        future_result = await goal_handle_client.get_result_async()
        msg1 = future_result.result.message
        self.goal_handle_clients[goal_handle_client.goal_id.uuid.tostring()] = None
        del self.goal_handle_clients[goal_handle_client.goal_id.uuid.tostring()]
        self.child_tasknodes[id(child_tasknode)] = None
        del self.child_tasknodes[id(child_tasknode)]
        child_tasknode.destroy_node()
        if msg1 == "Success" and msg2 == "Success":
            return "Success"
        elif msg1 != "Success":
            return msg1
        else:
            return msg2

    async def subtask(self, goal_handle, task_tree):
        # print("subtask")
        command = task_tree[1]
        self.get_logger().warn(command[0])
        self.subtask_client = ActionClient(self, TsDoSubtask, "subtask_node_" + str(command[0]), callback_group=ReentrantCallbackGroup())
        if not self.subtask_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('No action server available')
            goal_handle.abort()
            result = TsReq.Result()
            result.message = "Abort"
            return result
        goal_msg = TsDoSubtask.Goal()
        if len(command) >= 2:
            goal_msg.arg_json = command[1]
        else:
            goal_msg.arg_json = "{}"

        print(f'subtask: subtask_node_{command[0]} args:{goal_msg.arg_json}')
        self.subtask_goalhandle = await self.subtask_client.send_goal_async(goal_msg)
        if not self.subtask_goalhandle.accepted:
            self.get_logger().info("goal rejected")
            return "Abort"
        
        future_result = await self.subtask_goalhandle.get_result_async()
        self.get_logger().info(f"returned {future_result.result.message}")
        return future_result.result.message
    
    async def create_tasknode(self, task_tree):
        global executor
        # generate task_node
        child_tasknode = TaskNode(task_tree)
        # self.child_tasknode.append(self.childtask_node)
        executor.add_node(child_tasknode)  # added last added task
        ## execute
        client = ActionClient(self, TsDoTask, child_tasknode.name, callback_group=ReentrantCallbackGroup())
        if not client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('No action server available')
            goal_handle.abort()
            result = TsDoTask.Result()
            result.message = "Abort"
            return result
        goal = TsDoTask.Goal()
        goal_handle_client = await client.send_goal_async(goal)
        if not goal_handle_client.accepted:
            self.get_logger().info("goal reject")
            goal_handle.abort()
            result = TsDoTask.Result()
            result.message = "Goal Reject"
            return result
        self.get_logger().info("goal accept")
        return goal_handle_client, child_tasknode



class TaskSchedulerManager(Node):
    """TaskNodeを管理するマネージャノード
    """

    def __init__(self):
        """コンストラクタ
        """
        self.cb_group = ReentrantCallbackGroup()

        self.goal_handle_clients = {}
        super().__init__("task_scheduler_manager")
        self.get_logger().info('task_scheduler_manager')
        self.action_server = ActionServer(
            self, TsReq, "tms_ts_master", self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback, callback_group=self.cb_group
        )
    
    def destroy_node(self):
        """デストラクタ
        """
        self.action_server.destroy()
        super().destroy_node()
    
    def goal_callback(self, goal_handle):
        """タスク実行要求受付
        """
        self.get_logger().info('Received Task Request')
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        """タスクのキャンセル
        """
        self.get_logger().info('Received Cancel task')
        # self._cancel_timer = self.create_timer(0.0, self._cancel, callback_group=self.cb_group)
        client = None
        try:
            client = self.goal_handle_clients[goal_handle.goal_id.uuid.tostring()]
        except IndexError as e:
            self.get_logger().error(f"{str(e)}")
            return CancelResponse.ACCEPT
        if client is not None:
            client.cancel_goal_async()
        
        return CancelResponse.ACCEPT
    
    # async def _cancel(self):
    #     self._cancel_timer.cancel()
    #     await self.goal_handle_client.cancel_goal_async()

    async def execute_callback(self, goal_handle):
        """タスク処理
        """
        global executor
        self.goal_handle = goal_handle

        task_id = goal_handle.request.task_id
        arg_json = goal_handle.request.arg_json
        if arg_json != '':  # 引数が存在するとき
            self.arg_data = json.loads(arg_json)
        else:
            self.arg_data = {}
        self.get_logger().info(f'Execute task{task_id}, args:{self.arg_data}')
        
        task_tree = await self.convert_task_to_subtasks(task_id, self.arg_data)
        pprint(task_tree)

        # generate task_node
        task_node = TaskNode(task_tree)
        executor.add_node(task_node)  # added last added task
        ## execute
        client = ActionClient(self, TsDoTask, task_node.name, callback_group=ReentrantCallbackGroup())
        if not client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('No action server available')
            goal_handle.abort()
            result = TsReq.Result()
            result.message = "Abort"
            return result
        goal = TsDoTask.Goal()
        goal_handle_client = await client.send_goal_async(goal)

        self.goal_handle_clients[goal_handle.goal_id.uuid.tostring()] = goal_handle_client

        if not goal_handle_client.accepted:
            self.get_logger().info("goal reject")
            goal_handle.abort()
            result = TsReq.Result()
            result.message = "Goal Reject"
            return result
        self.get_logger().info("goal accept")
        self.future_result = await goal_handle_client.get_result_async()

        self.goal_handle_clients[goal_handle.goal_id.uuid.tostring()] = None
        del self.goal_handle_clients[goal_handle.goal_id.uuid.tostring()]
        # 結果を返す
        msg = self.future_result.result.message
        if msg == "Success":
            goal_handle.succeed()
            result = TsReq.Result()
            result.message = "Success"
        else:
            goal_handle.abort()
            result = TsReq.Result()
            result.message = msg
        # result.message = "Success"
        task_node.destroy_node()
        return result
        

    async def convert_task_to_subtasks(self, task_id, arg_data):
        def func(m):
            arg_str = m.groups()[0]
            args = arg_str.split('.')
            answer = arg_data.copy()
            for arg in args:
                answer = answer.get(arg, {})
            if answer == {}:
                self._is_valid_subtask_replace = False
                return '"ARGUMENT ERROR"'
            else:
                return str(answer)

        subtask_list = []
        tmsdb_data = await self.call_dbreader(task_id)  # this is list
        subtask_str = tmsdb_data[0].etcdata
        self.get_logger().info(f"find task '{tmsdb_data[0].name}'")
        self.get_logger().info(f"read task '{subtask_str}'")
        subtask_raw_list = re.findall(r'[0-9]+\$\{.*?\}|[0-9]+|\+|\|', subtask_str)

        self._is_valid_subtask_replace = True
        for subtask_raw in subtask_raw_list:
            subtask = subtask_raw.split("$")
            generated_subtask = []
            for elem in subtask:
                elem = re.sub(r"\((.*?)\)",\
                    func,\
                    elem)
                generated_subtask.append(elem)
            subtask_list.append(generated_subtask)

        if self._is_valid_subtask_replace == False:
            self.get_logger().warning('task argument error!')
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
        print(syntax_tree)
        if len(syntax_tree) != 1:
            self.get_logger().warning('subtask syntax error!')
            return []

        return syntax_tree[0]
    
    async def call_dbreader(self, id):
        """[tms_db_reader] DBからデータを読み取る
        """
        self.cli_dbreader = self.create_client(TmsdbGetData, 'tms_db_reader', callback_group=self.cb_group)
        while not self.cli_dbreader.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service "tms_db_reader" not available, waiting again...')
        req = TmsdbGetData.Request()
        req.tmsdb.id = id + 100000  # TODO: why add 100000 ?
        self.future_dbreader  = self.cli_dbreader.call_async(req)

        await self.future_dbreader

        if self.future_dbreader.result() is not None:
            res = self.future_dbreader.result().tmsdb
            return res
        else:
            self.get_logger().info('Service "tms_db_reader" call failed %r' % (self.future_dbreader.exception(),))

    async def read_name(self, id):
        """[tms_db_reader] DBから名前を読み取る
        """
        tmsdb = await self.call_dbreader(int(id))
        return tmsdb[0].name
    

def main(args=None):
    global executor
    rclpy.init(args=args)
    try:
        executor = MultiThreadedExecutor()
        manager_node = TaskSchedulerManager()
        executor.add_node(manager_node)
        try:
            executor.spin()
        finally:
            executor.shutdown()
    finally:
        rclpy.shutdown()

if __name__=='__main__':
    main()