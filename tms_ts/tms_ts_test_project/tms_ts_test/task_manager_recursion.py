import asyncio
from tms_msg_ts.srv import TsReq
from tms_msg_db.srv import TmsdbGetData
from tms_msg_ts.srv import TsDoTask
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
import time
import random

class Robot:

    def __init__(self, name='dummy robot'):
        self.name = name
    
    def execute(self):
        self.grasp()
        time.sleep(3.0)
        self.move()
        time.sleep(3.0)
        self.release()

    def grasp(self):
        print(f'[{self.name}] grasp')
        return 1
    
    def move(self):
        print(f'[{self.name}] move')
        return 1
    
    def release(self):
        print(f'[{self.name}] release')
        return 1


class Task:
    def __init__(self, rostime=0, task_id=0, robot_id=0, object_id=0, user_id=0, place_id=0, priority=0):
        self.rostime = rostime
        self.task_id = task_id
        self.robot_id = robot_id
        self.object_id = object_id
        self.user_id = user_id
        self.place_id = place_id
        self.priority = priority


class TaskNode(Node):
    """タスクを実行するノード
    受け取ったタスクによって、再帰的にタスクノードを作る。
    """
    _task_num = 0  # it is task count for name, not task_id 

    def __init__(self, task_tree, name_prefix=""):
        self.node_type = task_tree[0]
        self.task_tree = task_tree
        self.name = f"{name_prefix}{self.node_type}{TaskNode._task_num}_"
        TaskNode._task_num += 1
        self.cb_group = ReentrantCallbackGroup()
        self.child_task_nodes = []
        
        super().__init__(self.name)
        
        self.srv = self.create_service(TsDoTask, self.name, self.main, callback_group=self.cb_group)
    
    async def main(self, request, response):

        print(f"[{self.name}] >> init")

        if(self.node_type == "serial"):
            response.message = await self.serial()
        elif(self.node_type == "parallel"):
            response.message = await self.parallel()
        else:  # subtask
            response.message = await self.subtask()

        return response

    async def serial(self):
        global executor
        child_task_node1 = TaskNode(self.task_tree[1], name_prefix=self.name)
        self.child_task_nodes.append(child_task_node1)
        executor.add_node(child_task_node1)

        # child task 1 execute
        client = self.create_client(TsDoTask, child_task_node1.name)
        while not client.wait_for_service(timeout_sec = 1.0):
            print(f'service "{child_task_node1.name}" not available, waiting again...')
        req = TsDoTask.Request()
        self.future = client.call_async(req)
        result = await self.future
        if result is not None:
            print(f"[{self.name}] >> first task {child_task_node1.name} : {result.message}")
            if result.message != "Success":
                return result.message
        else:
            print(f"[{self.name}] >> first task {child_task_node1.name} : no return value")
            return "Error"

        child_task_node2 = TaskNode(self.task_tree[2], name_prefix=self.name)
        self.child_task_nodes.append(child_task_node2)
        executor.add_node(child_task_node2)

        # child task 2 execute
        client = self.create_client(TsDoTask, child_task_node2.name)
        while not client.wait_for_service(timeout_sec = 1.0):
            print(f'service "{child_task_node2.name}" not available, waiting again...')
        req = TsDoTask.Request()
        self.future = client.call_async(req)
        result = await self.future
        if result is not None:
            print(f"[{self.name}] >> second task {child_task_node2.name} : {result.message}")
            if result.message != "Success":
                return result.message
        else:
            print(f"[{self.name}] >> second task {child_task_node2.name} : no return value")
            return "Error"
        
        return "Success"

    
    async def parallel(self):
        global executor
        child_task_node1 = TaskNode(self.task_tree[1], name_prefix=self.name)
        child_task_node2 = TaskNode(self.task_tree[2], name_prefix=self.name)
        self.child_task_nodes.append(child_task_node1)
        self.child_task_nodes.append(child_task_node2)
        executor.add_node(child_task_node1)
        executor.add_node(child_task_node2)

        # parallel execute
        client1 = self.create_client(TsDoTask, child_task_node1.name)
        while not client1.wait_for_service(timeout_sec = 1.0):
            print(f'service "{child_task_node1.name}" not available, waiting again...')
        req1 = TsDoTask.Request()

        client2 = self.create_client(TsDoTask, child_task_node2.name)
        while not client2.wait_for_service(timeout_sec = 1.0):
            print(f'service "{child_task_node2.name}" not available, waiting again...')
        req2 = TsDoTask.Request()

        self.future1 = client1.call_async(req1)
        self.future2 = client2.call_async(req2)

        result1 = await self.future1
        result2 = await self.future2

        if result1 is not None:
            print(f"[{self.name}] >> first task {child_task_node1.name} : {result1.message}")
        else:
            print(f"[{self.name}] >> first task {child_task_node1.name} : no return value")
            return "Error"
        
        if result2 is not None:
            print(f"[{self.name}] >> first task {child_task_node2.name} : {result2.message}")
        else:
            print(f"[{self.name}] >> first task {child_task_node2.name} : no return value")
            return "Error"
        
        if result1.message == "Success" and result2.message == "Success":
            return "Success"
        else:
            return "Abort"

    async def subtask(self):
        command = self.task_tree[1]
        readable = [await self.read_name(c) for c in command]  # 表示用
        # readable = command

        # print(await self.read_name(command[0]))
        print(f"[{self.name}] >> start {readable}")
        wait = random.randint(5,15)
        print(f"[{self.name}] >> execute {wait} seconds for {readable}")
        time.sleep(wait)
        print(f"[{self.name}] >> end{readable}")
        return "Success"

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
        tmsdb = await self.call_dbreader(int(id))
        return tmsdb[0].name

    def destroy_node(self):
        for task_node in self.child_task_nodes:
            task_node.destroy_node()
        super().destroy_node()



class TaskSchedulerManager(Node):
    """TaskNodeの親クラス
    tms_ts_masterサービスを持ち、構文解析後、task_nodeを生成する。
    """

    def __init__(self):
        self.task_node_list = []
        self.task_node_client_list = []
        self.dic_client_to_task_node = {}
        self.dic_client_to_future = {}
        self.cb_group = ReentrantCallbackGroup()
        self.name = 'task_scheduler_manager'
        super().__init__('task_scheduler_manager')
        self.cli_dbreader = self.create_client(TmsdbGetData, 'tms_db_reader', callback_group=self.cb_group)
        while not self.cli_dbreader.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service "tms_db_reader" not available, waiting again...')
        self.srv_tms_ts_master = self.create_service(TsReq, 'tms_ts_master', self.tms_ts_master_callback, callback_group=self.cb_group)

    async def tms_ts_master_callback(self, request, response):
        global executor
        print(f"[ts_manager] receive request\n >> {request}")
        
        task = Task(
            rostime=request.rostime,
            task_id=request.task_id,
            robot_id=request.robot_id,
            object_id=request.object_id,
            user_id=request.user_id,
            place_id=request.place_id,
            priority=request.priority,
        )

        # syntax analyze
        task_tree = await self.convert_task_to_states(task)
        if task_tree == []:  # ERROR
            response.result = 0  # fault
            return response
        
        # generate task_node
        task_node = TaskNode(task_tree)
        self.task_node_list.append(task_node)
        executor.add_node(self.task_node_list[len(self.task_node_list) - 1])  # added last added task

        # make client for task_node
        self.call_task_node(task_node)

        response.result = 1  # Success
        return response

    def call_task_node(self, task_node):
        client = self.create_client(TsDoTask, task_node.name)
        self.dic_client_to_task_node[client] = task_node
        self.task_node_client_list.append(client)
        while not client.wait_for_service(timeout_sec=1.0):
            print(f'service "{task_node.name}" not available, waiting again...')
        self._timer_call_task_node = self.create_timer(0, self._call_task_node, callback_group=self.cb_group)
    
    async def _call_task_node(self):
        self._timer_call_task_node.cancel()
        req = TsDoTask.Request()
        client = self.task_node_client_list.pop()
        future = client.call_async(req)
        self.dic_client_to_future[client] = future

        print(f"[ts_manager] {self.dic_client_to_task_node[client].name} : launch")
        result = await self.dic_client_to_future[client]
        if result is not None:
            print(f"[ts_manager] {self.dic_client_to_task_node[client].name} : " + result.message)
        else:
            print(f"[ts_manager] {self.dic_client_to_task_node[client].name} : " + "error")
        
        end_task_node = self.dic_client_to_task_node[client]
        end_task_node.destroy_node()
        self.task_node_list.remove(end_task_node)
    
    async def call_dbreader(self, id):
        """[tms_db_reader] DBからデータを読み取る
        """
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
        tmsdb = await self.call_dbreader(int(id))
        return tmsdb[0].name

    async def convert_task_to_states(self, task):
        convert_dic ={
            "oid": task.object_id,
            "uid": task.user_id,
            "pid": task.place_id,
            "rid": task.robot_id,
        }

        subtask_list = []
        tmsdb_data = await self.call_dbreader(task.task_id)  # this is list
        subtask_str = tmsdb_data[0].etcdata
        print(f"[{self.name}] >> find task '{tmsdb_data[0].name}'")
        print(f"[{self.name}] >> read task '{subtask_str}'")
        subtask_raw_list = subtask_str.split(" ")
        for subtask_raw in subtask_raw_list:
            subtask = subtask_raw.split("$")
            # 辞書に含まれているなら置換
            subtask = [(str(convert_dic[command]) if command in convert_dic else command) for command in subtask]
            subtask_list.append(subtask)

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


    def destroy_node(self):
        for task_node in self.task_node_list:
            task_node.destroy_node()
        super().destroy_node()


def main(args=None):
    global executor
    rclpy.init(args=args)
    try:
        executor = MultiThreadedExecutor()

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