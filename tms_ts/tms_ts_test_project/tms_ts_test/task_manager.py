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


class SubtaskNode(Node):
    """サブタスクを実行するノード
    サブタスクは順序（状態）を持たない最小構成のタスク
    """
    _count = 0  # use for name
    def __init__(self, subtask=[], parent_name=""):
        self.name = f"subtask_{SubtaskNode._count}"
        self.parent_name = parent_name
        SubtaskNode._count += 1
        self.subtask = subtask
        self.cb_group = ReentrantCallbackGroup()
        super().__init__(self.name)
        self.srv = self.create_service(TsDoTask, self.name, self.main, callback_group=self.cb_group)

    async def main(self, request, response):
        sleep_time = random.randint(3,15)
        print(f"{self.parent_name}[{self.name}] >> start {self.subtask}")
        time.sleep(0.1)
        print(f"{self.parent_name}[{self.name}] >> take {sleep_time} seconds for {self.subtask}")
        time.sleep(sleep_time)
        if random.random() > 0.05:
            print(f"{self.parent_name}[{self.name}] >> end {self.subtask}")
            response.message = "Success"
        else:
            print(f"{self.parent_name}[{self.name}] >> ERROR {self.subtask}")
            response.message = "Abort"

        return response


class TaskScheduler(Node):
    """タスクを実行するノード
    タスクは順序（状態）をもつ
    タスクはサブタスクに分けることができる
    """
    _task_num = 0  # it is task count for name, not task_id 

    def __init__(self,task=Task()):
        self.name = "task_" + str(TaskScheduler._task_num)
        TaskScheduler._task_num += 1
        self.task = task
        self.subtask_node_list = []
        self.cb_group = ReentrantCallbackGroup()
        self.dic_subtask_node_to_future = {}

        super().__init__(self.name)
        self.cli_dbreader = self.create_client(TmsdbGetData, 'tms_db_reader', callback_group=self.cb_group)
        while not self.cli_dbreader.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service "tms_db_reader" not available, waiting again...')
    
        self.i = 0
        self.srv = self.create_service(TsDoTask, self.name, self.main, callback_group=self.cb_group)

    def destroy_node(self):
        for subtask_node in self.subtask_node_list:
            subtask_node.destroy_node()
        super().destroy_node()

    def destroy_subtask_nodes(self):
        for subtask_node in self.subtask_node_list:
            subtask_node.destroy_node()
        self.subtask_node_list = []

    async def call_dbreader(self, id):
        """[tms_db_reader] DBからデータを読み取る
        """
        # print(f"[tms_db_reader] << {id}")
        req = TmsdbGetData.Request()
        req.tmsdb.id = id + 100000  # TODO: why add 100000 ?
        self.future_dbreader  = self.cli_dbreader.call_async(req)

        await self.future_dbreader

        if self.future_dbreader.result() is not None:
            res = self.future_dbreader.result().tmsdb
            return res
        else:
            self.get_logger().info('Service "tms_db_reader" call failed %r' % (self.future_dbreader.exception(),))

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
                # _executable.append(_stack.pop(-1))
                pass  # 無視
            elif s == ['|']:
                pre1 = _stack.pop(-1)
                pre2 = _stack.pop(-1)
                pre2.extend(pre1)
                _stack.append(pre2)
            else:
                _stack.append([s])
        
        states = _stack
        print(f"[{self.name}] >> analyze task")
        i=0
        for state in states:
            readable = []
            for s in state:
                readable.append([await self.read_name(c) for c in s])
            print(f"\tstate{i} : {readable}")
            i += 1

        return states
    
    async def read_name(self, id):
        tmsdb = await self.call_dbreader(int(id))
        return tmsdb[0].name

    async def call_subtask_nodes(self):
        # call async subtasks
        for subtask_node in self.subtask_node_list:
            client = self.create_client(TsDoTask, subtask_node.name)
            while not client.wait_for_service(timeout_sec=1.0):
                print(f'service "{subtask_node.name}" not available, waiting again...')
            req = TsDoTask.Request()
            future = client.call_async(req)
            self.dic_subtask_node_to_future[subtask_node] = future
        
        results = []
        # wait all subtasks
        for subtask_node in self.subtask_node_list:
            result = await self.dic_subtask_node_to_future[subtask_node]
            if result is not None:
                results.append(result.message)
                # print(f"[{self.name}] >> {result.message} {subtask_node.name}")
            else:
                results.append("Error")
                # print(f"[{self.name}] >> Error")
        
        # when all subtasks return success, state is success
        is_success = True
        for result in results:
            if result != "Success":
                is_success = False
        
        self.destroy_subtask_nodes()  # destroy all subtasks
        return is_success

    async def main(self, request, response):
        global executor
        is_success = True

        print(f"[{self.name}] >> init")

        # taskを読み取り、いくつかのstateに変換する
        # stateはいくつかのsubtaskから構成される
        task_name = await self.read_name(self.task.task_id)
        states = await self.convert_task_to_states(self.task)
        print(f"[{self.name}] >> start task '{task_name}'")

        i = 0
        for state in states:
            readable = []  # 表示用
            for s in state:
                readable.append([await self.read_name(c) for c in s])
            print(f"[{self.name}] [state: {i}] >> start subtasks '{readable}'")

            for subtask in readable: #for subtask in state:
                subtask_node = SubtaskNode(subtask,parent_name=f"[{self.name}] [state: {i}] ")
                self.subtask_node_list.append(subtask_node)
                executor.add_node(subtask_node)

            is_success = await self.call_subtask_nodes()
            if not is_success:
                print(f"[{self.name}] [state: {i}] >> ERROR subtasks '{readable}'")
                break
            print(f"[{self.name}] [state: {i}] >> end subtasks '{readable}'")
            i += 1
        print(f"[{self.name}] >> end task '{task_name}'")

        if is_success:
            response.message = "Success"
        else:
            response.message = "Abort"
        
        return response


class TaskSchedulerManager(Node):
    """複数のタスク（スケジューラ）を管理するノード
    同時に複数のタスクを起動できる
    """
    def __init__(self):
        self.task_scheduler_list = []
        self.cb_group = ReentrantCallbackGroup()
        super().__init__('task_scheduler_manager')
        self.srv_tms_ts_master = self.create_service(TsReq, 'tms_ts_master', self.tms_ts_master_callback, callback_group=self.cb_group)
        self.task_scheduler_clients = []
        self.dic_client_to_future = {}
        self.dic_client_to_task_scheduler = {}
        self.dic_client_to_task_scheduler_name = {}
    
    def tms_ts_master_callback(self, request, response):
        global executor
        print("[ts_manager] receive request\n >> " + str(request))
        
        task = Task(
            rostime=request.rostime,
            task_id=request.task_id,
            robot_id=request.robot_id,
            object_id=request.object_id,
            user_id=request.user_id,
            place_id=request.place_id,
            priority=request.priority,
        )

        # generate task_scheduler node
        task_scheduler = TaskScheduler(task)
        self.task_scheduler_list.append(task_scheduler)
        executor.add_node(self.task_scheduler_list[len(self.task_scheduler_list) - 1])  # added last added task

        # make client for task_scheduler
        self.call_taskmanager(task_scheduler)

        response.result = 1  # Success
        return response
    
    def call_taskmanager(self, task_scheduler):
        client = self.create_client(TsDoTask, task_scheduler.name)
        self.dic_client_to_task_scheduler_name[client] = task_scheduler.name
        self.dic_client_to_task_scheduler[client] = task_scheduler
        while not client.wait_for_service(timeout_sec=1.0):
            print(f'service "{task_scheduler.name}" not available, waiting again...')
        self.task_scheduler_clients.append(client)
        self.timer = self.create_timer(0.1, self._call_taskmanager, callback_group=self.cb_group)

    async def _call_taskmanager(self):
        self.timer.cancel()
        req = TsDoTask.Request()
        client = self.task_scheduler_clients.pop()
        future = client.call_async(req)
        self.dic_client_to_future[client] = future

        print(f"[ts_manager] {self.dic_client_to_task_scheduler_name[client]} : launch")
        result = await self.dic_client_to_future[client]
        if result is not None:
            print(f"[ts_manager] {self.dic_client_to_task_scheduler_name[client]} : " + result.message)
        else:
            print(f"[ts_manager] {self.dic_client_to_task_scheduler_name[client]} : " + "error")

        end_task_scheduler = self.dic_client_to_task_scheduler[client]
        end_task_scheduler.destroy_node()
        self.task_scheduler_list.remove(end_task_scheduler)

    def destroy_node(self):
        for task_scheduler in self.task_scheduler_list:
            task_scheduler.destroy_node()
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