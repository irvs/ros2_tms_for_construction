from graphviz import Digraph

from tms_msg_ts.srv import TsReq
from tms_msg_db.srv import TmsdbGetData

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup

import os
import threading
from flask import Flask
app = Flask(__name__)

@app.route('/')
def hello_world():
    return os.getcwd()

class Task:
    def __init__(self, rostime=0, task_id=0, robot_id=0, object_id=0, user_id=0, place_id=0, priority=0):
        self.rostime = rostime
        self.task_id = task_id
        self.robot_id = robot_id
        self.object_id = object_id
        self.user_id = user_id
        self.place_id = place_id
        self.priority = priority


class TaskViewer(Node):

    def __init__(self):
        self.name = 'task_viewer'
        self.cb_group = ReentrantCallbackGroup()

        super().__init__(self.name)
        self.create_service(TsReq, 'task_viewer', self.task_viewer_callback, callback_group=self.cb_group)
    
    async def task_viewer_callback(self, request, response):
        print(f"[task_viewer] receive request\n >> {request}")
        
        task = Task(
            rostime=request.rostime,
            task_id=request.task_id,
            robot_id=request.robot_id,
            object_id=request.object_id,
            user_id=request.user_id,
            place_id=request.place_id,
            priority=request.priority,
        )
        task_tree = await self.convert_task_to_states(task)

        self.G = Digraph(format='png')
        self.draw_graph_i = 0
        await self.draw_graph(task_tree)
        self.G.render('/home/itsuka/ros2_ws/src/ros2_tms/tms_ts/tms_ts_test_project/png/task_node_tree')
        await self.draw_statement(task_tree)

        return response

    async def draw_statement(self, task_tree):
        self.G2 = Digraph(format='png')
        self.draw_statement_i = 0
        self.G2.attr('node', shape='ellipse')
        self.G2.node("Start")
        self.G2.node("End")
        task = await self._draw_statement(task_tree)
        for t in task[0]:
            self.G2.edge("Start", t)
        for t in task[1]:
            self.G2.edge(t, "End")

        self.G2.render('/home/itsuka/ros2_ws/src/ros2_tms/tms_ts/tms_ts_test_project/png/task_statement')

    async def _draw_statement(self, task_tree):
        if task_tree[0] == "serial":
            self.draw_statement_i += 1
            task_1 = await self._draw_statement(task_tree[1])
            task_2 = await self._draw_statement(task_tree[2])
            for t1 in task_1[1]:
                for t2 in task_2[0]:
                    self.G2.edge(t1,t2)
            return [task_1[0], task_2[1]]
        elif task_tree[0] == "parallel":
            self.draw_statement_i += 1
            task_1 = await self._draw_statement(task_tree[1])
            task_2 = await self._draw_statement(task_tree[2])
            return [task_1[0] + task_2[0] , task_1[1] + task_2[1]]
        elif task_tree[0] == "subtask":
            subtask = [await self.read_name(c) for c in task_tree[1]]
            this_subtask_name = str(subtask) +" (" + str(self.draw_statement_i) + ")"
            self.draw_statement_i += 1
            self.G2.attr('node', shape='rect')
            self.G2.node(this_subtask_name)
            return [[this_subtask_name], [this_subtask_name]]
            

    async def draw_graph(self, task_tree, parent_name=""):

        if task_tree[0] == "serial" or task_tree[0] == "parallel":
            this_name = str(task_tree[0])+ str(self.draw_graph_i )
            self.draw_graph_i  += 1
            if task_tree[0] == "serial":
                self.G.attr('node', shape='cds')
            else: # parallel
                self.G.attr('node', shape='parallelogram')

            self.G.node(this_name)
            if parent_name != "": 
                self.G.edge(parent_name, this_name)
            await self.draw_graph(task_tree[1], parent_name=this_name)
            await self.draw_graph(task_tree[2], parent_name=this_name)
        elif task_tree[0] == "subtask":
            subtask = [await self.read_name(c) for c in task_tree[1]]
            this_name = str(task_tree[0])+ str(self.draw_graph_i )
            this_subtask_name = str(subtask) +" (" + str(self.draw_graph_i) + ")"
            self.draw_graph_i  += 1
            self.G.attr('node', shape='ellipse')
            self.G.node(this_name)
            self.G.attr('node', shape='rect')
            self.G.node(this_subtask_name)
            if parent_name != "": 
                self.G.edge(parent_name, this_name)
            self.G.edge(this_name, this_subtask_name)


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


def ros2_main(args=None):
    rclpy.init(args=args)
    
    task_viewer = TaskViewer()
    rclpy.spin(task_viewer)

    rclpy.shutdown()

def main(args=None):

    t1 = threading.Thread(target=ros2_main)
    t2 = threading.Thread(target=server)
    t1.start()
    t2.start()

def server():
    app.run()

if __name__=='__main__':
    main()
