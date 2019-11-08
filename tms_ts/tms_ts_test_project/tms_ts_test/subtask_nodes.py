from abc import ABCMeta, abstractmethod
from tms_msg_db.srv import TmsdbGetData
from tms_msg_ts.srv import TsDoTask
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
import time
import random
import json

def main(args=None):
    global executor
    rclpy.init(args=args)

    try:
        executor = MultiThreadedExecutor()
        executor.add_node(SubtaskMove())
        executor.add_node(SubtaskGrasp())
        executor.add_node(SubtaskRelease())
        executor.add_node(SubtaskOpen())
        executor.add_node(SubtaskClose())
        executor.add_node(SubtaskSensing())
        executor.add_node(SubtaskRandomMove())
        executor.add_node(SubtaskWait())

        try:
            executor.spin()
        finally:
            executor.shutdown()

    finally:
        rclpy.shutdown()

if __name__=='__main__':
    main()


class SubtaskNodeBase(Node, metaclass=ABCMeta):
    """サブタスクを定義する抽象クラス
    """
    def __init__(self):
        super().__init__(self.node_name())
        self._dict = self.init_argument()
        self.cb_group = ReentrantCallbackGroup()
        self.srv = self.create_service(TsDoTask, \
            "subtask_node_" + str(self.id()), \
            self._service_callback, \
            # self._test_service_callback, \
            callback_group = self.cb_group)
        
        self.get_logger().info(f'begin service "subtask_node_{self.id()}"')

    @abstractmethod
    def node_name(self) -> str:
        """ROS2 ノードの名前"""
        pass

    @abstractmethod
    def id(self) -> int:
        """データベースID"""
        pass
    
    async def _service_callback(self, request, response):
        """引数更新・実行ログ用"""
        self.get_logger().info(">> Start")
        # 引数を更新
        self._dict.update(json.loads(request.arg_json))
        response = await self.service_callback(self._dict, response)
        self.get_logger().info(f">> {response.message}")
        return response 

    @abstractmethod
    async def service_callback(self, request, response) -> "response":
        """実行時の働き"""
        pass

    def init_argument(self) -> dict:
        """サブタスクに引数が必要な場合、オーバーライドしてください
        Returns:
            dict: タスクの引数をkey、デフォルト値をvalueとして辞書にしたもの
        """
        return {}

    def _test_service_callback(self, request, response) -> "response":
        self.get_logger().info("Callback accepted")
        wait = random.randint(5,15)
        self.get_logger().info(f"execute {wait} seconds")
        time.sleep(wait)
        self.get_logger().info("success")
        response.message = "Success"
        return response


class SubtaskMove(SubtaskNodeBase):
    def node_name(self):
        return "subtask_move"
    
    def id(self):
        return 9001
    
    async def service_callback(self, request, response):
        response.message = "Success"
        return response


class SubtaskGrasp(SubtaskNodeBase):
    def node_name(self):
        return "subtask_grasp"
    
    def id(self):
        return 9002
    
    async def service_callback(self, request, response):
        response.message = "Success"
        return response


class SubtaskRelease(SubtaskNodeBase):
    def node_name(self):
        return "subtask_release"
    
    def id(self):
        return 9003
    
    async def service_callback(self, request, response):
        response.message = "Success"
        return response


class SubtaskOpen(SubtaskNodeBase):
    def node_name(self):
        return "subtask_open"
    
    def id(self):
        return 9004
    
    async def service_callback(self, request, response):
        response.message = "Success"
        return response


class SubtaskClose(SubtaskNodeBase):
    def node_name(self):
        return "subtask_close"
    
    def id(self):
        return 9005
    
    async def service_callback(self, request, response):
        response.message = "Success"
        return response


class SubtaskRandomMove(SubtaskNodeBase):
    def node_name(self):
        return "subtask_random_move"
    
    def id(self):
        return 9006
    
    async def service_callback(self, request, response):
        response.message = "Success"
        return response


class SubtaskSensing(SubtaskNodeBase):
    def node_name(self):
        return "subtask_sensing"
    
    def id(self):
        return 9007
    
    async def service_callback(self, request, response):
        response.message = "Success"
        return response


class SubtaskWait(SubtaskNodeBase):
    def node_name(self):
        return "subtask_wait"
    
    def id(self):
        return 9900
    
    async def service_callback(self, request, response):
        self.get_logger().info(f'{request["wait_msec"]}')
        time.sleep(request["wait_msec"])
        response.message = "Success"
        return response
    
    def init_argument(self):
        """引数が必要なため、オーバーライド"""
        return {"wait_msec" : 30}
