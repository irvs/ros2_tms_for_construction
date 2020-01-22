from abc import ABCMeta, abstractmethod
from tms_msg_db.srv import TmsdbGetData
from tms_msg_ts.srv import TsDoTask
from std_msgs.msg import String
import rclpy
from rclpy.node import Node
from rclpy.executors import Executor, ShutdownException, TimeoutException
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from tms_msg_ts.action import TsDoSubtask
import json

class SubtaskNodeBase(Node, metaclass=ABCMeta):
    """サブタスクを定義する抽象クラス
    """
    def __init__(self):
        super().__init__(self.node_name())
        self._dict = self.init_argument()
        self.cb_group = ReentrantCallbackGroup()
        
        self._action_server = ActionServer(
            self,
            TsDoSubtask,
            "subtask_node_" + str(self.id()),
            execute_callback=self.execute_callback,
            callback_group=self.cb_group,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback
        )
        # self.srv = self.create_service(TsDoTask, \
        #     "subtask_node_" + str(self.id()), \
        #     self._service_callback, \
        #     # self._test_service_callback, \
        #     callback_group = self.cb_group)
        
    def destroy(self):
        self._action_server.destroy()
        super().destroy_node()
    
    def goal_callback(self, goal_request):
        self.get_logger().info("Received goal request")
        return GoalResponse.ACCEPT
    
    def cancel_callback(self, goal_handle):
        self.get_logger().info('Received cancel request')
        self._timer = self.create_timer(0.0, self._cancel_service_callback, callback_group=ReentrantCallbackGroup())
        return CancelResponse.ACCEPT

    async def execute_callback(self, goal_handle):
        self.get_logger().info(">> Start")
        self._dict.update(json.loads(goal_handle.request.arg_json))

        result = await self.service_callback(self._dict, TsDoSubtask.Result(), goal_handle)
        self.get_logger().warning(result.message)

        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
            result.message = "Canceled"
        elif result.message == "Success":
            goal_handle.succeed()
        elif result.message == "Canceled":
            goal_handle.canceled()
        else:  # Abortとして処理
            goal_handle.abort()
        
        if result.message != "Success":
            self.get_logger().warning(f"return {result.message}")
        return result

    @abstractmethod
    def node_name(self) -> str:
        """ROS2 ノードの名前"""
        pass

    @abstractmethod
    def id(self) -> int:
        """データベースID"""
        pass
    
    
    # async def _service_callback(self, request, response):
    #     """引数更新・実行ログ用"""
    #     self.get_logger().info(">> Start")
    #     # 引数を更新
    #     self._dict.update(json.loads(request.arg_json))
        
    #     response = await self.service_callback(self._dict, response)

    #     self.get_logger().info(f">> {response.message}")
    #     return response 

        

    @abstractmethod
    async def service_callback(self, request, response, goal_handle) -> "response":
        """実行時の働き"""
        pass

    async def _cancel_service_callback(self):
        self._timer.cancel()
        await self.cancel_service_callback()
    
    async def cancel_service_callback(self):
        """キャンセルときに必要な動作がある場合オーバーライドする
        """
        pass

    def init_argument(self) -> dict:
        """サブタスクに引数が必要な場合、オーバーライドしてください
        Returns:
            dict: タスクの引数をkey、デフォルト値をvalueとして辞書にしたもの
        """
        return {}

    def _test_service_callback(self, request, response, goal_handle) -> "response":
        self.get_logger().info("Callback accepted")
        wait = random.randint(5,15)
        self.get_logger().info(f"execute {wait} seconds")
        time.sleep(wait)
        self.get_logger().info("success")
        response.message = "Success"
        return response

