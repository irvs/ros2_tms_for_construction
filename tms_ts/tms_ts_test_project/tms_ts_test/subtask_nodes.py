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

from tms_msg_ur.srv import SpeakerSrv
from geometry_msgs.msg import PoseStamped

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
        executor.add_node(SubtaskSpeakerAnnounce())

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


from rcl_interfaces.msg import Log

class SubtaskMove(SubtaskNodeBase):
    def __init__(self):
        super().__init__()
        self.log_subscriber = self.create_subscription(Log, "/rosout", self.log_callback, callback_group=ReentrantCallbackGroup())
        self.is_navigation_end = False
        self.is_navigation_start = False
    
    def log_callback(self, data):
        print(f'{data.name} : {data.msg}')

        if data.name == "dwb_controller" and data.msg == "Received a goal, begin following path":
            self.is_navigation_start = True
        if data.name == "bt_navigator" and \
            data.msg == "Navigation succeeded":
            self.is_navigation_end = True
        

    def node_name(self):
        return "subtask_move"
    
    def id(self):
        return 9001
    
    async def service_callback(self, request, response):
        place_id = request["PLACE_ID"]
        position = request["position"]
        orientation = request["orientation"]
        print(f"position: {position}, orientation: {orientation}")
        if place_id != -1:
            pass
        
        self.is_navigation_start = False
        while not self.is_navigation_start:
            self.publisher = self.create_publisher(PoseStamped, "/goal_pose", 10)
            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = "map"
            pose_stamped.pose.position.x = position[0]
            pose_stamped.pose.position.y = position[1]
            pose_stamped.pose.position.z = position[2]
            pose_stamped.pose.orientation.x = orientation[0]
            pose_stamped.pose.orientation.y = orientation[1]
            pose_stamped.pose.orientation.z = orientation[2]
            pose_stamped.pose.orientation.w = orientation[3]
            self.publisher.publish(pose_stamped)
            self.publisher = self.create_publisher(PoseStamped, "/move_base_simple/goal", 10)
            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = "map"
            pose_stamped.pose.position.x = position[0]
            pose_stamped.pose.position.y = position[1]
            pose_stamped.pose.position.z = position[2]
            pose_stamped.pose.orientation.x = orientation[0]
            pose_stamped.pose.orientation.y = orientation[1]
            pose_stamped.pose.orientation.z = orientation[2]
            pose_stamped.pose.orientation.w = orientation[3]
            self.publisher.publish(pose_stamped)

            time.sleep(1)

        self.is_navigation_end = False
        while not self.is_navigation_end:
            print('navigation drive now...')
            time.sleep(1)
        """
        self.publisher = self.create_publisher(PoseStamped, "/goal_pose", 10)
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = "map"
        pose_stamped.pose.position.x = position[0]
        pose_stamped.pose.position.y = position[1]
        pose_stamped.pose.position.z = position[2]
        pose_stamped.pose.orientation.x = orientation[0]
        pose_stamped.pose.orientation.y = orientation[1]
        pose_stamped.pose.orientation.z = orientation[2]
        pose_stamped.pose.orientation.w = orientation[3]
        self.publisher.publish(pose_stamped)
        self.publisher = self.create_publisher(PoseStamped, "/move_base_simple/goal", 10)
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = "map"
        pose_stamped.pose.position.x = position[0]
        pose_stamped.pose.position.y = position[1]
        pose_stamped.pose.position.z = position[2]
        pose_stamped.pose.orientation.x = orientation[0]
        pose_stamped.pose.orientation.y = orientation[1]
        pose_stamped.pose.orientation.z = orientation[2]
        pose_stamped.pose.orientation.w = orientation[3]
        self.publisher.publish(pose_stamped)
        """
        response.message = "Success"
        return response
    
    def init_argument(self):
        """引数が必要なため、オーバーライド"""
        return {"PLACE_ID": -1, "position": [8.65, 1.62, 0.0], "orientation": [0.0, 0.0, 0.280, 0.960]}


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

class SubtaskSpeakerAnnounce(SubtaskNodeBase):
    def node_name(self):
        return "subtask_speaker_announce"
    
    def id(self):
        return 9300
    
    async def service_callback(self, request, response):
        self.get_logger().info(f'{request["announce"]}')
        self.cli = self.create_client(SpeakerSrv, "speaker_srv", callback_group=ReentrantCallbackGroup())
        req = SpeakerSrv.Request()
        req.data = request["announce"]
        await self.cli.call_async(req)
        response.message = "Success"
        return response
    
    def init_argument(self):
        return {"announce" : "よくわかりませんでした"}
