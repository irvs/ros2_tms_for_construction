import rclpy
from tms_ts_subtask.subtask_node_base import SubtaskNodeBase
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
import time
from tms_msg_ur.srv import SpeakerSrv
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient

def main(args=None):
    global executor
    rclpy.init(args=args)
    try:
        executor = MultiThreadedExecutor()
        executor.add_node(SubtaskTestPrint())
        try:
            executor.spin()
        finally:
            executor.shutdown()
    finally:
        rclpy.shutdown()

if __name__=='__main__':
    main()



class SubtaskMove(SubtaskNodeBase):
    def __init__(self):
        super().__init__()
        self.move_goalhandle = None

    def node_name(self):
        return "subtask_move"
    
    def id(self):
        return 9001
    
    async def service_callback(self, request, response, goal_handle):
        position = request["position"]
        orientation = request["orientation"]
        self.action_client = ActionClient(self, NavigateToPose, '/NavigateToPose', callback_group=self.cb_group)
        if not self.action_client.wait_for_server(timeout_sec=5.0):
            response.message = "Abort"
            return response
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = "map"
        goal_msg.pose.pose.position.x = position[0]
        goal_msg.pose.pose.position.y = position[1]
        goal_msg.pose.pose.position.z = position[2]
        goal_msg.pose.pose.orientation.x = orientation[0]
        goal_msg.pose.pose.orientation.y = orientation[1]
        goal_msg.pose.pose.orientation.z = orientation[2]
        goal_msg.pose.pose.orientation.w = orientation[3]
        self.move_goalhandle = await self.action_client.send_goal_async(goal_msg)
        if not self.move_goalhandle.accepted:
            self.get_logger().info("goal rejected")
            response.message = "Abort"
            return response
        future_result = await self.move_goalhandle.get_result_async()
        
        response.message = "Success"
        return response
    
    async def cancel_service_callback(self):
        """@override
        """
        print("Cancel action_move")
        if self.move_goalhandle is not None:
            await self.move_goalhandle.cancel_goal_async()


class SubtaskGrasp(SubtaskNodeBase):
    def node_name(self):
        return "subtask_grasp"
    
    def id(self):
        return 9002
    
    async def service_callback(self, request, response, goal_handle):
        response.message = "Success"
        return response


class SubtaskRelease(SubtaskNodeBase):
    def node_name(self):
        return "subtask_release"
    
    def id(self):
        return 9003
    
    async def service_callback(self, request, response, goal_handle):
        response.message = "Success"
        return response


class SubtaskOpen(SubtaskNodeBase):
    def node_name(self):
        return "subtask_open"
    
    def id(self):
        return 9004
    
    async def service_callback(self, request, response, goal_handle):
        response.message = "Success"
        return response


class SubtaskClose(SubtaskNodeBase):
    def node_name(self):
        return "subtask_close"
    
    def id(self):
        return 9005
    
    async def service_callback(self, request, response, goal_handle):
        response.message = "Success"
        return response


class SubtaskRandomMove(SubtaskNodeBase):
    def node_name(self):
        return "subtask_random_move"
    
    def id(self):
        return 9006
    
    async def service_callback(self, request, response, goal_handle):
        response.message = "Success"
        return response


class SubtaskSensing(SubtaskNodeBase):
    def node_name(self):
        return "subtask_sensing"
    
    def id(self):
        return 9007
    
    async def service_callback(self, request, response, goal_handle):
        response.message = "Success"
        return response


class SubtaskWait(SubtaskNodeBase):
    def node_name(self):
        return "subtask_wait"
    
    def id(self):
        return 9900
    
    async def service_callback(self, request, response, goal_handle):
        self.get_logger().info(f'{request["wait_sec"]}')
        time.sleep(request["wait_sec"])
        response.message = "Success"
        return response
    
    def init_argument(self):
        """引数が必要なため、オーバーライド"""
        return {"wait_sec" : 30}

class SubtaskSpeakerAnnounce(SubtaskNodeBase):
    def node_name(self):
        return "subtask_speaker_announce"
    
    def id(self):
        return 9300
    
    async def service_callback(self, request, response, goal_handle):
        self.get_logger().info(f'{request["announce"]}')
        self.cli = self.create_client(SpeakerSrv, "speaker_srv", callback_group=ReentrantCallbackGroup())
        req = SpeakerSrv.Request()
        req.data = request["announce"]
        await self.cli.call_async(req)
        response.message = "Success"
        return response
    
    def init_argument(self):
        return {"announce" : "よくわかりませんでした"}


class SubtaskTestPrint(SubtaskNodeBase):
    def node_name(self):
        return "subtask_test_print"
    
    def id(self):
        return 9401
    
    async def service_callback(self, request, response, goal_handle):
        print("Hello World! from subtask_test_print")
        response.message = "Success"
        return response
