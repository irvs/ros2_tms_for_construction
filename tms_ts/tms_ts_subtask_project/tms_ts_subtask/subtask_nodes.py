import rclpy
from tms_ts_subtask.subtask_node_base import SubtaskNodeBase
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
import math
import numpy as np
import time
from tms_msg_ur.srv import SpeakerSrv
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from std_msgs.msg import String, Float64
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

def main(args=None):
    global executor
    rclpy.init(args=args)
    try:
        executor = MultiThreadedExecutor()
        #executor.add_node(SubtaskTestPrint())
        executor.add_node(SubtaskControlZx120())
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
        goal_msg =pose.pose.orientation.y = orientation[1]
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
        response.message = "SuccSubtaskControlTurtlebotess"
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
        self.get_alogger().info(f'{request["wait_sec"]}')
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
        self.get_logger().info("Hello World! from subtask_test_print")
        response.message = "Success"
        return response
    
class SubtaskControlTurtlebot(SubtaskNodeBase):  
    def node_name(self):
        return "subtask_control_turtlebot"
    
    def id(self):
        return 9502
    
    class Pose:
        def __init__(self, x = 0.0, y = 0.0, yaw = 0.0):
            self.x = x
            self.y = y
            self.yaw = yaw
        
        def set(self, x, y, yaw): 
            self.x = x
            self.y = y
            self.yaw = yaw
        
        def get(self):
            return self.x, self.y, self.yaw     

    def euler_from_quaternion(self,quaternion):
        x = quaternion.x                                                                                          
        y = quaternion.y
        z = quaternion.z
        w = quaternion.w                                                                                  
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll      = np.arctan2(sinr_cosp, cosr_cosp)
        sinp  = 2 * (w * y - z * x)
        pitch = np.arcsin(sinp)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)
        return roll, pitch, yaw
    
    def odom_callback(self, odom):
        x = odom.pose.pose.position.x
        y = odom.pose.pose.position.y
        roll, pitch, yaw = self.euler_from_quaternion(odom.pose.pose.orientation)
        self.pose.set(x, y, yaw)
    
    def move(self, linear, angular = 0.0):
        self.vel.linear.x = linear
        self.vel.angular.z = angular
        self.pub.publish(self.vel)
    
    def distance(self, pose1, pose0):
        d = math.sqrt((pose1.x - pose0.x) * (pose1.x - pose0.x) + (pose1.y - pose0.y) * (pose1.y - pose0.y))
        return d
    
    async def service_callback(self, request, response, goal_handle):
        self.get_logger().info("Move Turtlebot! from control_turtlebot")
        self.pub = self.create_publisher(Twist,'cmd_vel', 10)
        self.sub = self.create_subscription(Odometry,'odom', self.odom_callback, 10)
        self.vel = Twist()
        self.state = 0
        self.odom  = Odometry()
        self.pose  = self.Pose()
        v = Twist()                                                                 
        t = 5.0                                                       
        d = 1.0
        while rclpy.ok():
            if self.state == 0:
                self.state = 1
                start_time = time.time()
            elif self.state == 1:
                self.move(0.2)
                if time.time() - start_time > t:
                    self.state = 2
                    start_time = time.time()
            elif self.state == 2:
                self.move(0.0)
                if time.time() - start_time > t:
                    self.state = 3
                    start_pose = self.Pose()
                    start_pose.x, start_pose.y, start_pose.yaw = self.pose.get()
            elif self.state == 3:
                self.move(-0.2)
                if self.distance(self.pose, start_pose) > d:
                    self.state = 4
            else:
                self.move(0.0)
    
        response.message = "Success"
        return response


class SubtaskControlZx120(SubtaskNodeBase):
    def node_name(self):
        return "subtask_zx120_boom_sample"
    
    def id(self):
        return 8001
    
    async def service_callback(self, request, response, goal_handle):
        self.publisher = self.create_publisher(Float64, '/zx120/boom/cmd', 10)
        msg = Float64()
        msg.data = -2.0
        while rclpy.ok():
            self.publisher.publish(msg)

        self.get_logger().info("Control zx120 !! from subtask_zx120_sample")
        response.message = "Success"
        return response

"""class SubtaskControlZx120Swing(SubtaskNodeBase):
    def node_name(self):
        return "subtask_zx120_sample_swing"
    
    def id(self):
        return 9505
    
    async def service_callback(self, request, response, goal_handle):
        self.publisher = self.create_publisher(Float64, '/zx120/swing/cmd', 10)
        msg = Float64()
        msg.data = -2.0
        while rclpy.ok():
            self.publisher.publish(msg)

        self.get_logger().info("Control zx120 !! from subtask_zx120_sample_swing")
        response.message = "Success"
        return response"""