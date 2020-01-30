import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from tms_ts_subtask.subtask_node_base import SubtaskNodeBase
import time
from threading import Event 
from websocket import create_connection


# MACRO
TIME = 20  # ベッドの動作のための待つ時間
# ベッドのコマンド
LINK_UP = '1'
LINK_DOWN = '2'
HEAD_UP = '3'
HEAD_DOWN = '4'
FOOT_UP = '5'
FOOT_DOWN = '6'
HEIGHT_UP = '7'
HEIGHT_DOWN = '8'
# ベッドのwebsocket address
BED_WS_ADDRESS = 'ws://192.168.4.131:9989'


def main(args=None):
    global executor
    rclpy.init(args=args)

    try:
        executor = MultiThreadedExecutor()
        executor.add_node(SubtaskBedLinkUp())
        executor.add_node(SubtaskBedLinkDown())
        executor.add_node(SubtaskBedHeadUp())
        executor.add_node(SubtaskBedHeadDown())
        executor.add_node(SubtaskBedFootUp())
        executor.add_node(SubtaskBedFootDown())
        executor.add_node(SubtaskBedHeightUp())
        executor.add_node(SubtaskBedHeightDown())

        try:
            executor.spin()
        finally:
            executor.shutdown()

    finally:
        rclpy.shutdown()

if __name__=='__main__':
    main()


class SubtaskBedLinkUp(SubtaskNodeBase):
    """ベッドの頭、足を組み合わせて起こすタスク
    """
    def node_name(self):
        return "subtask_bed_link_up"
    
    def id(self):
        return 9100
    
    async def service_callback(self, request, response, goal_handle):
        ws = create_connection(BED_WS_ADDRESS)  # open socket
        ws.send(LINK_UP + f",{request['sec']}")  # send to socket
        response.message = ws.recv()
        self.get_logger().error(response.message)
        ws.close()  # close socket
        return response
        # TIME = request['sec']
        # time.sleep(TIME)
        # event = Event()
        # while not event.wait(timeout=TIME):
        #     if goal_handle.is_cancel_requested:
        #         response.message = "Canceled"
        #         return response
        # response.message = "Success"
        # return response
    
    def init_argument(self):
        return {"sec": 20.0}
    
    async def cancel_service_callback(self):
        """@override
        """
        print("Cancel subtask_bed_link_up")
        ws = create_connection(BED_WS_ADDRESS)  # open socket
        ws.send("0,0.0")  # send to socket
        ws.close()  # close socket



class SubtaskBedLinkDown(SubtaskNodeBase):
    """ベッドの頭、足を組み合わせて寝かせるタスク
    """
    def node_name(self):
        return "subtask_bed_link_down"
    
    def id(self):
        return 9101
    
    async def service_callback(self, request, response, goal_handle):
        ws = create_connection(BED_WS_ADDRESS)  # open socket
        ws.send(LINK_DOWN + f",{request['sec']}")  # send to socket
        response.message = ws.recv()
        self.get_logger().error(response.message)
        ws.close()  # close socket
        return response
    # async def service_callback(self, request, response, goal_handle):
    #     ws = create_connection(BED_WS_ADDRESS)  # open socket
    #     ws.send(LINK_DOWN + f",{request['sec']}")  # send to socket
    #     ws.close()  # close socket
    #     TIME = request['sec']

    #     time.sleep(TIME)
    #     # event = Event()
    #     # while not event.wait(timeout=TIME):
    #     #     if goal_handle.is_cancel_requested:
    #     #         response.message = "Canceled"
    #     #         return response
    #     response.message = "Success"
    #     return response
    
    def init_argument(self):
        return {"sec": 20.0}
    
    async def cancel_service_callback(self):
        """@override
        """
        print("Cancel subtask_bed_link_up")
        ws = create_connection(BED_WS_ADDRESS)  # open socket
        ws.send("0,0.0")  # send to socket
        ws.close()  # close socket


class SubtaskBedHeadUp(SubtaskNodeBase):
    """ベッドの頭を上げるタスク
    """
    def node_name(self):
        return "subtask_bed_head_up"
    
    def id(self):
        return 9102

    async def service_callback(self, request, response, goal_handle):
        ws = create_connection(BED_WS_ADDRESS)  # open socket
        ws.send(HEAD_UP + f",{request['sec']}")  # send to socket
        response.message = ws.recv()
        self.get_logger().error(response.message)
        ws.close()  # close socket
        return response   

    # async def service_callback(self, request, response, goal_handle):
    #     ws = create_connection(BED_WS_ADDRESS)  # open socket
    #     ws.send(HEAD_UP + f",{request['sec']}")  # send to socket
    #     ws.close()  # close socket
    #     TIME = request['sec']

    #     time.sleep(TIME)
    #     # event = Event()
    #     # while not event.wait(timeout=TIME):
    #     #     if goal_handle.is_cancel_requested:
    #     #         response.message = "Canceled"
    #     #         return response
    #     response.message = "Success"
    #     return response

    def init_argument(self):
        return {"sec": 20.0}

    async def cancel_service_callback(self):
        """@override
        """
        print("Cancel subtask_bed_link_up")
        ws = create_connection(BED_WS_ADDRESS)  # open socket
        ws.send("0,0.0")  # send to socket
        ws.close()  # close socket

class SubtaskBedHeadDown(SubtaskNodeBase):
    """ベッドの頭を下げるタスク
    """
    def node_name(self):
        return "subtask_bed_head_down"
    
    def id(self):
        return 9103

    async def service_callback(self, request, response, goal_handle):
        ws = create_connection(BED_WS_ADDRESS)  # open socket
        ws.send(HEAD_DOWN + f",{request['sec']}")  # send to socket
        response.message = ws.recv()
        self.get_logger().error(response.message)
        ws.close()  # close socket
        return response   
             
    # async def service_callback(self, request, response, goal_handle):
    #     ws = create_connection(BED_WS_ADDRESS)  # open socket
    #     ws.send(HEAD_DOWN + f",{request['sec']}")  # send to socket
    #     ws.close()  # close socket
    #     TIME = request['sec']

    #     time.sleep(TIME)
    #     # event = Event()
    #     # while not event.wait(timeout=TIME):
    #     #     if goal_handle.is_cancel_requested:
    #     #         response.message = "Canceled"
    #     #         return response
    #     response.message = "Success"
    #     return response

    def init_argument(self):
        return {"sec": 20.0}

    async def cancel_service_callback(self):
        """@override
        """
        print("Cancel subtask_bed_link_up")
        ws = create_connection(BED_WS_ADDRESS)  # open socket
        ws.send("0,0.0")  # send to socket
        ws.close()  # close socket

class SubtaskBedFootUp(SubtaskNodeBase):
    """ベッドの足を上げるタスク
    """
    def node_name(self):
        return "subtask_bed_foot_up"
    
    def id(self):
        return 9104

    async def service_callback(self, request, response, goal_handle):
        ws = create_connection(BED_WS_ADDRESS)  # open socket
        ws.send(FOOT_UP + f",{request['sec']}")  # send to socket
        response.message = ws.recv()
        self.get_logger().error(response.message)
        ws.close()  # close socket
        return response   
             
    # async def service_callback(self, request, response, goal_handle):
    #     ws = create_connection(BED_WS_ADDRESS)  # open socket
    #     ws.send(FOOT_UP + f",{request['sec']}")  # send to socket
    #     ws.close()  # close socket
    #     TIME = request['sec']

    #     time.sleep(TIME)
    #     # event = Event()
    #     # while not event.wait(timeout=TIME):
    #     #     if goal_handle.is_cancel_requested:
    #     #         response.message = "Canceled"
    #     #         return response
    #     response.message = "Success"
    #     return response

    def init_argument(self):
        return {"sec": 20.0}

    async def cancel_service_callback(self):
        """@override
        """
        print("Cancel subtask_bed_link_up")
        ws = create_connection(BED_WS_ADDRESS)  # open socket
        ws.send("0,0.0")  # send to socket
        ws.close()  # close socket

class SubtaskBedFootDown(SubtaskNodeBase):
    """ベッドの足を下げるタスク
    """
    def node_name(self):
        return "subtask_bed_foot_down"
    
    def id(self):
        return 9105

    async def service_callback(self, request, response, goal_handle):
        ws = create_connection(BED_WS_ADDRESS)  # open socket
        ws.send(FOOT_DOWN + f",{request['sec']}")  # send to socket
        response.message = ws.recv()
        self.get_logger().error(response.message)
        ws.close()  # close socket
        return response   
             
    # async def service_callback(self, request, response, goal_handle):
    #     ws = create_connection(BED_WS_ADDRESS)  # open socket
    #     ws.send(FOOT_DOWN + f",{request['sec']}")  # send to socket
    #     ws.close()  # close socket
    #     TIME = request['sec']

    #     time.sleep(TIME)
    #     # event = Event()
    #     # while not event.wait(timeout=TIME):
    #     #     if goal_handle.is_cancel_requested:
    #     #         response.message = "Canceled"
    #     #         return response
    #     response.message = "Success"
    #     return response

    def init_argument(self):
        return {"sec": 20.0}

    async def cancel_service_callback(self):
        """@override
        """
        print("Cancel subtask_bed_link_up")
        ws = create_connection(BED_WS_ADDRESS)  # open socket
        ws.send("0,0.0")  # send to socket
        ws.close()  # close socket

class SubtaskBedHeightUp(SubtaskNodeBase):
    """ベッドの高さを上げるタスク
    """
    def node_name(self):
        return "subtask_bed_height_up"
    
    def id(self):
        return 9106

    async def service_callback(self, request, response, goal_handle):
        ws = create_connection(BED_WS_ADDRESS)  # open socket
        ws.send(HEIGHT_UP + f",{request['sec']}")  # send to socket
        response.message = ws.recv()
        self.get_logger().error(response.message)
        ws.close()  # close socket
        return response   
             
    # async def service_callback(self, request, response, goal_handle):
    #     ws = create_connection(BED_WS_ADDRESS)  # open socket
    #     ws.send(HEIGHT_UP + f",{request['sec']}")  # send to socket
    #     ws.close()  # close socket
    #     TIME = request['sec']

    #     time.sleep(TIME)
    #     # event = Event()
    #     # while not event.wait(timeout=TIME):
    #     #     if goal_handle.is_cancel_requested:
    #     #         response.message = "Canceled"
    #     #         return response
    #     response.message = "Success"
    #     return response

    def init_argument(self):
        return {"sec": 20.0}

    async def cancel_service_callback(self):
        """@override
        """
        print("Cancel subtask_bed_link_up")
        ws = create_connection(BED_WS_ADDRESS)  # open socket
        ws.send("0,0.0")  # send to socket
        ws.close()  # close socket

class SubtaskBedHeightDown(SubtaskNodeBase):
    """ベッドの高さを下げるタスク
    """
    def node_name(self):
        return "subtask_bed_height_down"
    
    def id(self):
        return 9107

    async def service_callback(self, request, response, goal_handle):
        ws = create_connection(BED_WS_ADDRESS)  # open socket
        ws.send(HEIGHT_DOWN + f",{request['sec']}")  # send to socket
        response.message = ws.recv()
        self.get_logger().error(response.message)
        ws.close()  # close socket
        return response   
             
    # async def service_callback(self, request, response, goal_handle):
    #     ws = create_connection(BED_WS_ADDRESS)  # open socket
    #     ws.send(HEIGHT_DOWN + f",{request['sec']}")  # send to socket
    #     ws.close()  # close socket
    #     TIME = request['sec']

    #     time.sleep(TIME)
    #     # event = Event()
    #     # while not event.wait(timeout=TIME):
    #     #     if goal_handle.is_cancel_requested:
    #     #         response.message = "Canceled"
    #     #         return response
    #     response.message = "Success"
    #     return response

    def init_argument(self):
        return {"sec": 20.0}

    async def cancel_service_callback(self):
        """@override
        """
        print("Cancel subtask_bed_link_up")
        ws = create_connection(BED_WS_ADDRESS)  # open socket
        ws.send("0,0.0")  # send to socket
        ws.close()  # close socket