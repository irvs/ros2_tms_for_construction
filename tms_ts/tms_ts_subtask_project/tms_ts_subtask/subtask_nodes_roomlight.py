import rclpy
from tms_ts_subtask.subtask_node_base import SubtaskNodeBase
from rclpy.executors import MultiThreadedExecutor
import requests

# 九大Big Sensor Boxの電気をコントロールするip address
BSEN_URL = "http://192.168.100.101/codemari_kyudai/CodemariServlet?deviceID=9999&locale=ja&cmd=%251CFLP%"

def main(args=None):
    global executor
    rclpy.init(args=args)

    try:
        executor = MultiThreadedExecutor()
        executor.add_node(SubtaskRoomLightOn())
        executor.add_node(SubtaskRoomLightOff())
        try:
            executor.spin()
        finally:
            executor.shutdown()

    finally:
        rclpy.shutdown()

if __name__=='__main__':
    main()


class SubtaskRoomLightOn(SubtaskNodeBase):
    """電気をつけるタスク
    """
    def node_name(self):
        return "subtask_roomlight_on"
    
    def id(self):
        return 9200
    
    async def service_callback(self, request, response):
        print('light on!')
        res = requests.get(BSEN_URL + "2003")

        response.message = "Success"
        return response


class SubtaskRoomLightOff(SubtaskNodeBase):
    """電気を消すタスク
    """
    def node_name(self):
        return "subtask_roomlight_off"
    
    def id(self):
        return 9201
    
    async def service_callback(self, request, response):
        self.get_logger().info('light off!')
        res = requests.get(BSEN_URL + "2005")

        response.message = "Success"
        return response
