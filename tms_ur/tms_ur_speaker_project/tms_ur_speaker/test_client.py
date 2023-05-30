from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from tms_msg_ur.srv import SpeakerSrv
from tms_msg_ur.srv import SpeakerWavSrv
import rclpy
from std_msgs.msg import String

import wave

class TmsUrSpeakerTestClient(Node):

    def __init__(self):
        super().__init__('tms_ur_speaker_test_client')
        self.timer = self.create_timer(0, self.timer_callback, ReentrantCallbackGroup())
    
    async def timer_callback(self):
        self.timer.cancel()
        # await self.announce_test()
        await self.wav_test()
        
    async def announce_test(self):
        self.cli = self.create_client(SpeakerSrv, 'speaker_srv', callback_group=ReentrantCallbackGroup())
        while not self.cli.wait_for_service(1.0):
            self.get_logger().info('service "speaker_srv" not found...')
        
        self.get_logger().info('send request')
        self.req = SpeakerSrv.Request()
        self.req.data = "おはようございます。今日は過ごしやすい日です。"
        await self.cli.call_async(self.req)
        self.get_logger().info('end service')

    async def wav_test(self):
        self.cli = self.create_client(SpeakerWavSrv, 'speaker_wav_srv', callback_group=ReentrantCallbackGroup())
        while  not self.cli.wait_for_service(1.0):
            self.get_logger().info('service "speaker_wav_srv" not found...')
        
        data = b''
        with open('./wav/google_output.wav',mode='rb') as f:
            data = f.read()

        self.get_logger().info('send request')
        self.req = SpeakerWavSrv.Request()

        datas = list(bytearray(data))
        for d in datas:
            self.req.data.append(d.to_bytes(1,"little"))
        # self.req.data = [b'0',b'1',b'1']
        
        await self.cli.call_async(self.req)
        self.get_logger().info('end service')


def main(args=None):
    rclpy.init(args=args)

    tms_ur_speaker_test_client = TmsUrSpeakerTestClient()
    rclpy.spin(tms_ur_speaker_test_client)

    tms_ur_speaker_test_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()