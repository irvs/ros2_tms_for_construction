import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from tms_msg_ur.srv import SpeakerSrv
from tms_msg_ur.srv import SpeakerWavSrv
from std_msgs.msg import String
import subprocess
import os

import pyaudio
import wave


base = '~/ros2_ws/src/ros2_tms/tms_ur/tms_ur_speaker_project/wav'


def play_wave(filename):
    subprocess.run(['aplay', filename])
"""
    try:
        wf = wave.open(filename, "r")
    except FileNotFoundError:
        print("Error: " + filename)
        return 0
        
    p = pyaudio.PyAudio()
    stream = p.open(format=p.get_format_from_width(wf.getsampwidth()),
                    channels=wf.getnchannels(),
                    rate=wf.getframerate(),
                    output=True)

    chunk = 1024
    data = wf.readframes(chunk)
    while data != b'':
        stream.write(data)
        data = wf.readframes(chunk)
    stream.close()
    p.terminate()
"""


def jtalk(t):
    open_jtalk=['open_jtalk']
    mech=['-x','/var/lib/mecab/dic/open-jtalk/naist-jdic']
    htsvoice=['-m','/usr/share/hts-voice/mei/mei_normal.htsvoice']
    speed=['-r','1.0']
    quality=['-a','0.57']
    outwav=['-ow','./wav/open_jtalk.wav']
    cmd=open_jtalk+mech+htsvoice+speed+quality+outwav
    c = subprocess.Popen(cmd,stdin=subprocess.PIPE)
    c.stdin.write(t.encode())
    c.stdin.close()
    c.wait()
    play_wave('./wav/open_jtalk.wav')


def speak(data):
    if data == '':
        return 0
    elif data[0]=='\\':
        play_wave(os.path.join(data[1:]))
        return 0
    else:
        talk = data.replace(',','')
        jtalk(talk)
        return 0


def subscription_callback(msg):
    global g_node
    g_node.get_logger().info(
        'Speaker: "%s"' % msg.data
    )
    speak(msg.data)


def service_callback(request, response):
    global g_node
    g_node.get_logger().info(
        request.data
    )
    speak(request.data)
    return response

def wav_srv_callback(request, response):
    global g_node
    g_node.get_logger().info('incoming wav data')

    # print(request.data)
    with open('./wav/speaker_temp.wav', 'wb') as f:
        for d in request.data:
            f.write(d)
    
    play_wave('./wav/speaker_temp.wav')
    return response

def main(args=None):
    global g_node
    rclpy.init(args=args)

    g_node = rclpy.create_node('tms_ur_speaker')
    g_node.get_logger().info('start')

    subscription = g_node.create_subscription(String, 'speaker', subscription_callback, 10)
    subscription  # prevent unused variable warning

    srv = g_node.create_service(SpeakerSrv, 'speaker_srv', service_callback, callback_group=ReentrantCallbackGroup())
    wav_srv = g_node.create_service(SpeakerWavSrv, 'speaker_wav_srv', wav_srv_callback, callback_group=ReentrantCallbackGroup())

    while rclpy.ok():
        rclpy.spin_once(g_node)

    g_node.destroy_node()
    rclpy.shutdown()


if __name__=='__main__':
    main()
