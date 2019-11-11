import rclpy
from tms_msg_ur.srv import SpeakerSrv
from std_msgs.msg import String
import subprocess
import os


base = '~/ros2_ws/src/ros2_tms/tms_ur/tms_ur_speaker_project/wav'


def jtalk(t):
    open_jtalk=['open_jtalk']
    mech=['-x','/var/lib/mecab/dic/open-jtalk/naist-jdic']
    htsvoice=['-m','/usr/share/hts-voice/mei/mei_normal.htsvoice']
    speed=['-r','1.0']
    quality=['-a','0.57']
    outwav=['-ow','open_jtalk.wav']
    cmd=open_jtalk+mech+htsvoice+speed+quality+outwav
    c = subprocess.Popen(cmd,stdin=subprocess.PIPE)
    c.stdin.write(t.encode())
    c.stdin.close()
    c.wait()
    aplay = ['aplay','-q','open_jtalk.wav']
    wr = subprocess.Popen(aplay)


def speak(data):
    if data == '':
        return 0
    elif data[0]=='\\':
        aplay = ['aplay','-q',os.path.join(base, data[1:]+'.wav')]
        wr = subprocess.Popen(aplay)
        #soxi = ['soxi','-D',os.path.join(base, data[1:]+'.wav')]
        #ret = subprocess.check_output(soxi)
        print(ret)
        return ret
    else:
        talk = data.replace(',','')
        jtalk(talk)
        #soxi = ['soxi','-D',os.path.join(base, 'open_jtalk.wav')]
        #ret = subprocess.check_output(soxi)
        #print(ret)
        #return ret


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
    ret = speak(request.data)
    response.sec = float(ret)
    return response

def main(args=None):
    global g_node
    rclpy.init(args=args)

    g_node = rclpy.create_node('tms_ur_speaker')
    print('tms_ur_speaker : '+base)

    subscription = g_node.create_subscription(String, 'speaker', subscription_callback, 10)
    subscription  # prevent unused variable warning

    srv = g_node.create_service(SpeakerSrv, 'speaker_srv', service_callback)


    while rclpy.ok():
        rclpy.spin_once(g_node)

    g_node.destroy_node()
    rclpy.shutdown()


if __name__=='__main__':
    main()
