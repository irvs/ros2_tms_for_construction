import rclpy
from tms_msg_ur.msg import JuliusMsg
from tms_msg_ur.srv import GSpeechSrv
from std_msgs.msg import Bool
from std_msgs.msg import String
from std_msgs.msg import Empty
import os
import sys
import socket
import requests
import re
import subprocess
import shlex
import time
import json
import base64
import threading


julius_path = '/usr/local/bin/julius'
jconf_path = '/home/ubuntu/ros2_ws/src/ros2_tms/tms_ur/tms_ur_listener_project/julius_dictation_kit/tms.jconf'
julius = None
julius_socket = None
adinrec_path = '/usr/local/bin/adinrec'
wav_file = '/home/ubuntu/ros2_ws/src/ros2_tms/tms_ur/tms_ur_listener_project/wav/rec.wav'
gs_filename = 'gs://ros-tms/rec.wav'

def invoke_julius():
    print('INFO : invoke julius')
    args = julius_path + ' -C ' + jconf_path + ' -module '
    p = subprocess.Popen(
            shlex.split(args),
            stdin=None,
            stdout=None,
            stderr=None
        )
    print('INFO : invoke julius complete.')
    print('INFO : wait 2 seconds.')
    time.sleep(3.0)
    print('INFO : invoke julius complete')
    return p


def kill_julius(julius):
    print('INFO : terminate julius')
    julius.kill()
    while julius.poll() is None:
        print('INFO : wait for 0.1 sec julius\' termination')
        time.sleep(0.1)
    print('INFO : terminate julius complete')


def get_OS_PID(process):
    psef = 'ps -ef | grep ' + process + ' | grep -ve grep -vie python |head -1|awk \'{print($2)}\''
    if sys.version_info.major == 3:
        PID = str(subprocess.check_output(psef, shell=True), encoding='utf-8').rstrip ()
    else:
        PID = subprocess.check_output(psef, shell=True).rstrip ()
    return PID


def create_socket():
    print('INFO : create a socket to connect julius')
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.connect(('localhost', 10500))
    print('INFO : create a socket to connect julius complete')
    return s


def delete_socket(s):
    print('INFO : delete a socket')
    s.close()
    print('INFO : delete a socket complete')
    return True


def invoke_julius_set():
    julius = invoke_julius()
    julius_socket = create_socket()
    sf = julius_socket.makefile('rb')
    return (julius, julius_socket, sf)


def power_callback(data):
    global julius,julius_socket,sf
    if data.data == True:
        node.get_logger().info("invoke julius")
        julius, julius_socket, sf = invoke_julius_set()
    else:
        node.get_logger().info("DIE julius")
        kill_julius(julius)
        delete_socket(julius_socket)

def gSpeech_callback(request, response):
    print("listen command")
    args = adinrec_path + ' ' + wav_file
    ret = subprocess.check_output(shlex.split(args))
    speak = String()
    speak.data = "\sound2"
    speaker_pub.publish(speak)
    print("recording succeed")

    file = open(wav_file,'rb').read()
    enc = base64.b64encode(file)
    print("enc success")

    data = '{"config":{"encoding":"LINEAR16","sampleRateHertz":16000,"languageCode":"ja-JP"},"audio":{"content":"' + enc.decode('utf-8') + '"}}'
    json_file = open('/home/ubuntu/ros2_ws/src/ros2_tms/tms_ur/tms_ur_listener_project/json/sync-request.json','w')
    json_file.write(data)
    json_file.close()
    print("write json_file")

    args = 'curl -s -k -H "Content-Type: application/json" -H "Authorization: Bearer '+token.decode().rstrip('\n')+'" https://speech.googleapis.com/v1/speech:recognize -d @/home/ubuntu/ros2_ws/src/ros2_tms/tms_ur/tms_ur_listener_project/json/sync-request.json'
    print(args)
    ret = subprocess.check_output(shlex.split(args))
    ret_str = ret.decode()
    print(ret_str)
    json_dict = json.loads(ret_str)
    if "results" in json_dict:
        script = json_dict["results"][0]["alternatives"][0]["transcript"]
        val = float(json_dict["results"][0]["alternatives"][0]["confidence"])
    else:
        script = ""
        val = 0.0

    response.data = script
    response.value = val
    return response

    # msg = julius_msg()
    # msg.data = script
    # msg.value = val
    # pub.publish(msg)

def get_token():
    global token
    args = 'gcloud auth print-access-token'
    token = subprocess.check_output(shlex.split(args))
    print(token)
    t=threading.Timer(600,get_token)
    t.setDaemon(True)
    t.start()

def main(args=None):
    rclpy.init(args=args)

    # REVIEW: anonymous=Trueにどうやってするのかわからない
    global node
    node= rclpy.create_node('tms_ir_listener_client' )

    global speaker_pub, pub
    speaker_pub = node.create_publisher(String, "/speaker", 10)
    pub = node.create_publisher(JuliusMsg, "julius_msg", 10)

    node.create_subscription(Bool, "/julius_power", power_callback)
    node.create_service(GSpeechSrv, "gSpeech", gSpeech_callback)

    global julius,julius_socket,sf
    julius, julius_socket, sf = invoke_julius_set()

    reResult = re.compile(u'WHYPO WORD="(\S*)" .* CM="(\d\.\d*)"')

    t=threading.Thread(target=get_token)
    t.setDaemon(True)
    t.start()

    def timer_callback():
        if julius.poll() is None:
            try:
                line = sf.readline().decode('utf-8')
            except socket.timeout:
                line = ""
                return 
            print(line)
            tmp = reResult.search(line)
            if tmp:
                print(tmp.group(1))
                msg = JuliusMsg()
                msg.data = tmp.group(1)
                msg.value = float(tmp.group(2))
                pub.publish(msg)
    
    timer_period = 0.01  # 100Hz  # rospy.Rate(100)
    timer = node.create_timer(timer_period, timer_callback)

    rclpy.spin(node)

    node.get_logger().info("exit")
    kill_julius(julius)
    delete_socket(julius_socket)
    node.destoy_timer(timer)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()