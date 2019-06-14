#!/usr/bin/python
# -*- coding: utf-8 -*-

### This is ROS2 Version of tms_ur_listener_server.py 

from rclpy.node import Node
from tms_msg_ur.msg import JuliusMsg
from tms_msg_ur.srv import GSpeechSrv, SlackSrv, SpeakerSrv
from std_msgs.msg import Bool
from std_msgs.msg import String
from std_msgs.msg import Int32
from janome.tokenizer import Tokenizer
from tms_msg_db.msg import Tmsdb
from tms_msg_db.srv import TmsdbGetData
from tms_msg_ts.srv import TsReq
from tms_rc_double.srv import SkypeSrv
import rclpy
import requests
import time
import subprocess
import shlex
import json
import datetime
import threading
import urllib
import os
host_url = os.getenv("ROS_HOSTNAME", "localhost")


trigger = ['ROS-TMS','LOOMO','TARO']
error_msg0 = "すみません。聞き取れませんでした。"
error_msg1 = "すみません。よくわかりませんでした。"
error_msg2 = "エラーが発生したため、処理を中断します"
sid = 100000


class TmsUrListener(Node):
    def __init__(self):
        super().__init__('tms_ur_listener')
        self.create_subscription(JuliusMsg,"/julius_msg", lambda msg: self.callback(msg,0))
        self.create_subscription(JuliusMsg,"/pi1/julius_msg", lambda msg: self.callback(msg,1))
        self.create_subscription(JuliusMsg,"/pi2/julius_msg", lambda msg: self.callback(msg,2))
        self.create_subscription(JuliusMsg,"/pi3/julius_msg", lambda msg: self.callback(msg,3))
        self.create_subscription(JuliusMsg,"/pi4/julius_msg", lambda msg: self.callback(msg,4))
        self.create_subscription(JuliusMsg,"/pi5/julius_msg", lambda msg: self.callback(msg,5))
        self.create_subscription(String,"/watch_msg", lambda msg: self.callback(msg,100))
        self.create_subscription(String,"/line_msg", lambda msg: self.callback(msg,200))
        self.create_subscription(String,"/slack_msg", lambda msg: self.callback(msg,201))
        self.gSpeech_launched = False
        self.julius_flag = True
        self.timer = threading.Timer(1,self.alarm)

        self.power_pub = self.create_publisher(Bool, "julius_power", 10)
        self.speaker_pub = self.create_publisher(String, "speaker", 10)
        self.bed_pub = self.create_publisher(Int32, "rc_bed", 10)
        self.tok = Tokenizer()

        self.get_logger().info('tms_ur_listener_server ready...')
        print('tms_ur_listener_server ready...')
    

    def destroy_node(self):
        self.get_logger().info("Stopping the node")
        super().destroy_node()

    def alarm(self):
        while True:
            print("alarm")
            self.speaker('\sound4')
            time.sleep(1.5)
            temp_dbdata = Tmsdb()
            temp_dbdata.id = 1100
            temp_dbdata.state = 1
            target = self.db_reader(temp_dbdata)
            if target is None:
                self.announce(error_msg2)
                return
            print('rp:'+str(target.rp))
            if target.rp > -0.2:
                break

    def julius_power(self, data, t=0):
        if self.julius_flag != data:
            msg = Bool()
            msg.data = data
            time.sleep(float(t))
            self.power_pub.publish(msg)
            self.julius_flag = data
            if data == True:
                time.sleep(1.5)
                self.speaker('\sound3')
    
    def launch_gSpeech(self, id):
        # servicename = '/pi' + str(id) + '/gSpeech'
        ##################################################### DEBUG
        servicename = '/gSpeech'
        ###########################################################
        gspeech = self.create_client(GSpeechSrv, servicename)
        gspeech_req = GSpeechSrv.Request()

        while not gspeech.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service "/gSpeech"  not available, waiting again...')
        
        self.future_gspeech  = gspeech.call_async(gspeech_req)
        rclpy.spin_until_future_complete(self,self.future_gspeech)
        if self.future_gspeech.result() is not None:
            response = self.future_gspeech.result()
            print(response)
            return response
        else:
            self.get_logger().info('Service call failed %r' % (self.future_gspeech.exception(),))

    def speaker(self,data):
        speak = String()
        speak.data = data
        self.speaker_pub.publish(speak)

    def announce(self,data):
        print(data)
        #rospy.wait_for_service('speaker_srv', timeout=1.0)
        tim = 0.0

        speak = self.create_client(SpeakerSrv, 'speaker_srv')
        speak_req = SpeakerSrv.Request()
        speak_req.data = data

        while not speak.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service "speaker_srv" not available, waiting again...')

        self.future_speak = speak.call_async(speak_req)
        rclpy.spin_until_future_complete(self, self.future_speak) 

        if self.future_speak.result() is not None:
            tim = self.future_speak.result().sec
        else:
            send_slack = self.create_client(SlackSrv,'slack_srv')
            slack_req = SlackSrv.Request()
            slack_req.data = data
            self.future_slack = send_slack.call_async(slack_req)
            if self.future_slack.result() is not None:
                tim = self.future_slack.result().sec
            else:
                self.get_logger().info('Service call failed %r' % (self.future_slack.exception(),))
        return tim

    def db_reader(self,data):
        tms_db_reader = self.create_client(TmsdbGetData, 'tms_db_reader')
        db_reader_req = TmsdbGetData.Request()
        db_reader_req.tmsdb = data
        
        while not tms_db_reader.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('tms_db_reader not available, waiting again...')
        
        self.future_db_reader  = tms_db_reader.call_async(db_reader_req)
        rclpy.spin_until_future_complete(self,self.future_db_reader)
        if self.future_db_reader.result() is not None:
            res = self.future_db_reader.result().tmsdb
            return res
        else:
            self.get_logger().info('Service call failed %r' % (self.future_db_reader.exception(),))
            return None

    def tag_reader(self,data):
        temp_dbdata = Tmsdb()
        temp_dbdata.tag='「'+data+'」'
        target = self.db_reader(temp_dbdata)
        return target


    def ask_remote(self, words, command = "robot_task", talk = False):
        payload ={
            "words":words
        }
        tms_master = "192.168.4.80"
        ret = requests.post("http://" + tms_master + ":4000/rms_svr",json = payload)
        remote_tms = ret.json()
        print(remote_tms)
        if remote_tms["message"] == "OK":
            payload = {
                "url": remote_tms["hostname"],
                "room": remote_tms["name"],
                "command": command,
            }
            if command == "robot_task":
                payload["service"] = "tms_ts_master"
                payload["service_type"] = "tms_msg_ts/ts_req"

            ret = requests.post("http://" + host_url + ":5000/rp",json = payload)
            ret_dict = ret.json()
            if ret_dict["message"] == "OK":
                if talk:
                    try:
                        skype_client = rospy.ServiceProxy('skype_server',skype_srv)
                        res = skype_client(remote_tms["skype_id"])
                    except:
                        print(res)
                tim = self.announce(ret_dict["announce"])
                self.julius_power(True,tim.sec)
                return True
            else:
                print(ret_dict["message"])
                tim = self.announce(error_msg1)
                self.julius_power(True,tim.sec)
                return False
            
        else:
            tim = self.announce(error_msg1)
            self.julius_power(True,tim.sec)
            return False

            print(ret.json())

            return True

    def callback(self,data,id):
        self.get_logger().info(str(id))
        self.get_logger().info(data.data)
        print(str(id))
        print(data.data)
        if id < 100:
            if data.data not in trigger:
                return
            if self.gSpeech_launched == True:
                return
            self.gSpeech_launched = True
            self.get_logger().info("call trigger on raspi:%d" % id)
            self.get_logger().info("kill julius!!")
            self.julius_power(False)
            self.speaker("\sound1")
            time.sleep(0.5)
            self.announce(error_msg0)
            time.sleep(0.5)
            data = self.launch_gSpeech(id)
            self.gSpeech_launched = False

        if data.data == "":
            tim = self.announce(error_msg0)
            self.julius_power(True,tim.sec)
            return
        self.get_logger().info("get command!")
        tokens = self.tok.tokenize(data.data.decode('utf-8'))
        words = []
        verb = ''
        for token in tokens:
            print(token)
            if token.part_of_speech.split(',')[0] == u'動詞':
                verb += token.base_form.encode('utf-8')
            elif token.part_of_speech.split(',')[0] == u'名詞':
                if token.base_form.encode('utf-8') != "*":
                    words.append(token.base_form.encode('utf-8'))
                else:
                    words.append(token.surface.encode('utf-8'))
        if verb != '':
            words.append(verb)
        if "言う" in words: #「〇〇に行って」が「〇〇に言って」と認識される
            words.append("行く")
        if "入る" in words: #同上
            words.append("行く")
        
        print(str(words).decode('string-escape'))

        task_id = 0
        robot_id = 0
        object_id = 0
        user_id = 0#1100
        place_id = 0
        announce = ""
        task_name = "" #for remote task
        robot_name = ""
        object_name = ""
        user_name = ""#"太郎さん"
        place_name = ""
        task_dic = {}
        robot_dic = {}
        object_dic = {}
        user_dic = {}#{1100:"太郎さん"}
        place_dic = {}
        other_words = []
        
        for word in words:
            res = self.tag_reader(word)
            if res is None:
                tim = self.announce(error_msg2)
                self.julius_power(True,tim.sec)
                return
            for target in res.tmsdb:
                if target.type == 'task':
                    task_dic[target.id] = target.announce
                elif target.type == 'robot':
                    robot_dic[target.id] = target.announce
                elif target.type == 'object':
                    object_dic[target.id] = target.announce
                elif target.type == 'person':
                    user_dic[target.id] = target.announce
                elif target.type == 'furniture':
                    place_dic[target.id] = target.announce
                else:
                    other_words.append(word)
    
        print("task:" + str(task_dic))
        print("robot:" + str(robot_dic))
        print("object:" + str(object_dic))
        print("user:" + str(user_dic))
        print("place:" + str(place_dic))

        if len(task_dic) == 1:
            task_id = task_dic.keys()[0]
            announce = task_dic[task_id]
        elif len(task_dic) > 1:
            print("len(task_dic) > 1")
            #「ベッド」がタスクとして認識されてしまい、「ベッドに行って」が失敗してしまう
            # => ロボットによるタスクを優先するため、task_id が小さい方を優先(要検討)
            task_id = min(task_dic.keys())
            announce = task_dic[task_id]

        if task_id == 0:
            print('ask docomo Q&A api')
            print(data.data)
            urlenc = urllib.quote(data.data)
            args = "curl -s 'https://api.apigw.smt.docomo.ne.jp/knowledgeQA/v1/ask?APIKEY=" + self.apikey + "&q=" + urlenc + "'"
            ret = subprocess.check_output(shlex.split(args))
            json_dict = json.loads(ret,"utf-8")
            announce = "すみません、わかりませんでした。"
            if "message" in json_dict:
                print(json_dict["message"]["textForDisplay"])
                announce = json_dict["message"]["textForSpeech"]
            tim = self.announce(announce)
            self.julius_power(True,tim.sec)
            return
    
        elif task_id == 8100: #search_object
            if len(object_dic) == 1:
                object_id = object_dic.keys()[0]
                object_name = object_dic[object_id]
            elif len(object_dic) > 1:
                print("len(object_dic) > 1")
                #未実装
            else:
                self.ask_remote(words, "search_object")
                return

            place_id = 0
            place_name = ""
            temp_dbdata = Tmsdb()
            temp_dbdata.id = object_id
            temp_dbdata.state = 1

            #target = self.db_reader(temp_dbdata)
            db_target = self.db_reader(temp_dbdata)
            target = db_target.tmsdb[0]
            if target is None:
                tim = self.announce(error_msg2)
                self.julius_power(True,tim.sec)
                return
            place_id = target.place

            temp_dbdata = Tmsdb()
            temp_dbdata.id = place_id + sid

            db_target =self.db_reader(temp_dbdata)
            target = db_target.tmsdb[0]
            #target = self.db_reader(temp_dbdata)
            if target is None:
                tim = self.announce(error_msg2)
                self.julius_power(True,tim.sec)
                return
            place_name = target.announce

            if object_name == "" or place_name == "":
                tim = self.announce(error_msg1)
                self.julius_power(True,tim.sec)
                return

            anc_list = announce.split("$")
            announce = ""
            for anc in anc_list:
                if anc == "object":
                    announce += object_name
                elif anc == "place":
                    announce += place_name
                else:
                    announce += anc
            tim = self.announce(announce)
            self.julius_power(True,tim.sec)
        elif task_id == 8101: #weather_forecast
            place = "福岡市"
            date = ""
            weather = ""
            for word in other_words:
                if word in ['今日','明日','明後日','あさって']:
                    date = word
            if date == "":
                tim = self.announce(error_msg1)
                self.julius_power(True,tim.sec)
                return

            args = "curl -s http://weather.livedoor.com/forecast/webservice/json/v1\?city\=400010"
            ret = subprocess.check_output(shlex.split(args))
            json_dict = json.loads(ret,"utf-8")
            if "forecasts" in json_dict:
                if date == '今日':
                    weather = json_dict["forecasts"][0]["telop"].encode('utf-8')
                elif date == '明日':
                    weather = json_dict["forecasts"][1]["telop"].encode('utf-8')
                elif date == '明後日' or date == 'あさって':
                    weather = json_dict["forecasts"][2]["telop"].encode('utf-8')
            if weather == "":
                tim = self.announce(error_msg1)
                self.julius_power(True,tim.sec)
                return

            anc_list = announce.split("$")
            announce = ""
            for anc in anc_list:
                if anc == "place":
                    announce += place
                elif anc == "date":
                    announce += date
                elif anc == "weather":
                    announce += weather
                else:
                    announce += anc
            tim = self.announce(announce)
            self.julius_power(True,tim.sec)

        elif task_id == 8102: #set_alarm
            today = datetime.datetime.today()
            print('now:' + today.strftime("%Y/%m/%d %H:%M:%S"))
            if today.hour < 6:
                date = 0
            else:
                date = 1
            hour = -1
            minute = 0
            for i,word in enumerate(other_words):
                if word == "今日":
                    date = 0
                elif word == "明日" and today.hour > 6:
                    date = 1
                elif word in ["時","時半"] and i>0:
                    if words[i-1].isdigit():
                        hour = int(words[i-1])
                        if word == "時半":
                            minute = 30
                        if i>1 and words[i-2] == "午後" and hour <=12:
                            hour += 12
                        elif i>1 and words[i-2] == "夜" and hour <=12 and hour>=6:
                            hour += 12
                elif word == "分":
                    if words[i-1].isdigit():
                        minute = int(words[i-1])
            print("d:"+str(date)+" h:"+str(hour)+" m:"+str(minute))
            if hour == -1:
                tim = self.announce(error_msg1)
                self.julius_power(True,tim.sec)
                return

            tgt_tim = datetime.datetime(today.year,today.month,today.day,hour,minute,0,0)
            tgt_time += datetime.timedelta(1)
            print('tgt_time:' + tgt_time.strftime("%Y/%m/%d %H:%M:%S"))
            offset = tgt_time - today
            print('offset_sec:' + str(offset.seconds))

            if offset.seconds < 0:
                tim = self.announce(error_msg1)
                self.julius_power(True,tim.sec)
                return

            self.timer = threading.Timer(15,self.alarm)#(offset.seconds,self.alarm)
            self.timer.start()

            anc_list = announce.split("$")
            announce = ""
            for anc in anc_list:
                if anc == "date":
                    announce += str(tgt_time.month)+"月"+str(tgt_time.day)+"日"
                elif anc == "time":
                    announce += str(tgt_time.hour)+"時"+str(tgt_time.minute)+"分"
                else:
                    announce += anc

            tim = self.announce(announce)
            self.julius_power(True,tim.sec)
        elif task_id == 8103:
            url = "http://192.168.100.101/codemari_kyudai/CodemariServlet?deviceID=9999&locale=ja&cmd=%251CFLP%"
            onoff = ""
            if "つける" in other_words:
                print("light on")
                onoff = "付け"
                url += "2003"
            elif "消す" in other_words:
                print("light off")
                onoff = "消し"
                url += "2005"
            else:
                tim = self.announce(error_msg1)
                self.julius_power(True,tim.sec)
                return

            anc_list = announce.split("$")
            announce = ""
            for anc in anc_list:
                if anc == "onoff":
                    announce += onoff
                else:
                    announce += anc
            tim = self.announce(announce)
            self.julius_power(True,tim.sec)

            res = requests.get(url)
            print(res.text)
        elif task_id == 8104:
            msg = Int32()
            cmd = ""
            if "起こす" in words:
                msg.data = 1
                cmd = "を起こし"
            elif "寝かせる" in words:
                msg.data = 2
                cmd = "を寝かせ"
            elif "立てる" in words:
                msg.data = 3
                cmd = "を立て"
            elif "倒す" in words:
                msg.data = 4
                cmd = "を倒し"
            elif "上げる" in words:
                msg.data = 7
                cmd = "の高さを上げ"
            elif "下げる" in words:
                msg.data = 8
                cmd = "の高さを下げ"
            else:
                tim = self.announce(error_msg1)
                self.julius_power(True,tim.sec)
                return
            anc_list = announce.split("$")
            announce = ""
            for anc in anc_list:
                if anc == "cmd":
                    announce += cmd
                else:
                    announce += anc
            tim = self.announce(announce)
            self.julius_power(True,tim.sec)
            self.bed_pub.publish(msg)

        elif task_id == 8105:
            print(user_dic)
            if len(user_dic) == 1:
                user_id = user_dic.keys()[0]
                user_name = user_dic[user_id]
            elif len(user_dic) > 1:
                print("len(user_dic) > 1")
                #未実装
            else:
                self.ask_remote(words, "get_health_condition")
                return

            place_id = 0
            place_name = ""
            temp_dbdata = Tmsdb()
            temp_dbdata.id = user_id
            temp_dbdata.state = 1

            #target = self.db_reader(temp_dbdata)
            db_target = self.db_reader(temp_dbdata)
            target = db_target.tmsdb[0]
            if target is None:
                tim = self.announce(error_msg2)
                self.julius_power(True,tim.sec)
                return

            note = json.loads(target.note)
            rate = note["rate"]
            
            anc_list = announce.split("$")
            announce = ""
            for anc in anc_list:
                if anc == "user":
                    announce += user_name
                elif anc == "data":
                    announce += str(rate)
                else:
                    announce += anc
            tim = self.announce(announce)
            self.julius_power(True,tim.sec)

        else: #robot_task
            if task_id ==8009:
                 talk = True
            else:
                talk = False
            task_announce_list = announce.split(";")
            for i in range(len(task_announce_list)):
                anc_list = task_announce_list[i].split("$")
                announce = ""
                task_flag = 0
                for anc in anc_list:
                    if anc == "robot":
                        if len(robot_dic) == 1:
                            robot_id = robot_dic.keys()[0]
                            robot_name = robot_dic[robot_id]
                        elif len(robot_dic) > 1:
                            print("len(robot_dic) > 1")
                            #未実装

                        if robot_id==0:
                            if i == len(task_announce_list) - 1:
                                self.ask_remote(words, "robot_task",talk)
                                return
                            else:
                                task_flag = 1
                        announce += robot_name
                    elif anc == "object":
                        if len(object_dic) == 1:
                            object_id = object_dic.keys()[0]
                            object_name = object_dic[object_id]
                        elif len(object_dic) > 1:
                            print("len(object_dic) > 1")
                            #未実装

                        if object_id==0:
                            if i == len(task_announce_list) - 1:
                                self.ask_remote(words, "robot_task",talk)
                                return
                            else:
                                task_flag = 1
                        announce += object_name
                    elif anc == "user":
                        if len(user_dic) == 1:
                            user_id = user_dic.keys()[0]
                            user_name = user_dic[user_id]
                        elif len(user_dic) > 1:
                            print("len(user_dic) > 1")
                            #未実装

                        if user_id==0:
                            if i == len(task_announce_list) - 1:
                                self.ask_remote(words, "robot_task",talk)
                                return
                            else:
                                task_flag = 1
                        announce += user_name
                    elif anc == "place":
                        if len(place_dic) == 1:
                            place_id = place_dic.keys()[0]
                            place_name = place_dic[place_id]
                        elif len(place_dic) > 1:
                            print("len(place_dic) > 1")
                            #未実装

                        if place_id==0:
                            if i == len(task_announce_list) - 1:
                                self.ask_remote(words, "robot_task",talk)
                                return
                            else:
                                task_flag = 1
                        announce += place_name
                    else:
                        announce += anc

                if task_flag == 1:
                    continue
                print('send command')
                try:
                    rospy.wait_for_service('tms_ts_master', timeout=1.0)
                except rospy.ROSException:
                    print("tms_ts_master is not work")

                try:
                    tms_ts_master = rospy.ServiceProxy('tms_ts_master',ts_req)
                    res = tms_ts_master(0,task_id,robot_id,object_id,user_id,place_id,0)
                    print(res)
                except rospy.ServiceException as e:
                    print("Service call failed: %s" % e)

                tim = self.announce(announce)
                self.julius_power(True,tim.sec)
                return


def main(args=None):
    rclpy.init(args=args)

    tms_ur_listener =TmsUrListener()

    rclpy.spin(tms_ur_listener)

    minimal_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
