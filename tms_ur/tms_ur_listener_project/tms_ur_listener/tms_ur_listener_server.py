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
# from tms_rc_double.srv import SkypeSrv
from rclpy.callback_groups import ReentrantCallbackGroup
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
import asyncio
import pprint
from websocket import create_connection


# trigger = ['ROS-TMS','LOOMO','TARO']
trigger = ['ROS-TMS']
error_msg0 = "すみません。聞き取れませんでした。"
error_msg1 = "すみません。よくわかりませんでした。"
error_msg2 = "エラーが発生したため、処理を中断します"
sid = 100000


class TmsUrListenerServer(Node):
    def __init__(self):
        super().__init__('tms_ur_listener_server')

        self.gSpeech_launched = False
        self.julius_flag = True
        self.tok = Tokenizer()

        # Docomo Api Key
        self.apikey = ""
        with open('/home/ros2tms/apikey', 'r') as f:
            for line in f:
                self.apikey += line.replace('\n','')

        # Publishers
        self.pub_power = self.create_publisher(Bool, "julius_power", 10)
        self.pub_speaker = self.create_publisher(String, "speaker", 10)
        self.pub_bed = self.create_publisher(Int32, "rc_bed", 10)

        # Callbacks ( subscribers )
        self.cb_group = ReentrantCallbackGroup()
        # self.create_subscription(JuliusMsg,"/julius_msg", lambda msg: self.callback(msg,0), callback_group=self.cb_group)
        self.create_subscription(JuliusMsg,"/julius_msg", self.julius_callback, callback_group=self.cb_group)
        # self.create_subscription(JuliusMsg,"/pi1/julius_msg", lambda msg: self.callback(msg,1), callback_group=self.cb_group)
        # self.create_subscription(JuliusMsg,"/pi2/julius_msg", lambda msg: self.callback(msg,2), callback_group=self.cb_group)
        # self.create_subscription(JuliusMsg,"/pi3/julius_msg", lambda msg: self.callback(msg,3), callback_group=self.cb_group)
        # self.create_subscription(JuliusMsg,"/pi4/julius_msg", lambda msg: self.callback(msg,4), callback_group=self.cb_group)
        # self.create_subscription(JuliusMsg,"/pi5/julius_msg", lambda msg: self.callback(msg,5), callback_group=self.cb_group)
        # self.create_subscription(String,"/watch_msg", lambda msg: self.callback(msg,100), callback_group=self.cb_group)
        # self.create_subscription(String,"/line_msg", lambda msg: self.callback(msg,200), callback_group=self.cb_group)
        # self.create_subscription(String,"/slack_msg", lambda msg: self.callback(msg,201), callback_group=self.cb_group)

        # Callbacks ( clients )
        self.cli_gspeech = self.create_client(GSpeechSrv, '/gSpeech')  # servicename = '/pi' + str(id) + '/gSpeech'
        while not self.cli_gspeech.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service "/gSpeech"  not available, waiting again...')

        self.cli_speaker = self.create_client(SpeakerSrv, 'speaker_srv')
        while not self.cli_speaker.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service "speaker_srv" not available, waiting again...')

        self.cli_dbreader = self.create_client(TmsdbGetData, 'tms_db_reader')
        while not self.cli_dbreader.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service "tms_db_reader" not available, waiting again...')
        
        self.get_logger().info('tms_ur_listener_server ready...')


    ### 通信に使うメソッド群 ##############################
    def julius_power(self, data, t=0):
        """[tms_ur_speaker]juliusを起動する・終了する
        """
        if self.julius_flag != data:
            msg = Bool()
            msg.data = data
            time.sleep(float(t))
            self.pub_power.publish(msg)
            self.julius_flag = data
            if data == True:
                time.sleep(1.5)
                # self.speaker('\sound3')
    

    async def call_gspeech(self, id):
        """[tms_ur_listener_client] google speech apiを呼び出し、音声認識をする
        """
        req = GSpeechSrv.Request()
        self.future_gspeech  = self.cli_gspeech.call_async(req)

        await self.future_gspeech

        if self.future_gspeech.result() is not None:
            response = self.future_gspeech.result()
            return response
        else:
            self.get_logger().info('Service "/gSpeech" call failed %r' % (self.future_gspeech.exception(),))


    def speaker(self,data):
        """[tms_ur_speaker] speakerから音を出す(by topic)
        """
        speak = String()
        speak.data = data
        self.pub_speaker.publish(speak)


    async def call_speaker(self,data):
        """[tms_ur_speaker] speakerから音を出す(by service)
        """
        req = SpeakerSrv.Request()
        req.data = data
        self.future_speaker = self.cli_speaker.call_async(req)

        await self.future_speaker

        tim = 0.0
        if self.future_speaker.result() is not None:
            tim = self.future_speaker.result().sec
            print("announce:"+str(tim))
            return tim
        else:
            self.get_logger().info('Service "speaker_srv" call failed %r' % (self.future_gspeech.exception(),))
        

    async def call_dbreader(self,data):
        """[tms_db_reader] DBからデータを読み取る
        """
        req = TmsdbGetData.Request()
        req.tmsdb = data
        self.future_dbreader  = self.cli_dbreader.call_async(req)

        await self.future_dbreader

        if self.future_dbreader.result() is not None:
            res = self.future_dbreader.result().tmsdb
            return res
        else:
            self.get_logger().info('Service "tms_db_reader" call failed %r' % (self.future_dbreader.exception(),))


    async def tag_reader(self,data):
        """[tms_db_reader] DBからタグを読み取る
        """
        temp_dbdata = Tmsdb()
        temp_dbdata.tag='「'+data+'」'
        target = await self.call_dbreader(temp_dbdata)
        return target


    ### コールバック時の動作を表すメソッド群 ##############################
    def command_to_words(self, command):
        """command(string)の中の名詞、動詞を判別しそれらの集合を返す
        """
        tokens = self.tok.tokenize(command)  # janome.tokenizerで日本語解析
        words = []
        verb = ''
        for token in tokens:
            # print('token: ' + str(token))
            if token.part_of_speech.split(',')[0] == u'動詞':
                verb += token.base_form
            elif token.part_of_speech.split(',')[0] == u'名詞':
                if token.base_form != "*":
                    words.append(token.base_form)
                else:
                    words.append(token.surface)
        if verb != '':
            words.append(verb)
        if "言う" in words: #「〇〇に行って」が「〇〇に言って」と認識される
            words.append("行く")
        if "入る" in words: #同上
            words.append("行く")
        
        return words

    async def ask_to_docomo_api(self, command):
        """command(string)を単にDocomo API に投げ、会話する
        """
        announce = "すみません、現在会話はできません"
        # urlenc = urllib.parse.quote(command)
        # args = "curl -s 'https://api.apigw.smt.docomo.ne.jp/knowledgeQA/v1/ask?APIKEY=" + self.apikey + "&q=" + urlenc + "'"
        # ret = subprocess.check_output(shlex.split(args))
        # print(ret)
        # if ret != b'':  # json return
        #     json_dict = json.loads(ret)
        #     print(json_dict)

        #     if "message" in json_dict:  # docomo api return message
        #         print(json_dict["message"]["textForDisplay"])
        #         announce = json_dict["message"]["textForSpeech"]
        #     else :
        #         announce = "apiキーが違うようです"
        
        tim = await self.call_speaker(announce)
        self.julius_power(True,tim)
    
    async def ask_to_weather_api(self, words):
        """words(list<string>)の中に"今日"、"明日"、"明後日"、"あさって"が含まれていれば、
        福岡市の天気を伝える
        """
        place = "福岡市"
        date = ""
        weather = ""
        for word in words:
            if word in ['今日','明日','明後日','あさって']:
                date = word
        if date == "":
            tim = await self.call_speaker(error_msg1)
            self.julius_power(True,tim)
            return

        args = "curl -s http://weather.livedoor.com/forecast/webservice/json/v1\?city\=400010"
        ret = subprocess.check_output(shlex.split(args))
        json_dict = json.loads(ret)
        pprint.pprint(json_dict)
        if "forecasts" in json_dict:
            if date == '今日':
                weather = json_dict["forecasts"][0]["telop"]
            elif date == '明日':
                weather = json_dict["forecasts"][1]["telop"]
            elif date == '明後日' or date == 'あさって':
                weather = json_dict["forecasts"][2]["telop"]
        if weather == "":
            tim = await self.call_speaker("今日、明日、または明後日の福岡市の天気に対応しています"))
            self.julius_power(True,tim)
            return
        tim = await self.call_speaker(place + "の" +date + "の天気は" + weather + "です")
        self.julius_power(True,tim)
    

    async def control_roomlights(self, words):
        """words(list<string>)の中に"つける"、"消す"が含まれていれば
        部屋の電気をそのとおりにする
        """
        url = "http://192.168.100.101/codemari_kyudai/CodemariServlet?deviceID=9999&locale=ja&cmd=%251CFLP%"
        onoff = ""
        if "つける" in words:
            print("light on")
            onoff = "付け"
            url += "2003"
        elif "消す" in words:
            print("light off")
            onoff = "消し"
            url += "2005"
        else:
            tim = await self.call_speaker(error_msg1)
            self.julius_power(True,tim)
            return

        anc_list = announce.split("$")
        announce = ""
        for anc in anc_list:
            if anc == "onoff":
                announce += onoff
            else:
                announce += anc
        tim = await self.call_speaker(announce)
        self.julius_power(True,tim)

        res = requests.get(url)
            print(res.text)
    

    async def control_tms_rc_bed(self, words):
        """words(List<string>)の中に対応する単語が含まれていれば、
        ベッドをそのとおりに動かす
        """
        control_number = 0
        cmd = ""
        if "起こす" in words:
            control_number = 1
            cmd = "を起こし"
        elif "寝かせる" in words:
            control_number = 2
            cmd = "を寝かせ"
        elif "立てる" in words:
            control_number = 3
            cmd = "を立て"
        elif "倒す" in words:
            control_number = 4
            cmd = "を倒し"
        elif "上げる" in words:
            control_number = 7
            cmd = "の高さを上げ"
        elif "下げる" in words:
            control_number = 8
            cmd = "の高さを下げ"
        else:
            tim = await self.call_speaker(error_msg1)
            self.julius_power(True,tim)
            return
        anc_list = announce.split("$")
        announce = ""
        for anc in anc_list:
            if anc == "cmd":
                announce += cmd
            else:
                announce += anc
        tim = await self.call_speaker(announce)
        self.julius_power(True,tim)
        ws = create_connection('ws://192.168.4.131:9989')  # open socket
        ws.send(str(control_number))  # send to socket
        ws.close()  # close socket


    ### メインコールバック ##############################
    async def julius_callback(self, julius_data,id=0):
        """[tms_ur_listener_client] マイクに向かって何か発音したときのコールバック関数
        """
        self.get_logger().info(julius_data.data)

        ### 1. 「ROS-TMS」とマイクに向かって発音していたら、子機上でGoogle Speech APIを立ち上げる
        if julius_data.data not in trigger:  # ROS-TMSと発音してなかったら終了
            return
        if self.gSpeech_launched == True:  # すでに端末がGSpeechを起動していたら終了
            return
        self.gSpeech_launched = True
        self.get_logger().info("[tms_ur_listener_client] << kill julius.")
        self.julius_power(False)
        self.speaker("\sound1")  # 起動音
        time.sleep(0.5)
        gspeech_data = await self.call_gspeech(id)
        self.gSpeech_launched = False

        ### 2-1. Google Speech APIが認識に失敗した場合、失敗音を出して終了
        if gspeech_data.data == "": 
            tim = await self.call_speaker(error_msg0)
            self.julius_power(True,tim)
            return
        
        ### 2-2. Google Speech APIが認識に成功した場合、単語ごとに分解しwordsに格納
        self.get_logger().info("[tms_ur_listener_client] >> get command.")
        words = self.command_to_words(gspeech_data.data)
        print(f'words: {words}')
        
        ### 3. wordsのwordをタグとしてDBを検索し、各辞書に格納
        task_dic = {}  # type == "task"
        robot_dic = {}  # type == "robot"
        object_dic = {}  # type == "object"
        person_dic = {}  # type == "person"
        furniture_dic = {}  # type == "furniture"
        other_words = {}  # type != ALL_TYPES
        for word in words:
            responses = await self.tag_reader(word)
            for target in responses:
                print(target)
                if target.type == 'task':
                    if target.id == 8102:  # 「起こす」でタイマーが起動してしまい、ベッドを起こしてが聞かない
                        continue
                    task_dic[target.id] = target.announce
                elif target.type == 'robot':
                    robot_dic[target.id] = target.announce
                elif target.type == 'object':
                    object_dic[target.id] = target.announce
                elif target.type == 'person':
                    person_dic[target.id] = target.announce
                elif target.type == 'furniture':
                    furniture_dic[target.id] = target.announce
                else:
                    other_words.append(word)

        print("task:" + str(task_dic))
        print("robot:" + str(robot_dic))
        print("object:" + str(object_dic))
        print("person:" + str(person_dic))
        print("place:" + str(furniture_dic))

        ### 4. id == "task"のものがあった場合,task_idを決定
        task_id = 0
        if len(task_dic) == 1:
            task_id = list(task_dic.keys())[0]
            announce = task_dic[task_id]
        elif len(task_dic) > 1:
            print("len(task_dic) > 1")
            #「ベッド」がタスクとして認識されてしまい、「ベッドに行って」が失敗してしまう
            # => ロボットによるタスクを優先するため、task_id が小さい方を優先(要検討)
            task_id = min(task_dic.keys())
            announce = task_dic[task_id]

        print("task_id : " + str(task_id))

        # task_id = 8101 ##################################DEBUG####################
        for word in words:
            other_words.append(word)

        if task_id == 0:
            print('ask docomo Q&A api')
            print(data.data)

            announce = "すみません、よくわかりませんでした"
            urlenc = urllib.parse.quote(data.data)
            args = "curl -s 'https://api.apigw.smt.docomo.ne.jp/knowledgeQA/v1/ask?APIKEY=" + self.apikey + "&q=" + urlenc + "'"
            ret = subprocess.check_output(shlex.split(args))
            print(ret)
            if ret != b'':  # json return
                json_dict = json.loads(ret)
                print(json_dict)

                if "message" in json_dict:  # docomo api return message
                    print(json_dict["message"]["textForDisplay"])
                    announce = json_dict["message"]["textForSpeech"]
                else :
                    announce = "apiキーが違うようです"
            
            tim = await self.call_speaker(announce)
            self.julius_power(True,tim)
            return
        # 「今日の天気」、「明日の天気」、「明後日の天気」----------
        elif task_id == 8101:
            await self.ask_to_weather_api(words)
        # アラーム ------------
        elif task_id == 8102:
            pass  # 未実装
            self.julius_power(True)
        # 電気をつけて / 消して ----------
        elif task_id == 8103: 
            await self.control_roomlights(words)
        # tms_rc_bed ベッドを起こして / 寝かせて ----------
        elif task_id == 8104:
            await self.control_tms_rc_bed(words)
        # 物体探索「どこ」、「ある」 ----------
        elif task_id == 8100:
            if len(object_dic) == 1:
                object_id = object_dic.keys()[0]
                object_name = object_dic[object_id]
            elif len(object_dic) > 1:
                print("len(object_dic) > 1")
                #未実装
            else:
                # self.ask_remote(words, "search_object")
                return

            place_id = 0
            place_name = ""
            temp_dbdata = Tmsdb()
            temp_dbdata.id = object_id
            temp_dbdata.state = 1

            #target = await self.call_dbreader(temp_dbdata)
            db_target = await self.call_dbreader(temp_dbdata)
            target = db_target[0]
            if target is None:
                tim = await self.call_speaker(error_msg2)
                self.julius_power(True,tim)
                return
            place_id = target.place

            temp_dbdata = Tmsdb()
            temp_dbdata.id = place_id + sid

            db_target =await self.call_dbreader(temp_dbdata)
            target = db_target[0]
            #target = await self.call_dbreader(temp_dbdata)
            if target is None:
                tim = await self.call_speaker(error_msg2)
                self.julius_power(True,tim)
                return
            place_name = target.announce

            if object_name == "" or place_name == "":
                tim = await self.call_speaker(error_msg1)
                self.julius_power(True,tim)
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
            tim = await self.call_speaker(announce)
            self.julius_power(True,tim)
        elif task_id == 8101: #weather_forecast
            place = "福岡市"
            date = ""
            weather = ""
            for word in other_words:
                if word in ['今日','明日','明後日','あさって']:
                    date = word
            if date == "":
                tim = await self.call_speaker(error_msg1)
                self.julius_power(True,tim)
                return

            args = "curl -s http://weather.livedoor.com/forecast/webservice/json/v1\?city\=400010"
            ret = subprocess.check_output(shlex.split(args))
            json_dict = json.loads(ret)
            pprint.pprint(json_dict)
            if "forecasts" in json_dict:
                if date == '今日':
                    weather = json_dict["forecasts"][0]["telop"]
                elif date == '明日':
                    weather = json_dict["forecasts"][1]["telop"]
                elif date == '明後日' or date == 'あさって':
                    weather = json_dict["forecasts"][2]["telop"]
            if weather == "":
                tim = await self.call_speaker(error_msg1)
                self.julius_power(True, 5.0)  #tim)
                return

            # anc_list = announce.split("$")
            # announce = ""
            # for anc in anc_list:
            #     if anc == "place":
            #         announce += place
            #     elif anc == "date":
            #         announce += date
            #     elif anc == "weather":
            #         announce += weather
            #     else:
            #         announce += anc
            # tim = await self.call_speaker(announce)
            tim = await self.call_speaker(place + "の" +date + "の天気は" + weather + "です")
            self.julius_power(True,tim)
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
                self.julius_power(True,tim)
                return

            tgt_tim = datetime.datetime(today.year,today.month,today.day,hour,minute,0,0)
            tgt_time += datetime.timedelta(1)
            print('tgt_time:' + tgt_time.strftime("%Y/%m/%d %H:%M:%S"))
            offset = tgt_time - today
            print('offset_sec:' + str(offset.seconds))

            if offset.seconds < 0:
                tim = await self.call_speaker(error_msg1)
                self.julius_power(True,tim)
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

            tim = await self.call_speaker(announce)
            self.julius_power(True,tim)
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
                tim = await self.call_speaker(error_msg1)
                self.julius_power(True,tim)
                return

            anc_list = announce.split("$")
            announce = ""
            for anc in anc_list:
                if anc == "onoff":
                    announce += onoff
                else:
                    announce += anc
            tim = await self.call_speaker(announce)
            self.julius_power(True,tim)

            res = requests.get(url)
            print(res.text)
        elif task_id == 8104:
            control_number = 0
            cmd = ""
            if "起こす" in words:
                control_number = 1
                cmd = "を起こし"
            elif "寝かせる" in words:
                control_number = 2
                cmd = "を寝かせ"
            elif "立てる" in words:
                control_number = 3
                cmd = "を立て"
            elif "倒す" in words:
                control_number = 4
                cmd = "を倒し"
            elif "上げる" in words:
                control_number = 7
                cmd = "の高さを上げ"
            elif "下げる" in words:
                control_number = 8
                cmd = "の高さを下げ"
            else:
                tim = await self.call_speaker(error_msg1)
                self.julius_power(True,tim)
                return
            anc_list = announce.split("$")
            announce = ""
            for anc in anc_list:
                if anc == "cmd":
                    announce += cmd
                else:
                    announce += anc
            tim = await self.call_speaker(announce)
            self.julius_power(True,tim)
            ws = create_connection('ws://192.168.4.131:9989')  # open socket
            ws.send(str(control_number))  # send to socket
            ws.close()  # close socket
            

        elif task_id == 8105:
            print(user_dic)
            if len(user_dic) == 1:
                user_id = user_dic.keys()[0]
                user_name = user_dic[user_id]
            elif len(user_dic) > 1:
                print("len(user_dic) > 1")
                #未実装
            else:
                # self.ask_remote(words, "get_health_condition")
                return

            place_id = 0
            place_name = ""
            temp_dbdata = Tmsdb()
            temp_dbdata.id = user_id
            temp_dbdata.state = 1

            #target = await self.call_dbreader(temp_dbdata)
            db_target = await self.call_dbreader(temp_dbdata)
            target = db_target[0]
            if target is None:
                tim = await self.call_speaker(error_msg2)
                self.julius_power(True,tim)
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
            tim = await self.call_speaker(announce)
            self.julius_power(True,tim)
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
                                # self.ask_remote(words, "robot_task",talk)
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
                                # self.ask_remote(words, "robot_task",talk)
                                return
                            else:
                                task_flag = 1
                        announce += object_name
                    elif anc == "user":
                        if len(person_dic) == 1:
                            user_id = person_dic.keys()[0]
                            user_name = person_dic[user_id]
                        elif len(person_dic) > 1:
                            print("len(person_dic) > 1")
                            #未実装

                        if user_id==0:
                            if i == len(task_announce_list) - 1:
                                # self.ask_remote(words, "robot_task",talk)
                                return
                            else:
                                task_flag = 1
                        announce += user_name
                    elif anc == "place":
                        if len(furniture_dic) == 1:
                            place_id = furniture_dic.keys()[0]
                            place_name = furniture_dic[place_id]
                        elif len(furniture_dic) > 1:
                            print("len(furniture_dic) > 1")
                            #未実装

                        if place_id==0:
                            if i == len(task_announce_list) - 1:
                                # self.ask_remote(words, "robot_task",talk)
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

                    tim = await self.call_speaker(announce)
                    self.julius_power(True,tim)
                    return


def main(args=None):
    rclpy.init(args=args)

    tms_ur_listener =TmsUrListenerServer()

    rclpy.spin(tms_ur_listener)

    tms_ur_listener.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
