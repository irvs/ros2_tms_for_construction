from janome.tokenizer import Tokenizer
from pymongo import MongoClient
import pprint
import json
import re

from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
import rclpy
from tms_msg_ts.srv import TsReq
from std_msgs.msg import String
from tms_msg_ts.srv import TaskTextRecognize
from tms_msg_ur.srv import SpeakerSrv

MONGODB_IPADDRESS = '192.168.4.119'
MONGODB_PORTNUMBER = 27017

class TaskTextRecognizer(Node):
    def __init__(self):
        super().__init__('task_text_recognizer')
        self.cb_group = ReentrantCallbackGroup()
        self.cli_ts_req = self.create_client(TsReq, 'tms_ts_master', callback_group=self.cb_group)
        while not self.cli_ts_req.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service "tms_ts_master" not available, waiting again...')
        #self.subscriber = self.create_subscription(String, 'tms_ts_text_recognizer', self.tms_ts_text_callback, callback_group=self.cb_group)
        self.srv = self.create_service(TaskTextRecognize, "tms_ts_text_recognizer", self.tms_ts_text_callback, callback_group=self.cb_group)

    async def tms_ts_text_callback(self, request, response):
        tags = self.tokenize(request.data)
        self.arg_data = {}
        tms_objects = self.tms_objects_search(tags)

        tags = set(tags)  # タグが重複しないように一旦setにする
        for tms_object in tms_objects:
            del tms_object["_id"]
            self.arg_data[tms_object["type"]] = tms_object
            tags.add("("+tms_object["type"]+")")
        print(self.arg_data)
        tags = list(tags)  # setからlistに戻す

        task = self.task_search(tags)
        announce_text = ""

        if(task != None):  # タスクが存在するとき
            # task = tasks[0]

            _is_valid_task_argument = True
            print(task.get("tokens", []))
            for token in  task.get("tokens", []):
                if token not in self.arg_data:
                    _is_valid_task_argument = False
            
            if _is_valid_task_argument:  # タスクが存在し、かつ引数がすべて揃っているとき
                req = TsReq.Request()
                req.task_id = task["id"]
                if self.arg_data:
                    req.data = json.dumps(self.arg_data)
                announce_text = task["announce"]
                announce_text = re.sub(r"\((.*?)\)", self.re_func, announce_text)
                if request.is_announce:
                    await self.play_jtalk(announce_text)

                self.cli_ts_req.call_async(req)
                self.get_logger().info(f"Call task {task['id']}")
            else:   # タスクが存在するが、引数がすべて揃っていないとき
                if request.is_announce:
                    announce_text = task.get("error_announce", None)
                    if announce_text != None:
                        await self.play_jtalk(announce_text)
                    
                self.get_logger().info(f"Lack of arguments for task {task['id']}")

        else:  # タスクが存在しないとき
            self.get_logger().info("There are no task.")
            announce_text =""
        
        response.data = announce_text
        return response


    def tokenize(self, sentence):
        search_tags = []
        t = Tokenizer()
        self.get_logger().info("\n[tokens]")
        for token in t.tokenize(sentence):
            self.get_logger().info(str(token))
        
        self.get_logger().info("\n[search_tags]")
        tokens = t.tokenize(sentence)
        i = 0
        for token in tokens:
            if token.part_of_speech.split(',')[0] == u'動詞':
                search_tags.append(token.base_form)
            elif token.part_of_speech.split(',')[0] == u'名詞' and \
                token.part_of_speech.split(',')[1] == u'接尾':
                search_tags.append(tokens[i-1].surface + token.surface)
            elif token.part_of_speech.split(',')[0] == u'名詞':
                if token.base_form != '*':
                    search_tags.append(token.base_form)
                else:
                    search_tags.append(token.surface)
            i += 1
        
        search_tags.append(sentence)
        for search_tag in search_tags:
            self.get_logger().info(str(search_tag))
        
        return search_tags
    
    def task_search(self,tags):
        client = MongoClient(MONGODB_IPADDRESS, MONGODB_PORTNUMBER)

        db = client.rostmsdb

        self.get_logger().info("\n[tasks]")

        cursor = db.default.aggregate([
            {"$match": {"type": "task"}},
            {"$unwind": "$require_tag"},
            {"$match": {"require_tag": {"$in": tags}}},
            {"$group": {
                "_id": "$_id",
                "id": {"$addToSet": "$id"},
                # "name": {"$addToSet": "$name"},
                # "type": {"$addToSet": "$type"},
                # "etcdata": {"$addToSet": "$etcdata"},
                # "note": {"$addToSet": "$note"},
                # "announce": {"$addToSet": "$announce"},
                "count": {"$sum": 1},
            }},
            {
                "$project": {"_id": 0,
                    "id": {"$arrayElemAt": ["$id", 0]},
                    # "name": {"$arrayElemAt": ["$name", 0]},
                    # "type": {"$arrayElemAt": ["$type", 0]},
                    # "etcdata": {"$arrayElemAt": ["$etcdata", 0]},
                    # "note": {"$arrayElemAt": ["$note", 0]},
                    # "announce": {"$arrayElemAt": ["$announce", 0]},
                    "count": 1,
                },
            },
            {"$sort": {"count": -1, "id": 1}},
        ])
        document_array = []
        for doc in cursor:
            document_array.append(doc)
        pprint.pprint(document_array)
        if len(document_array) == 0:
            return None
        task = db.default.find_one({"id": document_array[0]["id"]})
        return task
    
    def tms_objects_search(self,tags):
        client = MongoClient(MONGODB_IPADDRESS, MONGODB_PORTNUMBER)
        db = client.rostmsdb
        cursor = db.default.aggregate([
            # {"$match": {"type": "room_place"}},
            {"$match": {"call_name": {"$in": tags}}},
            {"$sort": {"id": 1}},
        ])
        document_array = []
        for doc in cursor:
            document_array.append(doc)

        pprint.pprint(document_array)
        return document_array

    
    async def play_jtalk(self, t):
        self.cli_speaker = self.create_client(SpeakerSrv, 'speaker_srv', callback_group=ReentrantCallbackGroup())
        while  not self.cli_speaker.wait_for_service(1.0):
            self.get_logger().info('service "speaker_srv" not found...')
        req = SpeakerSrv.Request()
        req.data = t
        await self.cli_speaker.call_async(req)
    # def place_search(self, tags):
    #     client = MongoClient(MONGODB_IPADDRESS, MONGODB_PORTNUMBER)
    #     db = client.rostmsdb
    #     cursor = db.

    def re_func(self,m):
        arg_str = m.groups()[0]
        args = arg_str.split('.')
        if self.arg_data is None:
            return 'エラー'
        
        answer = self.arg_data.copy()
        for arg in args:
            answer = answer.get(arg, {})
        if answer == {}:
            return 'エラー'
        else:
            return str(answer)

def main(args=None):
    rclpy.init(args=args)

    task_text_recognizer = TaskTextRecognizer()

    rclpy.spin(task_text_recognizer)

    rclpy.shutdown()


if __name__=='__main__':
    main()

