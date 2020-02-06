from janome.tokenizer import Tokenizer
from pymongo import MongoClient
import pprint
import json
import re

from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
import rclpy
from tms_msg_ts.action import TsReq
from rclpy.action import ActionClient

from std_msgs.msg import String
from tms_msg_ts.srv import TaskTextRecognize
from tms_msg_ur.srv import SpeakerSrv

MONGODB_IPADDRESS = '192.168.4.119'
MONGODB_PORTNUMBER = 27017

class TmsUrTextRecognizer(Node):
    def __init__(self):
        super().__init__('task_text_recognizer')
        self.goal_handles = {}
        self.cb_group = ReentrantCallbackGroup()
        self.cli_ts_req = ActionClient(self, TsReq, 'tms_ts_master', callback_group=self.cb_group)
        while not self.cli_ts_req.wait_for_server(timeout_sec=1.0):
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

        if "キャンセル" in tags:
            futures = []
            for gh in self.goal_handles.values():
                print(str(gh.status))
                futures.append(gh.cancel_goal_async())
            # for future in futures:
            #     await futures
            announce_text = "キャンセルしました"
            await self.play_jtalk(announce_text)
            response.data = announce_text
            return response

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
                req = TsReq.Goal()
                req.task_id = task["id"]
                if self.arg_data:
                    req.arg_json = json.dumps(self.arg_data)
                announce_text = task["announce"]
                announce_text = re.sub(r"\((.*?)\)", self.re_func, announce_text)
                if request.is_announce:
                    await self.play_jtalk(announce_text)

                goal_handle = await self.cli_ts_req.send_goal_async(req)
                self.goal_handles[goal_handle.goal_id.uuid.tostring()] = goal_handle
                goal_handle.get_result_async().add_done_callback(self.done_callback)
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

    async def done_callback(self, future):
        print(f"Finish: {future.result().result.message}")
        msg = future.result().result.message
        # self.goal_handle_clients[goal_handle_client.goal_id.uuid.tostring()] = None
        # del self.goal_handle_clients[goal_handle_client.goal_id.uuid.tostring()]
        if msg != "Success":
            if msg == "Canceled":
                await self.play_jtalk("タスクがキャンセルされました")
            else:
                await self.play_jtalk("タスクが異常終了しました")

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
            {"$project": {
                "_id": 1,
                "id": 1,
                "name": 1,
                "require_tag": 1,
                "tag": 1,
                "require_tag_count": {"$size": "$require_tag"},
            }},
            {"$unwind": "$require_tag"},
            {"$match": {"require_tag": {"$in": tags}}},
            {"$group": {
                "_id": "$_id",
                "id": {"$first": "$id"},
                "name": {"$first": "$name"},
                "tag": {"$first": "$tag"},
                "require_tag_count": {"$first": "$require_tag_count"},
                "count": {"$sum": 1},
            }},
            {"$match": {"$expr": {"$eq": ["$count", "$require_tag_count"]}}},
            {"$unwind": {"path": "$tag", "preserveNullAndEmptyArrays": True}},
            {
                "$project": {
                    "_id": 1,
                    "id": 1,
                    "name": 1,
                    "tag": {"$ifNull": ["$tag", []]},
                    "m_tag": {"$ifNull": ["$tag", []]},
                }
            },
            {
                "$unwind": {
                    "path": "$tag",
                    "preserveNullAndEmptyArrays": True,
                }
            },
            {
                "$project":{
                    "_id": {"$concatArrays": [["$_id"], "$m_tag"]},
                    "id": 1,
                    "name": 1,
                    "tag": 1,
                    "m_tag": 1,
                    "tag_exist": {"$in": ["$tag", tags]}
                }
            },
            {
                "$group": {
                    "_id": "$_id",
                    "id": {"$first": "$id"},
                    "name": {"$first": "$name"},
                    "tag_exist": {"$addToSet": "$tag_exist"},
                }
            },
            {
                "$project": {
                    "_id": {"$arrayElemAt": ["$_id", 0]},
                    "id": 1,
                    "name": 1,
                    "tag_count": {"$cond": [{"$anyElementTrue": "$tag_exist"}, 1, 0]},
                },
            },
            {
                "$group": {
                    "_id": "$_id",
                    "id": {"$first": "$id"},
                    "name": {"$first": "$name"},
                    "count": {"$sum": "$tag_count"}
                }
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

    task_text_recognizer = TmsUrTextRecognizer()

    rclpy.spin(task_text_recognizer)

    rclpy.shutdown()


if __name__=='__main__':
    main()

