# Copyright 2023, IRVS Laboratory, Kyushu University, Japan.
# 
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
# 
#     http://www.apache.org/licenses/LICENSE-2.0
# 
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from janome.tokenizer import Tokenizer
from pymongo import MongoClient
import pprint
import json
import re
import rclpy

from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.action import ActionClient
from tms_msg_ts.action import TsReq
from tms_msg_ts.srv import TaskTextRecognize

MONGODB_IPADDRESS = '127.0.0.1'
MONGODB_PORTNUMBER = 27017

class TmsUrTextRecognizer(Node):
    def __init__(self):
        super().__init__('task_text_recognizer')
        self.goal_handles = {}
        self.cb_group = ReentrantCallbackGroup()
        self.cli_ts_req = ActionClient(self, TsReq, 'tms_ts_master', callback_group=self.cb_group)
        while not self.cli_ts_req.wait_for_server(timeout_sec=1.0):
            self.get_logger().info('service "tms_ts_master" not available, waiting again...')
        self.srv = self.create_service(TaskTextRecognize, "tms_ts_text_recognizer", self.tms_ts_text_callback, callback_group=self.cb_group)


    async def tms_ts_text_callback(self, request, response):
        tags = self.tokenize(request.data)
        self.arg_data = {}
        tms_objects = self.tms_objects_search(tags)
        self.get_logger().info(str(tms_objects))
        tags = set(tags)  # タグが重複しないように一旦setにする
        for tms_object in tms_objects:  # Mongodbの1行分のデータがtms_objectに対応
            del tms_object["_id"]
            self.arg_data[tms_object["type"]] = tms_object
            tags.add("("+tms_object["type"]+")")
        tags = list(tags)  # setからlistに戻す

        if "キャンセル" in tags:
            futures = []
            for gh in self.goal_handles.values():
                self.get_logger().info(str(gh.status))
                futures.append(gh.cancel_goal_async())
            text = "キャンセルしました"
            response.data = text
            return response

        task = self.task_search(tags)
        text = ""

        if(task != None):  # タスクが存在するとき
            _is_valid_task_argument = True
            for token in  task.get("tokens", []):
                if token not in self.arg_data:
                    _is_valid_task_argument = False
            
            if _is_valid_task_argument:  # タスクが存在し、かつ引数がすべて揃っているとき
                req = TsReq.Goal()
                req.task_id = task["id"]
                if self.arg_data:
                    req.arg_json = json.dumps(self.arg_data)
                text = task["announce"]
                text = re.sub(r"\((.*?)\)", self.re_func, text) # textの正規表現部分を取り出して、re_funcの戻り値で置換
                goal_handle = await self.cli_ts_req.send_goal_async(req)
                self.goal_handles[goal_handle.goal_id.uuid.tostring()] = goal_handle
                goal_handle.get_result_async().add_done_callback(self.done_callback)
                self.get_logger().info(f"Call task {task['id']}")
            else:   # タスクが存在するが、引数がすべて揃っていないとき
                if request.is_announce:
                    text = task.get("error_announce", None)
                self.get_logger().info(f"Lack of arguments for task {task['id']}")

        else:  # タスクが存在しないとき
            self.get_logger().info("There are no task.")
            text ="There are no task."
        
        response.data = text
        return response


    async def done_callback(self, future):
        msg = future.result().result.message
        if msg != "Success":
            if msg == "Canceled":
                self.get_logger().info("タスクがキャンセルされました")
            else:
                self.get_logger().info("タスクが異常終了しました")


    def tokenize(self, sentence):
        search_tags = []
        t = Tokenizer()
        for token in t.tokenize(sentence):
            self.get_logger().info(str(token))
        tokens = t.tokenize(sentence)
        i = 0
        #base_form: 基本形, surface: 表層形, part_of_speech: 品詞
        for token in tokens:
            self.get_logger().info(str(token))
            if token.part_of_speech.split(',')[0] == u'動詞':
                search_tags.append(token.base_form)
            elif token.part_of_speech.split(',')[0] == u'名詞' and token.part_of_speech.split(',')[1] == u'接尾':
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
        cursor = db.default.aggregate([
            {"$match": {"type": "task"}}, # $match : 条件に合致するフィールドを出力
            {"$project": {                # $project : 出力するフィールドを指定 (0：OFF , 1：ON)
                "_id": 1,
                "id": 1,
                "name": 1,
                "require_tag": 1,
                "tag": 1,
                "require_tag_count": {"$size": "$require_tag"}, # $size : 配列の要素数を指定
            }},
            {"$unwind": "$require_tag"},                # $unwind : 配列を展開
            {"$match": {"require_tag": {"$in": tags}}}, # $in : 配列に含まれるかどうか判定
            {"$group": {                        # $group : ドキュメントをグループ化("_id","id","name","tag","require_tag_count","count"を条件指定してグループ化)
                "_id": "$_id",
                "id": {"$first": "$id"},        # $first : グループ化したドキュメントの最初の値を指定
                "name": {"$first": "$name"},
                "tag": {"$first": "$tag"},
                "require_tag_count": {"$first": "$require_tag_count"},
                "count": {"$sum": 1},           # $sum : グループ化したドキュメントの合計値を指定
            }},
            {"$match": {"$expr": {"$eq": ["$count", "$require_tag_count"]}}},  # $expr : ドキュメント内のフィールドを比較
            {"$unwind": {"path": "$tag", "preserveNullAndEmptyArrays": True}}, #$eq : ドキュメント内のフィールドを比較(等号)
            {"$project": {
                    "_id": 1,
                    "id": 1,
                    "name": 1,
                    "tag": {"$ifNull": ["$tag", []]},  # $ifNull : $tagが存在する場合には"$tag"を、そうでない場合には[]を返す
                    "m_tag": {"$ifNull": ["$tag", []]},
            }},
            {"$unwind": {
                    "path": "$tag",
                    "preserveNullAndEmptyArrays": True,
            }},
            {"$project":{
                    "_id": {"$concatArrays": [["$_id"], "$m_tag"]},
                    "id": 1,
                    "name": 1,
                    "tag": 1,
                    "m_tag": 1,
                    "tag_exist": {"$in": ["$tag", tags]}
            }},
            {"$group": {
                    "_id": "$_id",
                    "id": {"$first": "$id"},
                    "name": {"$first": "$name"},
                    "tag_exist": {"$addToSet": "$tag_exist"},
            }},
            {"$project": {
                    "_id": {"$arrayElemAt": ["$_id", 0]},
                    "id": 1,
                    "name": 1,
                    "tag_count": {"$cond": [{"$anyElementTrue": "$tag_exist"}, 1, 0]},
            },
            },
            {"$group": {
                    "_id": "$_id",
                    "id": {"$first": "$id"},
                    "name": {"$first": "$name"},
                    "count": {"$sum": "$tag_count"}
            }},
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
    

    #オブジェクトの判別 + データ取得用関数(143:bed, 145:entrance, 146:kitchen, 147:TV room, 150:sensor)
    def tms_objects_search(self,tags):
        client = MongoClient(MONGODB_IPADDRESS, MONGODB_PORTNUMBER)
        db = client.rostmsdb
        cursor = db.default.aggregate([
            {"$match": {"call_name": {"$in": tags}}},
            {"$sort": {"id": 1}},
        ])
        document_array = []
        for doc in cursor:
            document_array.append(doc)
        return document_array


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