from janome.tokenizer import Tokenizer
from pymongo import MongoClient
import pprint

from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
import rclpy
from tms_msg_ts.srv import TsReq
from std_msgs.msg import String

MONGODB_IPADDRESS = '192.168.4.119'
MONGODB_PORTNUMBER = 27017

class TaskTextRecognizer(Node):
    def __init__(self):
        super().__init__('task_text_recognizer')
        self.cb_group = ReentrantCallbackGroup()
        self.cli_ts_req = self.create_client(TsReq, 'tms_ts_master', callback_group=self.cb_group)
        while not self.cli_ts_req.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service "tms_ts_master" not available, waiting again...')
        self.subscriber = self.create_subscription(String, 'tms_ts_text_recognizer', self.tms_ts_text_callback, callback_group=self.cb_group)
        
    def tms_ts_text_callback(self, msg):
        tags = self.tokenize(msg.data)
        tasks = self.task_search(tags)

        if(len(tasks) >= 1):
            task = tasks[0]
            req = TsReq.Request()
            req.task_id = task["id"]
            self.cli_ts_req.call_async(req)
            self.get_logger().info(f"Call task {task['id']}")
        else:
            self.get_logger().info("There are no task.")


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
        
        for search_tag in search_tags:
            self.get_logger().info(str(search_tag))
        
        return search_tags
    
    def task_search(self,tags):
        client = MongoClient(MONGODB_IPADDRESS, MONGODB_PORTNUMBER)

        db = client.rostmsdb

        self.get_logger().info("\n[tasks]")
        cursor = db.default.aggregate([
            { '$sample' : {'size': 5} }
        ])
        cursor = db.default.aggregate([
            {"$match": {"type": "task"}},
            {"$unwind": "$require_tag"},
            {"$match": {"require_tag": {"$in": tags}}},
            {"$group": {
                "_id": "$_id",
                "id": {"$addToSet": "$id"},
                "name": {"$addToSet": "$name"},
                "type": {"$addToSet": "$type"},
                "etcdata": {"$addToSet": "$etcdata"},
                "note": {"$addToSet": "$note"},
                "announce": {"$addToSet": "$announce"},
                "count": {"$sum": 1},
            }},
            {
                "$project": {"_id": 0,
                    "id": {"$arrayElemAt": ["$id", 0]},
                    "name": {"$arrayElemAt": ["$name", 0]},
                    "type": {"$arrayElemAt": ["$type", 0]},
                    "etcdata": {"$arrayElemAt": ["$etcdata", 0]},
                    "note": {"$arrayElemAt": ["$note", 0]},
                    "announce": {"$arrayElemAt": ["$announce", 0]},
                    "count": 1,
                },
            },
            {"$sort": {"count": -1, "id": 1}},
        ])
        document_array = []
        for doc in cursor:
            document_array.append(doc)

        pprint.pprint(document_array)
        return document_array

def main(args=None):
    rclpy.init(args=args)

    task_text_recognizer = TaskTextRecognizer()

    rclpy.spin(task_text_recognizer)

    rclpy.shutdown()


if __name__=='__main__':
    main()

