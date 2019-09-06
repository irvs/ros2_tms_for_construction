import os

import sys
sys.path.append("/home/ros_tms/tms_db/tms_db_manager/scripts")
from example_interfaces.srv import AddTwoInts
import rclpy
import json
import copy
from bson import json_util
from bson.objectid import ObjectId
from datetime import *

import pymongo
from rclpy.executors import Executor
from rclpy.callback_groups import CallbackGroup
from rclpy.node import Node
from std_msgs.msg import String
from tms_msg_db.msg import TmsdbStamped, Tmsdb
from tms_msg_db.srv import *
import scripts.tms_db_util as db_util



client = pymongo.MongoClient('localhost:27017')
db = client.rostmsdb

class TmsDbWriter(Node):
    def __init__(self):
        super().__init__("tms_db_writer")
        db_host = 'localhost'
        db_port = 27017
        # self.is_connected = db_util.check_connection(db_host, db_port)
        # if not self.is_connected:
        #     raise Exception("Problem of connection")
        self.sub = self.create_subscription(TmsdbStamped, "tms_db_data",  self.dbWriteCallback)
        self.writeInitData()

    def dbWriteCallback(self, msg):
        # rclpy.loginfo("writing the one msg")
        for tmsdb in msg.tmsdb:
            try:
                doc = db_util.msg_to_document(tmsdb)
                print(doc)
                # if sys.argv[1] =="true":
                #      db.history.insert(doc)
                #      result = db.now.find({"name": doc['name'], "sensor": doc['sensor']})
                #      if result.count() >= 1:
                #          del doc['id']
                result = db.now.update(
                    {"name": doc['name'], "sensor": doc['sensor']},
                    doc,
                    upsert=True
                )
                print(result)
            except ValueError as e:
                print("ServiceException: %s" % e)

    def writeInitData(self):
        cursor = db.default.find({"$or": [{"type": "furniture"}, {"type": "robot"}]})
        for doc in cursor:
            if '_id' in doc:
                del doc['_id']

            result = db.now.update(
                {"name": doc['name']},
                doc,
                upsert=True
            )
            # print(result)
        # rclpy.loginfo("Writed the init data using collection of default.")

    # def shutdown(self):
        # rclpy.loginfo("Stopping the node")

def main(args=None):
    if args is None:
        args = sys.argv

    rclpy.init(args=args)

    node = TmsDbWriter()
    rclpy.spin(node) 

if __name__ == '__main__':
    main()
    # try:
    #     TmsDbWriter()
    #     rclpy.spin()
    # except rclpy.Exception:
    #     rclpy.loginfo("tms_db_writer node terminated.")

# test topic message
# rostopic pub /tms_db_data tms_msg_db/TmsdbStamped "header:
#   seq: 0
#   stamp:
#     secs: 0
#     nsecs: 0
#   frame_id: ''
# tmsdb:
# - {time: '', type: '', id: 0, name: '', x: 0.0, y: 0.0, z: 0.0, rr: 0.0, rp: 0.0,
#   ry: 0.0, offset_x: 0.0, offset_y: 0.0, offset_z: 0.0, joint: '', weight: 0.0, rfid: '',
#   etcdata: '', place: 0, extfile: '', sensor: 0, probability: 0.0, state: 0, task: '',
#   note: '', tag: ''}"
