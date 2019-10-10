import os

import sys
sys.path.append("/home/ros_tms/tms_db/tms_db_manager/scripts")
from example_interfaces.srv import AddTwoInts
import rclpy
import json
import copy
import threading
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


class TmsDbPublisher(Node):

    def __init__(self):
        super().__init__("tms_db_publisher")

        db_host = 'localhost'
        db_port = 27017
        self.is_connected = db_util.check_connection(db_host, db_port)
        # if not self.is_connected:
        #     raise Exception("Problem of connection")

        self.data_pub = self.create_publisher(TmsdbStamped, 'tms_db_publisher')
        
        self.dbPublisherSrvCallback()

    def dbPublisherSrvCallback(self):
        threading.Timer(0.01, self.dbPublisherSrvCallback).start()
        temp_dbdata = Tmsdb()
        current_environment_information = TmsdbStamped()
        cursor = db.now.find()
        for doc in cursor:
            del doc['_id']
            temp_dbdata = db_util.document_to_msg(doc, Tmsdb)
            current_environment_information.tmsdb.append(temp_dbdata)
        self.data_pub.publish(current_environment_information)
        

def main(args=None):
    if args is None:
        args = sys.argv

    rclpy.init(args=args)

    node = TmsDbPublisher()
    node.dbPublisherSrvCallback()

    rclpy.spin(node) 

if __name__ == '__main__':
    #TmsDbReader()
	#rclpy.spin() 
	main()
      
      
      # rate = rclpy.Rate(100)  # 100hz

        # while not rclpy.is_shutdown():
            # temp_dbdata = Tmsdb()
            # current_environment_information = TmsdbStamped()

        #     # cursor = db.now.find({'$or': [{'state': 1}, {'state': 2}]})
            # cursor = db.now.find()
        #     # print(cursor.count())
            # for doc in cursor:
            #     del doc['_id']
            #     temp_dbdata = db_util.document_to_msg(doc, Tmsdb)
            #     current_environment_information.tmsdb.append(temp_dbdata)
        #     # rospy.loginfo("send db data!")
            # self.data_pub.publish(current_environment_information)

        #     rate.sleep()

    # def shutdown(self):
    #     super().__init__("Stopping the node")
