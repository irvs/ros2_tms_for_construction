"""
The Original Code is mongodb_store package's util.py
http://www.ros.org/wiki/mongodb_store
The Initial Developers of the Original Code are Chris Burbridge and Nick Hawes.
This file is licensed under the MIT License.
This file is modified by Minsoo Song.
"""

import rclpy
import pymongo
import json
import copy
from rclpy.node import Node
from rclpy.callback_groups import CallbackGroup
from std_msgs.msg import String
from rclpy.executors import Executor
from bson import json_util
from bson.objectid import ObjectId


def msg_to_document(msg):
    msg_dict = {}
    slot_types = []
    # print(msg)
    
    if hasattr(msg, 'slot_types'):
        slot_types = msg.slot_types
    else:
        slot_types = [None] * len(msg.__slots__)          
    # print(msg.__slots__)
    for (attr, type) in zip(msg.__slots__, slot_types):
        # print(msg.__slots__)
        result = zip(msg.__slots__, slot_types)

        resultSet = set(result)
        # print(resultSet)
        # print(attr)
        msg_dict[attr[1:]] = sanitize_value(attr, getattr(msg, attr), type)
        
    return msg_dict
    

def sanitize_value(attr, v, type):
    # if isinstance(v, str):
    #     #if type == 'uint8[]':
    #     #     v = Binary(v)
    #     # else:
    #     #     try:
    #     #         v = unicode(v, "utf-8")
    #     #     except UnicodeDecodeError as e:
    #     #         v = Binary(v)
    #     v = str(v)

    #     return v

    # if isinstance(v, rclpy.Message):
    #     return msg_to_document(v)
    # if isinstance(v, genpy.rostime.Time):
    #     return msg_to_document(v)
    # elif isinstance(v, genpy.rostime.Duration):
    #     return msg_to_document(v)
    if isinstance(v, list):
        result = []
        for t in v:
            if hasattr(t, 'type'):
                result.append(sanitize_value(None, t, t.type))
            else:
                result.append(sanitize_value(None, t, None))
        return result
    else:
        
        return v


def check_connection(db_host, db_port):
    try:
        from pymongo import Connection
        Connection(db_host, db_port)
        rclpy.loginfo("Connected to the ROS-TMS Database!")
        #rclpy.shutdown()
        return True
    except Exception as e:
        #self.get_logger().info('gError: "%s"' % str(e))
        #self.get_logger().info('Could not connect to mongo server "%s":"%d"' % (db_host, db_port))
        #rclpy.shutdown()
        return False


def document_to_msg(document, TYPE):
    msg = TYPE()
    _fill_msg(msg, document)
    return msg


def _fill_msg(msg, dic):
    for i in dic:
        if isinstance(dic[i], dict):
            
            _fill_msg(getattr(msg, i), dic[i])


        else:
            if i == "time":
                 dic[i] = str(dic[i])
            elif i == "id":
                dic[i] = int(dic[i])
            elif i == "name":
                dic[i] = str(dic[i])
            elif i == "x":
                dic[i] = float(dic[i])
            elif i == "y":
                dic[i] = float(dic[i])
            elif i == "z":
                dic[i] = float(dic[i])
            elif i == "rr":
                dic[i] = float(dic[i])
            elif i == "rp":
                dic[i] = float(dic[i])
            elif i == "ry":
                dic[i] = float(dic[i])
            elif i == "offset_x":
                dic[i] = float(dic[i])
            elif i == "offset_y":
                dic[i] = float(dic[i])
            elif i == "offset_z":
                dic[i] = float(dic[i])
            elif i == "joint":
                dic[i] = str(dic[i])
            elif i == "weight":
                dic[i] = float(dic[i])
            elif i == "rfid":
                dic[i] = str(dic[i])
            elif i == "etcdata":
                dic[i] = str(dic[i])
            elif i == "place":
                dic[i] = int(dic[i])
            elif i == "extfile":
                dic[i] = str(dic[i])
            elif i == "sensor":
                dic[i] = int(dic[i])
            elif i == "probability":
                dic[i] = float(dic[i])
            elif i == "state":
                dic[i] = int(dic[i])
            elif i == "task":
                dic[i] = str(dic[i])
            elif i == "note":
                dic[i] = str(dic[i])
            elif i == "tag":
                dic[i] = str(dic[i])
            elif i == "announce":
                dic[i] = str(dic[i])

            if i != "require_tag" and i != "noun" and i != "error_announce" and i != "tokens":
                setattr(msg, i, dic[i])
