"""
The Original Code is mongodb_store package's util.py
http://www.ros.org/wiki/mongodb_store
The Initial Developers of the Original Code are Chris Burbridge and Nick Hawes.
This file is licensed under the MIT License.
This file is modified by Minsoo Song and Ryuichi Maeda.
"""

import array
import numpy
import rclpy
import pymongo
from bson import Binary


def connect_db(db_name: str, db_host: str = 'localhost', db_port: int = 27017) -> pymongo.database.Database:
    """
    Connect to MongoDB database.

    Parameters
    ----------
    db_name : str
        Target database name.

    Returns
    -------
    pymongo.database.Database
        Connected database.
    """
    client = pymongo.MongoClient(host=db_host, port=db_port)
    db = client[db_name]
    return db


def msg_to_document(msg):
    """
    Convert supplied ROS msg to dictionary.

    Parameters
    ----------
    msg : Any
        ROS msg.

    Returns
    -------
    dict
        Dictionary converted from supplied ROS msg.
    """
    msg_dict = {}
    slot_types = []
    
    if hasattr(msg, 'slot_types'):
        slot_types = msg.slot_types
    else:
        slot_types = [None] * len(msg.__slots__)

    for (attr, type) in zip(msg.__slots__, slot_types):
        msg_dict[attr[1:]] = sanitize_value(getattr(msg, attr), type)
        
    return msg_dict
    

def sanitize_value(v, type):
    """
    Sanitize supplied value.

    Parameters
    ----------
    val : Any
        Value to be sanitized.

    Returns
    -------
    Any
        Sanitized value.
    """

    if isinstance(v, str) and type == 'uint8[]':
        return Binary(v)

    if isinstance(v, object) and hasattr(v, '__slots__'):
        return msg_to_document(v)

    if isinstance(v, numpy.ndarray) or isinstance(v, array.array):
        v = v.tolist()

    if isinstance(v, list):
        result = []
        for t in v:
            if hasattr(t, '_type'):
                result.append(sanitize_value(t, t._type))
            else:
                result.append(sanitize_value(t, None))
        return result
        
    return v


def check_connection(db_host: str, db_port: int) -> bool:
    """
    Check connection to mongod server.

    Parameters
    ----------
    db_host : str
        mongod host.
    db_port : int
        mongod port.

    Returns
    -------
    bool
        Result of whether the connection has been made or not.
    """
    try:
        pymongo.MongoClient(db_host, db_port)
        rclpy.loginfo("Connected to the ROS2-TMS Database!")
        return True
    except:
        return False


# TODO fix for using json.
def document_to_msg(document, TYPE):
    msg = TYPE()
    _fill_msg(msg, document)
    return msg


# TODO fix for using json.
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


def get_collection_by_id(db: pymongo.database.Database, id: int) -> pymongo.collection.Collection:
    """
    Get collection based on ID.

    Parameters
    ----------
    db : pymongo.database.Database
        Targent MongoDB database.
    id : int
        Target ID.

    Returns
    -------
    pymongo.collection.Collection
        MongoDB collection.
    """
    if id >= 11000:
        return db['machine']
    elif id >= 10000:
        return db['state']
    elif id >= 9000:
        return db['subtask']
    elif id >= 8000:
        return db['task']
    elif id >= 7000:
        return db['object']
    elif id >= 6000:
        return db['furniture']
    elif id >= 5000:
        return db['space']
    elif id >= 4000:
        return db['structure']
    elif id >= 3000:
        return db['sensor']
    elif id >= 2000:
        return db['robot']
    else:
        return db['person']
