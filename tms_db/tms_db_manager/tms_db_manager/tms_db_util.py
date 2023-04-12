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

from sensor_msgs.msg import PointCloud2, PointField


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


def msg_to_document(msg: object) -> dict:
    """
    Convert supplied ROS msg to dictionary.

    Parameters
    ----------
    msg : object
        ROS msg.

    Returns
    -------
    dict
        Dictionary converted from supplied ROS msg.
    """
    msg_dict = {}
    for attr in msg.__slots__:
        msg_dict[attr[1:]] = sanitize_value(getattr(msg, attr))
        
    return msg_dict
    

def sanitize_value(val):
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
    if isinstance(val, object) and hasattr(val, '__slots__'):
        return msg_to_document(val)

    if isinstance(val, numpy.ndarray) or isinstance(val, array.array):
        val = val.tolist()

    if isinstance(val, list):
        result = []
        for t in val:
            result.append(sanitize_value(t))
        return result
        
    return val


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


def document_to_msg(document: dict, TYPE: object) -> object:
    """
    Convert document to ROS msg.

    Parameters
    ----------
    document : dict
        Data fetched from MongoDB.
    TYPE : object
        ROS msg type.

    Returns
    -------
    object
        ROS msg.
    """
    msg = TYPE()
    _fill_msg(msg, document)
    return msg


def _fill_msg(msg: object, dic: dict) -> None:
    """
    Filling ROS msg by given dictionary.

    Parameters
    ----------
    msg : object
        ROS msg type.
    dic : dict
        dictionary data.
    """
    for i in dic:
        if isinstance(dic[i], dict):
            _fill_msg(getattr(msg, i), dic[i])
        else:
            attr = getattr(msg, i)
            attr_type = type(attr)

            if attr_type in [type(dic[i]), array.array]:
                if i == 'fields' and type(msg) is PointCloud2:
                    # this is used for sensor_msgs.msg.PointField
                    fields = [PointField(name=field['name'], offset=field['offset'], datatype=field['datatype'], count=field['count']) for field in dic[i]]
                    setattr(msg, i, fields)
                else:
                    setattr(msg, i, dic[i])
            elif attr_type is numpy.ndarray:
                setattr(msg, i, numpy.asarray(dic[i], dtype=attr.dtype))
            else:
                setattr(msg, i, attr_type(dic[i]))

def set_time_index(collection: pymongo.collection.Collection) -> None:
    """
    Set time index for collection.

    Parameters
    ----------
    collection : pymongo.collection.Collection
        Target collection.
    """
    time_index = [('time', pymongo.DESCENDING)]
    index_list = collection.list_indexes()
    for index in index_list:
        if index['key'] == time_index:
            return
        
    collection.create_index(time_index)