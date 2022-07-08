import json
from rclpy.node import Node
from tms_msg_db.msg import Tmsdb
from tms_msg_db.srv import TmsdbGetData
import rclpy
import pymongo
import tms_db_manager.tms_db_util as db_util


class TmsDbReader(Node):
    """Read data from MongoDB."""
  
    def __init__(self):
        super().__init__('tms_db_reader')

        # declare parameter
        self.declare_parameter('db_host', 'localhost')
        self.declare_parameter('db_port', 27017)

        # get parameter
        self.db_host = self.get_parameter('db_host').get_parameter_value().string_value
        self.db_port = self.get_parameter('db_port').get_parameter_value().integer_value

        self.db  = db_util.connect_db('rostmsdb', self.db_host, self.db_port)
        self.srv = self.create_service(TmsdbGetData, 'tms_db_reader', self.db_reader_srv_callback)


    def db_reader_srv_callback(self, request, response):
        """
        Respond to requests from client nodes.

        Parameters
        ----------
        request
            Request from client node.
        response
            Response to client node.

        Returns
        -------
        response
            Response to client node.
        """
        collection = self.db[request.type]

        if request.latest_only:
            latest_data = self.get_latest_data(request, collection)
            response.tmsdbs.append(self.allocate_tmsdb(latest_data))
            return response
        else:
            all_data = self.get_all_data(request, collection)
            for data in all_data:
                msg = self.allocate_tmsdb(data)
                response.tmsdbs.append(msg)
            return response

    
    def get_latest_data(self, request, collection) -> dict:
        """
        Get latest data only.

        Parameters
        ----------
        request
            Request from a client node.
        collection
            MongoDB's target collection.

        Returns
        -------
        dict
            Requested latest data.
        """
        latest_data = collection.find_one(
            {'id': request.id},
            sort=[('time', pymongo.DESCENDING)]
        )
        return latest_data


    def get_all_data(self, request, collection) -> pymongo.cursor.Cursor:
        """
        Get all data.

        Parameters
        ----------
        request
            Request from a client node.
        collection
            MongoDB's target collection.

        Returns
        -------
        pymongo.cursor.Cursor
            Requested all data.
        """
        all_data = collection.find({'id': request.id}).sort([('time', pymongo.ASCENDING)])
        return all_data


    def allocate_tmsdb(self, data: dict) -> Tmsdb:
        """
        Allocate dictionary data to Tmsdb msg.

        Parameters
        ----------
        dict : data
            Dictionary data.

        Returns
        -------
        Tmsdb
            Tmsdb msg data.
        """
        tmsdb = Tmsdb()
        tmsdb.time      = data['time']
        tmsdb.type      = data['type']
        tmsdb.id        = data['id']
        tmsdb.name      = data['name']
        tmsdb.is_insert = data['is_insert']
        tmsdb.msg       = json.dumps(data['msg'])
        return tmsdb


def main(args=None):
    rclpy.init(args=args)
    tms_db_reader = TmsDbReader()
    rclpy.spin(tms_db_reader) 

    tms_db_reader.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
	main()
